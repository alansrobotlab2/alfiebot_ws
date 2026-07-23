#include "ros_interface.h"
#include "driverboard.h"
#include "servo_control.h"
#include <rmw_microros/rmw_microros.h>

extern DriverBoard b;

// =============================================================================
// micro-ROS entities
// =============================================================================
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_subscription_t head_subscriber;
static rcl_publisher_t    state_publisher;

static alfie_msgs__msg__HeadCmd   head_cmd_msg;
static alfie_msgs__msg__HeadState head_state_msg;

// =============================================================================
// State
// =============================================================================
RosAgentState_t agent_state         = WAITING_AGENT;
bool            ros_entities_created = false;
bool            micro_ros_initialized = false;

// =============================================================================
// Command decode: ServoCmd (SI) -> mBuf register mirror
// =============================================================================
static void applyServoCmd(uint8_t index, const alfie_msgs__msg__ServoCmd *cmd)
{
    b.mBuf[index].memory.torqueSwitch  = cmd->enabled ? 1 : 0;
    b.mBuf[index].memory.targetLocation = radToCount(index, cmd->target_location);
    b.mBuf[index].memory.runningSpeed  = radPerSecToSpeed(cmd->target_speed);
    b.mBuf[index].memory.acceleration  = radPerSec2ToAccel(cmd->target_acceleration);

    float torque = cmd->target_torque;
    if (torque < 0.0f)    torque = 0.0f;
    if (torque > 1000.0f) torque = 1000.0f;
    b.mBuf[index].memory.torqueLimit = (uint16_t)torque;
}

void headCmdCallback(const void *msgin)
{
    const alfie_msgs__msg__HeadCmd *msg = (const alfie_msgs__msg__HeadCmd *)msgin;

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        applyServoCmd(i, &msg->servos[i]);
    }

    b.eye_pwm[EYE_LEFT]  = msg->eye_pwm[0];
    b.eye_pwm[EYE_RIGHT] = msg->eye_pwm[1];

    b.new_head_command = true;
    b.last_cmd_time    = millis();
}

// =============================================================================
// State encode: mBuf register mirror -> ServoState (SI)
// =============================================================================
static void fillServoState(uint8_t index, alfie_msgs__msg__ServoState *st)
{
    const MemoryStruct &m = b.mBuf[index].memory;

    st->enabled             = (m.torqueSwitch == 1);
    st->target_location     = countToRad(m.targetLocation);
    st->target_speed        = speedToRadPerSec((int16_t)m.runningSpeed);
    st->target_acceleration = (float)m.acceleration * SERVO_ACCEL_STEPS_PER_UNIT
                              / SERVO_COUNTS_PER_RAD;
    st->target_torque       = (float)m.torqueLimit;

    st->current_location    = countToRad(m.currentLocation);
    st->current_speed       = speedToRadPerSec(m.currentSpeed);

    // Present load is sign-magnitude, magnitude 0..1000 == 0..100%.
    int16_t load = (m.currentLoad & 0x8000) ? -(int16_t)(m.currentLoad & 0x7FFF)
                                            : (int16_t)m.currentLoad;
    st->current_load        = (float)load / 10.0f;

    st->current_temperature = m.currentTemperature;
    st->servo_status        = m.servoStatus;
    st->is_moving           = (m.mobileSign != 0);
    st->current_voltage     = (float)m.currentVoltage / 10.0f;
    st->current_current     = (float)m.currentCurrent * SERVO_CURRENT_MA_PER_UNIT;
}

void publishHeadState(void)
{
    if (!micro_ros_initialized) {
        return;
    }

    int64_t now_ns = rmw_uros_epoch_nanos();
    head_state_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    head_state_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        fillServoState(i, &head_state_msg.servos[i]);
    }

    head_state_msg.eye_state[0] = b.eye_state[EYE_LEFT];
    head_state_msg.eye_state[1] = b.eye_state[EYE_RIGHT];

    rcl_publish(&state_publisher, &head_state_msg, NULL);
}

// =============================================================================
// Watchdog: no command within timeout -> limp servos + eyes off
// =============================================================================
void handleWatchdog(void)
{
    if (agent_state == AGENT_CONNECTED &&
        b.last_cmd_time > 0 &&
        (millis() - b.last_cmd_time) > WATCHDOG_TIMEOUT_MS) {

        disableAllServoTorques();          // Core 0 applies via updateServoIdle()
        b.eye_pwm[EYE_LEFT]  = 0;          // Core 0 applies via applyEyes()
        b.eye_pwm[EYE_RIGHT] = 0;
        b.last_cmd_time = 0;               // one-shot until next command
    }
}

// =============================================================================
// Entity lifecycle
// =============================================================================
void initializeRosInterface(void)
{
    set_microros_serial_transports(Serial);
    delay(2000);

    agent_state           = WAITING_AGENT;
    ros_entities_created  = false;
    micro_ros_initialized = false;
}

bool createRosEntities(void)
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(&node, "head_controller", NAMESPACE, &support) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_subscription_init_best_effort(
            &head_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, HeadCmd),
            "headcmd") != RCL_RET_OK) {
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_publisher_init_best_effort(
            &state_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, HeadState),
            "headstate") != RCL_RET_OK) {
        rcl_subscription_fini(&head_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&head_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor, &head_subscriber, &head_cmd_msg,
            &headCmdCallback, ON_NEW_DATA) != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&head_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    micro_ros_initialized = true;
    ros_entities_created  = true;
    return true;
}

void destroyRosEntities(void)
{
    if (!ros_entities_created) {
        return;
    }
    rclc_executor_fini(&executor);
    rcl_publisher_fini(&state_publisher, &node);
    rcl_subscription_fini(&head_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    micro_ros_initialized = false;
    ros_entities_created  = false;
}

// =============================================================================
// Connection state machine (Core 1)
// =============================================================================
// Time we entered WAITING_AGENT (0 == boot). Used to detect a long idle wait,
// which leaves stale USB-CDC/XRCE state on both ends and degrades the session.
static uint32_t waiting_since_ms = 0;
// Consecutive failed health pings while CONNECTED (see AGENT_HEALTH_MAX_MISSES).
static uint8_t  health_misses    = 0;

void rosStateMachineTask(void)
{
    switch (agent_state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                                  ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            if (agent_state == AGENT_AVAILABLE &&
                (millis() - waiting_since_ms) > AGENT_LONG_WAIT_REBOOT_MS) {
                // Agent appeared after a long idle wait: reconnect from a fresh
                // boot instead (recreates the known-good power-cycle condition;
                // see AGENT_LONG_WAIT_REBOOT_MS in config.h).
                rp2040.reboot();
            }
            delay(50);
            break;

        case AGENT_AVAILABLE:
            agent_state = createRosEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (agent_state == WAITING_AGENT) {
                destroyRosEntities();
                waiting_since_ms = millis();
                delay(50);
            } else {
                health_misses = 0;
            }
            break;

        case AGENT_CONNECTED:
            // Single short health ping; it still blocks this loop for up to
            // AGENT_HEALTH_TIMEOUT_MS, so misses are tolerated up to
            // AGENT_HEALTH_MAX_MISSES instead of retrying inline (retrying
            // inline stalled the HeadState publisher for up to 1s per check).
            EXECUTE_EVERY_N_MS(AGENT_HEALTH_CHECK_MS,
                if (RMW_RET_OK == rmw_uros_ping_agent(AGENT_HEALTH_TIMEOUT_MS, AGENT_HEALTH_ATTEMPTS)) {
                    health_misses = 0;
                } else if (++health_misses >= AGENT_HEALTH_MAX_MISSES) {
                    agent_state = AGENT_DISCONNECTED;
                }
            );

            if (agent_state == AGENT_CONNECTED && micro_ros_initialized) {
                // Spin every tick (100 Hz) for low command latency, but throttle
                // the outbound HeadState to a stable 50 Hz to match the master_status
                // watchdog and avoid saturating the best-effort USB-CDC link.
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                EXECUTE_AT_RATE_MS(STATE_PUBLISH_PERIOD_MS, publishHeadState());
            }
            handleWatchdog();
            break;

        case AGENT_DISCONNECTED:
            // Safe-limp before tearing the session down: handleWatchdog only
            // fires while CONNECTED, so without this a disconnect could leave
            // the servos holding their last targets with no commander.
            disableAllServoTorques();          // Core 0 applies via updateServoIdle()
            b.eye_pwm[EYE_LEFT]  = 0;          // Core 0 applies via applyEyes()
            b.eye_pwm[EYE_RIGHT] = 0;
            b.last_cmd_time = 0;

            destroyRosEntities();
            delay(100);
            agent_state = WAITING_AGENT;
            waiting_since_ms = millis();
            break;

        default:
            agent_state = WAITING_AGENT;
            waiting_since_ms = millis();
            break;
    }
}
