#include "ros_interface.h"
#include "driverboard.h"
#include "servo_control.h"
#include "arm_control.h"
#include <rmw_microros/rmw_microros.h>

extern DriverBoard b;

// =============================================================================
// micro-ROS entities
// =============================================================================
static rcl_allocator_t  allocator;
static rclc_support_t   support;
static rcl_node_t       node;
static rclc_executor_t  executor;

static rcl_subscription_t arm_subscriber;
static rcl_publisher_t    state_publisher;

static alfie_msgs__msg__ArmCmd   arm_cmd_msg;
static alfie_msgs__msg__ArmState arm_state_msg;

// =============================================================================
// State
// =============================================================================
RosAgentState_t agent_state          = WAITING_AGENT;
bool            ros_entities_created  = false;
bool            micro_ros_initialized = false;

// =============================================================================
// Command decode / state encode (6<->7 mapping lives in lib/arm_control)
// =============================================================================
void armCmdCallback(const void *msgin)
{
    const alfie_msgs__msg__ArmCmd *msg = (const alfie_msgs__msg__ArmCmd *)msgin;

    applyArmCmd(msg);                 // 6 joints -> 7 servos in b.mBuf[]

    b.new_arm_command = true;
    b.last_cmd_time   = millis();
}

void publishArmState(void)
{
    if (!micro_ros_initialized) {
        return;
    }

    int64_t now_ns = rmw_uros_epoch_nanos();
    arm_state_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
    arm_state_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    fillArmState(&arm_state_msg);     // 7 servos -> 6 joints + board id

    rcl_publish(&state_publisher, &arm_state_msg, NULL);
}

// =============================================================================
// Watchdog: no command within timeout -> limp servos
// =============================================================================
void handleWatchdog(void)
{
    if (agent_state == AGENT_CONNECTED &&
        b.last_cmd_time > 0 &&
        (millis() - b.last_cmd_time) > WATCHDOG_TIMEOUT_MS) {

        disableAllServoTorques();          // Core 0 applies via updateServoIdle()
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

    if (rclc_node_init_default(&node, ARM_NODE_NAME, b.getNamespace(), &support) != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_subscription_init_best_effort(
            &arm_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, ArmCmd),
            "armcmd") != RCL_RET_OK) {
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_publisher_init_best_effort(
            &state_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, ArmState),
            "armstate") != RCL_RET_OK) {
        rcl_subscription_fini(&arm_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&arm_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor, &arm_subscriber, &arm_cmd_msg,
            &armCmdCallback, ON_NEW_DATA) != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&arm_subscriber, &node);
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
    rcl_subscription_fini(&arm_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    micro_ros_initialized = false;
    ros_entities_created  = false;
}

// =============================================================================
// Connection state machine (Core 1)
// =============================================================================
void rosStateMachineTask(void)
{
    switch (agent_state) {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS))
                                  ? AGENT_AVAILABLE : WAITING_AGENT;
            );
            delay(50);
            break;

        case AGENT_AVAILABLE:
            agent_state = createRosEntities() ? AGENT_CONNECTED : WAITING_AGENT;
            if (agent_state == WAITING_AGENT) {
                destroyRosEntities();
                delay(50);
            }
            break;

        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(AGENT_HEALTH_CHECK_MS,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_HEALTH_TIMEOUT_MS, AGENT_HEALTH_ATTEMPTS))
                                  ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );

            if (micro_ros_initialized) {
                // Spin every tick (100 Hz) for low command latency, but throttle
                // the outbound ArmState to a stable 50 Hz to match the master_status
                // watchdog and avoid saturating the best-effort USB-CDC link.
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                EXECUTE_AT_RATE_MS(STATE_PUBLISH_PERIOD_MS, publishArmState());
            }
            handleWatchdog();
            break;

        case AGENT_DISCONNECTED:
            destroyRosEntities();
            delay(100);
            agent_state = WAITING_AGENT;
            break;

        default:
            agent_state = WAITING_AGENT;
            break;
    }
}
