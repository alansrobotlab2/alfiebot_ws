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
static rcl_service_t      servo_service;

static alfie_msgs__srv__ServoService_Request  servo_srv_req;
static alfie_msgs__srv__ServoService_Response servo_srv_res;


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
// Register-map service (Core 1 half)
// =============================================================================
// Read-only: the request's `operation` must be 'r'. Writes are rejected here
// rather than silently ignored, so a caller aiming at EEPROM gets a failure it
// can see instead of believing a value was stored.
//
// The response's readmemoryresult is 1 on a good read, 0 otherwise (bad servo
// id, unsupported operation, or the bus read came back short).

static void fillMemoryMap(const MemoryStruct *m, alfie_msgs__msg__ServoMemoryMap *out)
{
    out->firmwaremajor        = m->firmwareMajor;
    out->firmwaresub          = m->firmwareSub;
    out->servomajor           = m->servoMajor;
    out->servosub             = m->servoSub;
    out->servoid              = m->servoID;
    out->baudrate             = m->baudRate;
    out->returndelay          = m->returnDelay;
    out->responsestatuslevel  = m->responseStatusLevel;
    out->minanglelimit        = (uint16_t)m->minAngleLimit;
    out->maxanglelimit        = (uint16_t)m->maxAngleLimit;
    out->maxtemplimit         = m->maxTempLimit;
    out->maxinputvoltage      = m->maxInputVoltage;   // 0x0E
    out->mininputvoltage      = m->minInputVoltage;   // 0x0F
    out->maxtorque            = m->maxTorque;
    out->phase                = m->phase;
    out->unloadingcondition   = m->unloadingCondition;
    out->ledalarmcondition    = m->LEDAlarmCondition;
    out->pcoefficient         = m->Pcoefficient;
    out->dcoefficient         = m->Dcoefficient;
    out->icoefficient         = m->Icoefficient;
    out->minstartupforce      = m->minStartupForce;
    out->clockwiseinsensitivearea            = m->clockwiseInsensitiveArea;
    out->counterclockwiseinsensitiveregion   = m->counterclockwiseInsensitiveRegion;
    out->protectioncurrent    = m->protectionCurrent;
    out->angularresolution    = m->angularResolution;
    out->positioncorrection   = m->positionCorrection;
    out->operationmode        = m->operationMode;
    out->protectivetorque     = m->protectiveTorque;
    out->protectiontime       = m->protectionTime;
    out->overloadtorque       = m->overloadTorque;
    out->speedclosedlooppcoefficient  = m->speedClosedLoopPcoefficient;
    out->overcurrentprotectiontime    = m->OvercurrentProtectionTime;
    out->velocityclosedloopicoefficient = m->velocityClosedLoopIcoefficient;
    out->torqueswitch         = m->torqueSwitch;
    out->acceleration         = m->acceleration;
    out->targetlocation       = m->targetLocation;
    out->runningtime          = m->runningTime;
    out->runningspeed         = m->runningSpeed;
    out->torquelimit          = m->torqueLimit;
    out->lockmark             = m->lockMark;
    out->currentlocation      = m->currentLocation;
    out->currentspeed         = m->currentSpeed;
    out->currentload          = m->currentLoad;
    out->currentvoltage       = m->currentVoltage;
    out->currenttemperature   = m->currentTemperature;
    out->asyncwriteflag       = m->asyncWriteFlag;
    out->servostatus          = m->servoStatus;
    out->mobilesign           = m->mobileSign;
    out->currentcurrent       = m->currentCurrent;
}

void servoServiceCallback(const void *reqin, void *resin)
{
    const alfie_msgs__srv__ServoService_Request *req =
        (const alfie_msgs__srv__ServoService_Request *)reqin;
    alfie_msgs__srv__ServoService_Response *res =
        (alfie_msgs__srv__ServoService_Response *)resin;

    memset(&res->memorymap, 0, sizeof(res->memorymap));

    const uint8_t op = req->operation;
    if (op != SERVO_SERVICE_OP_READ && op != SERVO_SERVICE_OP_WRITE &&
        op != SERVO_SERVICE_OP_UNLOCK && op != SERVO_SERVICE_OP_LOCK) {
        return;                                  // readmemoryresult stays 0
    }
    if (req->servo < 1 || req->servo > NUM_SERVOS) {
        return;
    }

    // Park the request for Core 0 and wait for it to run on the servo tick.
    b.mem_req_id    = req->servo;
    b.mem_req_op    = op;
    b.mem_req_addr  = req->address;
    b.mem_req_value = req->value;
    b.mem_req_ok    = false;
    b.mem_req_write_result = SERVO_WRITE_NONE;
    b.mem_req_width = 0;
    __sync_synchronize();
    b.mem_req_pending = true;

    const uint32_t started = millis();
    while (b.mem_req_pending && (millis() - started) < MEM_REQ_TIMEOUT_MS) {
        delay(1);
    }

    if (b.mem_req_pending) {
        b.mem_req_pending = false;               // abandon it; Core 0 is wedged
        return;
    }

    __sync_synchronize();

    if (b.mem_req_ok) {
        fillMemoryMap(&b.mem_req_buf.memory, &res->memorymap);
        res->memorymap.readmemoryresult = 1;
    }
    // Reported even when the read failed, so a caller can tell "the write was
    // refused" apart from "the servo stopped answering".
    res->memorymap.writebyteresult = b.mem_req_write_result;
    res->memorymap.writewordresult = b.mem_req_width;
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

    if (rclc_service_init_default(
            &servo_service, &node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(alfie_msgs, srv, ServoService),
            "servoservice") != RCL_RET_OK) {
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&arm_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // 2 handles: the command subscription + the register-map service.
    if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
        rcl_service_fini(&servo_service, &node);
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
        rcl_service_fini(&servo_service, &node);
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&arm_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    if (rclc_executor_add_service(
            &executor, &servo_service, &servo_srv_req, &servo_srv_res,
            &servoServiceCallback) != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        rcl_service_fini(&servo_service, &node);
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
    rcl_service_fini(&servo_service, &node);
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
            // inline stalled the ArmState publisher for up to 1s per check).
            EXECUTE_EVERY_N_MS(AGENT_HEALTH_CHECK_MS,
                if (RMW_RET_OK == rmw_uros_ping_agent(AGENT_HEALTH_TIMEOUT_MS, AGENT_HEALTH_ATTEMPTS)) {
                    health_misses = 0;
                } else if (++health_misses >= AGENT_HEALTH_MAX_MISSES) {
                    agent_state = AGENT_DISCONNECTED;
                }
            );

            if (agent_state == AGENT_CONNECTED && micro_ros_initialized) {
                // Spin every tick (100 Hz) for low command latency, but throttle
                // the outbound ArmState to a stable 50 Hz to match the master_status
                // watchdog and avoid saturating the best-effort USB-CDC link.
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
                EXECUTE_AT_RATE_MS(STATE_PUBLISH_PERIOD_MS, publishArmState());
            }
            handleWatchdog();
            break;

        case AGENT_DISCONNECTED:
            // Safe-limp before tearing the session down: handleWatchdog only
            // fires while CONNECTED, so without this a disconnect could leave
            // the servos holding their last targets with no commander.
            disableAllServoTorques();      // Core 0 applies via updateServoIdle()
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
