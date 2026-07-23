/**
 * @file ros_interface.cpp
 * @brief ROS2 micro-ROS interface implementation
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#include "ros_interface.h"
#include "driverboard.h"
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include <string.h>

// =============================================================================
// GLOBAL ROS INTERFACE VARIABLES
// =============================================================================

// Micro-ROS entities
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Subscribers and Publishers
rcl_subscription_t mecanum_subscriber;
rcl_publisher_t odom_publisher;

// Message instances
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;

// ROS state
bool micro_ros_initialized = false;
uint32_t last_command_time = 0;

// ROS State Machine Variables
RosAgentState_t agent_state = WAITING_AGENT;
uint32_t last_state_time = 0;
bool ros_entities_created = false;

// External reference to the driver board instance
extern DriverBoard rp;

// Inter-core communication flags
volatile bool new_velocity_command = false;
volatile bool new_odometry_data = false;

// =============================================================================
// ROS INTERFACE IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize ROS2 communication systems
 * Sets up micro-ROS transport and initializes state machine
 */
void initializeRosInterface(void) {
    // Set micro-ROS transport (USB Serial)
    set_microros_serial_transports(Serial);
    delay(2000); // Wait for transport to stabilize
    
    // Initialize state machine
    agent_state = WAITING_AGENT;
    last_state_time = millis();
    ros_entities_created = false;
    micro_ros_initialized = false;
}

/**
 * @brief Create all ROS entities (node, subscribers, publishers, executor)
 * @return true if successful, false if failed
 */
bool createRosEntities(void) {
    rcl_ret_t ret;
    
    // Initialize allocator
    allocator = rcl_get_default_allocator();
    
    // Create init_options and support
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        return false;
    }
    
    // Create node
    ret = rclc_node_init_default(&node, "mecanum_drive_controller", NAMESPACE, &support);
    if (ret != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    
    // Create subscriber for the mecanumdrive topic (relative: resolves under the
    // node NAMESPACE to /alfie/low/mecanumdrive).
    // BEST_EFFORT to match the command_mux publisher (and the other pico drivers)
    ret = rclc_subscription_init_best_effort(
        &mecanum_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "mecanumdrive"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create publisher for odometry (relative: resolves to /alfie/low/odom).
    // RELIABLE, deliberately: serialized nav_msgs/Odometry (~720 B, two 36-double
    // covariance blocks) exceeds the 512 B XRCE serial MTU, and best-effort
    // streams cannot fragment — a best-effort odom publisher fails EVERY publish
    // silently. Reliable streams fragment, and a RELIABLE publisher remains
    // QoS-compatible with the host's BEST_EFFORT subscribers.
    ret = rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_subscription_fini(&mecanum_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create executor
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        (void)rcl_publisher_fini(&odom_publisher, &node);
        (void)rcl_subscription_fini(&mecanum_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Add subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &mecanum_subscriber,
        &twist_msg,
        &mecanumDriveCallback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        (void)rcl_publisher_fini(&odom_publisher, &node);
        (void)rcl_subscription_fini(&mecanum_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    micro_ros_initialized = true;
    ros_entities_created = true;
    return true;
}

/**
 * @brief Destroy all ROS entities and cleanup resources
 */
void destroyRosEntities(void) {
    if (ros_entities_created) {
        // Cleanup in reverse order of creation
        rclc_executor_fini(&executor);
        rcl_publisher_fini(&odom_publisher, &node);
        rcl_subscription_fini(&mecanum_subscriber, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);
        
        micro_ros_initialized = false;
        ros_entities_created = false;
    }
}

/**
 * @brief ROS state machine task function
 * Implements the main ROS connectivity state machine
 */
// Time we entered WAITING_AGENT (0 == boot). Used to detect a long idle wait,
// which leaves stale USB-CDC/XRCE state on both ends and degrades the session.
static uint32_t waiting_since_ms = 0;
// Consecutive failed health pings while CONNECTED (see AGENT_HEALTH_MAX_MISSES).
static uint8_t  health_misses    = 0;

void rosStateMachineTask(void) {
    switch (agent_state) {
        case WAITING_AGENT:
            // Every 100ms, check if the micro-ROS agent is available and update agentState accordingly
            EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS,
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;
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
            // inline stalled the odometry publisher for up to 1s per check).
            EXECUTE_EVERY_N_MS(AGENT_HEALTH_CHECK_MS,
                if (RMW_RET_OK == rmw_uros_ping_agent(AGENT_HEALTH_TIMEOUT_MS, AGENT_HEALTH_ATTEMPTS)) {
                    health_misses = 0;
                } else if (++health_misses >= AGENT_HEALTH_MAX_MISSES) {
                    agent_state = AGENT_DISCONNECTED;
                }
            );

            if (agent_state != AGENT_CONNECTED) {
                break;
            }

            // Publish odometry data, throttled to a stable 50 Hz to match the
            // master_status watchdog and avoid saturating the best-effort USB-CDC
            // link (the ROS task still ticks at 100 Hz for low command latency).
            if (micro_ros_initialized) {
                EXECUTE_AT_RATE_MS(STATE_PUBLISH_PERIOD_MS, publishOdometry());
            }

            // Process ROS callbacks (commands) - give it 1ms to process queued messages
            if (micro_ros_initialized) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            }

            // Process velocity commands and handle watchdog
            processVelocityCommand();
            handleWatchdog();
            break;

        case AGENT_DISCONNECTED:
            // Safe-stop the base before tearing the session down: handleWatchdog
            // only stops on command timeout while CONNECTED, so without this a
            // disconnect mid-motion could leave the wheels running the last
            // velocity. (Core 0 pushes the zero speeds on its next cycle.)
            rp.velocity_cmd.linear_x = 0.0f;
            rp.velocity_cmd.linear_y = 0.0f;
            rp.velocity_cmd.angular_z = 0.0f;
            rp.velocity_cmd.timestamp = getSystemTimeMs();
            new_velocity_command = true;

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

/**
 * @brief Main ROS communication loop (deprecated - use rosStateMachineTask instead)
 * Should be called regularly from Core 1 loop
 */
void updateRosInterface(void) {
    // Redirect to state machine implementation
    rosStateMachineTask();
}

/**
 * @brief Callback function for /mecanumdrive topic subscriber
 * Receives geometry_msgs/Twist messages and updates velocity command
 * 
 * @param msgin Pointer to incoming Twist message
 */
void mecanumDriveCallback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    
        // Store velocity command with atomic access
    rp.velocity_cmd.linear_x = msg->linear.x;
    rp.velocity_cmd.linear_y = msg->linear.y;
    rp.velocity_cmd.angular_z = msg->angular.z;
    rp.velocity_cmd.timestamp = getSystemTimeMs();
    
    // Set flag to indicate new command received
    new_velocity_command = true;
    last_command_time = rp.velocity_cmd.timestamp;
}

/**
 * @brief Process velocity commands
 * Handles new velocity commands received from ROS
 */
void processVelocityCommand(void) {
    // Check if new velocity command flag is set
    if (new_velocity_command) {
        new_velocity_command = false; // Clear the flag
        // Velocity command is already updated by callback
        // Motor control module will read rp.velocity_cmd directly
    }
}

/**
 * @brief Publish odometry data to ROS2
 * Sends current robot odometry to /odom topic
 */
void publishOdometry(void) {
    if (!micro_ros_initialized) {
        return;
    }
    // Publish unconditionally at the STATE_PUBLISH cadence. Odometry doubles as
    // the drive board's heartbeat: gating on new_odometry_data made the topic go
    // fully silent whenever Core 0's encoder/I2C path stalled, which is
    // indistinguishable from a dead board. Stale values still flow (consumers
    // can see them frozen); the flag is cleared for Core 0 bookkeeping only.
    new_odometry_data = false;

    // Frame ids (point the rosidl strings at static buffers)
    static char odom_frame_id[] = "odom";
    static char base_frame_id[] = "base_link";
    odom_msg.header.frame_id.data = odom_frame_id;
    odom_msg.header.frame_id.size = strlen(odom_frame_id);
    odom_msg.header.frame_id.capacity = sizeof(odom_frame_id);
    odom_msg.child_frame_id.data = base_frame_id;
    odom_msg.child_frame_id.size = strlen(base_frame_id);
    odom_msg.child_frame_id.capacity = sizeof(base_frame_id);

    // Timestamp from local clock
    uint32_t now_ms = getSystemTimeMs();
    odom_msg.header.stamp.sec = (int32_t)(now_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000UL);

    // Pose
    odom_msg.pose.pose.position.x = rp.odometry.position_x;
    odom_msg.pose.pose.position.y = rp.odometry.position_y;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation (yaw -> quaternion about Z)
    float yaw = rp.odometry.orientation;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(yaw * 0.5f);
    odom_msg.pose.pose.orientation.w = cos(yaw * 0.5f);

    // Body-frame twist
    odom_msg.twist.twist.linear.x = rp.odometry.linear_velocity_x;
    odom_msg.twist.twist.linear.y = rp.odometry.linear_velocity_y;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = rp.odometry.angular_velocity;

    (void)rcl_publish(&odom_publisher, &odom_msg, NULL);
}

/**
 * @brief Monitor communication watchdog
 * Checks for command timeouts and safety conditions
 */
void handleWatchdog(void) {
    uint32_t current_time = getSystemTimeMs();
    
    // Check for command timeout only when agent is connected
    if (agent_state == AGENT_CONNECTED && last_command_time > 0 && 
        (current_time - last_command_time) > WATCHDOG_TIMEOUT_MS) {
        // Command timeout - stop the robot (field-by-field assignment for volatile)
        rp.velocity_cmd.linear_x = 0.0f;
        rp.velocity_cmd.linear_y = 0.0f;
        rp.velocity_cmd.angular_z = 0.0f;
        rp.velocity_cmd.timestamp = current_time;
        new_velocity_command = true;
        last_command_time = 0; // Reset to prevent repeated stops

        // Note: the Hiwonder I2C bus is owned by Core 0. We only zero the shared
        // velocity command here; Core 0's updateMotorControl() will push zero
        // speeds to the controller on its next cycle (within CONTROL_LOOP_PERIOD_MS).
    }
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
uint32_t getSystemTimeMs(void) {
    return millis();
}