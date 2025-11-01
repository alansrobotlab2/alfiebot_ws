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
    ret = rclc_node_init_default(&node, "mecanum_drive_controller", "", &support);
    if (ret != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    
    // Create subscriber for /mecanumdrive topic
    ret = rclc_subscription_init_default(
        &mecanum_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/mecanumdrive"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create publisher for odometry
    ret = rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
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
void rosStateMachineTask(void) {
    switch (agent_state) {
        case WAITING_AGENT:
            // Every 100ms, check if the micro-ROS agent is available and update agentState accordingly
            EXECUTE_EVERY_N_MS(AGENT_PING_INTERVAL_MS, 
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, AGENT_PING_ATTEMPTS)) ? AGENT_AVAILABLE : WAITING_AGENT;
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
            // Check connection health every 200ms
            EXECUTE_EVERY_N_MS(AGENT_HEALTH_CHECK_MS, 
                agent_state = (RMW_RET_OK == rmw_uros_ping_agent(AGENT_HEALTH_TIMEOUT_MS, AGENT_HEALTH_ATTEMPTS)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            
            // Publish odometry data
            if (micro_ros_initialized) {
                publishOdometry();
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
            destroyRosEntities();
            delay(100);
            agent_state = WAITING_AGENT;
            break;
            
        default:
            agent_state = WAITING_AGENT;
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
    // TODO: Implement odometry publishing
    // This will be implemented when odometry calculations are ready
    if (micro_ros_initialized && new_odometry_data) {
        // Populate odom_msg with current odometry data
        // odom_msg.pose.pose.position.x = rp.odometry.position_x;
        // odom_msg.pose.pose.position.y = rp.odometry.position_y;
        // ... etc
        
        // Publish the message
        // rcl_publish(&odom_publisher, &odom_msg, NULL);
        
        new_odometry_data = false;
    }
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
        
        // TODO: Add motor disabling functions when motor_control module is complete
        // disableAllMotors();
    }
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
uint32_t getSystemTimeMs(void) {
    return millis();
}