/**
 * @file ros_interface.cpp
 * @brief ROS2 micro-ROS interface implementation
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#include <ros_interface.h>
#include <../../include/driverboard.h>
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
rcl_subscription_t back_subscriber;
rcl_publisher_t state_publisher;

// Message instances
alfie_msgs__msg__BackCmd back_cmd_msg;
alfie_msgs__msg__BackState back_state_msg;

// ROS state
bool micro_ros_initialized = false;
uint32_t last_command_time = 0;

// ROS State Machine Variables
RosAgentState_t agent_state = WAITING_AGENT;
uint32_t last_state_time = 0;
bool ros_entities_created = false;

// External reference to the driver board instance
extern DriverBoard rp;

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
    ret = rclc_node_init_default(&node, "back_drive_controller", "", &support);
    if (ret != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    
    // Create subscriber for /backcmd topic
    ret = rclc_subscription_init_default(
        &back_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, BackCmd),
        "/backcmd"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create publisher for back state
    ret = rclc_publisher_init_default(
        &state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, BackState),
        "/backstate"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create executor
    ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
    if (ret != RCL_RET_OK) {
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Add subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &back_subscriber,
        &back_cmd_msg,
        &backDriveCallback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
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
        rcl_publisher_fini(&state_publisher, &node);
        rcl_subscription_fini(&back_subscriber, &node);
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
 * @brief Callback function for /backcmd topic subscriber
 * Receives alfie_msgs/BackCmd messages and updates actuator command
 * 
 * @param msgin Pointer to incoming BackCmd message
 */
void backDriveCallback(const void *msgin) {
    const alfie_msgs__msg__BackCmd *msg = (const alfie_msgs__msg__BackCmd *)msgin;
    
    // Store actuator command with atomic access
    rp.actuator_cmd.position = msg->position;
    rp.actuator_cmd.velocity = msg->velocity;
    rp.actuator_cmd.acceleration = msg->acceleration;
    rp.actuator_cmd.timestamp = getSystemTimeMs();
    
    // Set flag to indicate new command received
    rp.new_actuator_command = true;
    last_command_time = rp.actuator_cmd.timestamp;
}

/**
 * @brief Process actuator commands
 * Handles new actuator commands received from ROS
 */
void processVelocityCommand(void) {
    // Check if new actuator command flag is set
    if (rp.new_actuator_command) {
        rp.new_actuator_command = false; // Clear the flag
        // Actuator command is already updated by callback
        // Motor control module will read rp.actuator_cmd directly
    }
}

/**
 * @brief Publish actuator state data to ROS2
 * Sends current actuator state to /backstate topic
 */
void publishOdometry(void) {
    if (micro_ros_initialized && rp.new_actuator_state) {
        // Populate back_state_msg with current actuator state data
        back_state_msg.board_temp = rp.actuator_state.board_temp;
        back_state_msg.limit_switch_triggered = rp.actuator_state.limit_switch_triggered;
        back_state_msg.command_position = rp.actuator_state.command_position;
        back_state_msg.command_velocity = rp.actuator_state.command_velocity;
        back_state_msg.command_acceleration = rp.actuator_state.command_acceleration;
        back_state_msg.is_moving = rp.actuator_state.is_moving;
        back_state_msg.current_position = rp.actuator_state.current_position;
        back_state_msg.current_velocity = rp.actuator_state.current_velocity;
        back_state_msg.current_acceleration = rp.actuator_state.current_acceleration;
        back_state_msg.pulse_count = rp.actuator_state.pulses;
        
        // Publish the message
        rcl_publish(&state_publisher, &back_state_msg, NULL);
        
        rp.new_actuator_state = false;
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
        // Command timeout - stop the actuator (field-by-field assignment for volatile)
        rp.actuator_cmd.position = rp.motor.current_position; // Hold current position
        rp.actuator_cmd.velocity = 0.0f;
        rp.actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
        rp.actuator_cmd.timestamp = current_time;
        rp.new_actuator_command = true;
        last_command_time = 0; // Reset to prevent repeated stops
        
        // Emergency stop called in motor control module
        rp.emergencyStop();
    }
}

/**
 * @brief Get system time in milliseconds
 * @return Current system time in milliseconds
 */
uint32_t getSystemTimeMs(void) {
    return millis();
}