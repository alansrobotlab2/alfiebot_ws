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

// Services
rcl_service_t calibration_service;

// Message instances
alfie_msgs__msg__BackCmd back_cmd_msg;
alfie_msgs__msg__BackState back_state_msg;

// Service instances
alfie_msgs__srv__BackRequestCalibration_Request calibration_request;
alfie_msgs__srv__BackRequestCalibration_Response calibration_response;

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
    
    // Create calibration service
    ret = rclc_service_init_default(
        &calibration_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(alfie_msgs, srv, BackRequestCalibration),
        "/calibrate_back"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create executor (2 handles: 1 subscription + 1 service)
    ret = rclc_executor_init(&executor, &support.context, 2, &allocator);
    if (ret != RCL_RET_OK) {
        (void)rcl_service_fini(&calibration_service, &node);
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
        (void)rcl_service_fini(&calibration_service, &node);
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Add service to executor
    ret = rclc_executor_add_service(
        &executor,
        &calibration_service,
        &calibration_request,
        &calibration_response,
        &calibrationServiceCallback
    );
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        (void)rcl_service_fini(&calibration_service, &node);
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
        rcl_service_fini(&calibration_service, &node);
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
 * Commands are ignored when calibration is in progress.
 * 
 * @param msgin Pointer to incoming BackCmd message
 */
void backDriveCallback(const void *msgin) {
    const alfie_msgs__msg__BackCmd *msg = (const alfie_msgs__msg__BackCmd *)msgin;
    
    // Ignore commands during calibration
    if (rp.calibration_in_progress) {
        return;
    }
    
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
 * @brief Callback function for calibration service
 * Receives alfie_msgs/BackRequestCalibration requests and initiates calibration
 * 
 * This service performs synchronous calibration and blocks until complete.
 * The calibration process:
 * 1. Checks if already calibrating (rejects if so)
 * 2. Disables BackCmd processing
 * 3. Checks limit switch state
 * 4. If not triggered, moves downward at fixed PWM (74) until limit switch triggers
 * 5. Executes immediate stop when limit switch is triggered
 * 6. Resets encoder counts and position to zero
 * 7. Sets is_calibrated flag
 * 8. Re-enables BackCmd processing
 * 
 * @param req_msg Pointer to incoming service request (empty)
 * @param res_msg Pointer to outgoing service response (bool success)
 * 
 * Response:
 * - success=true: Calibration completed successfully (limit switch triggered)
 * - success=false: Calibration failed (already calibrating or timeout)
 */
void calibrationServiceCallback(const void *req_msg, void *res_msg) {
    // Cast messages to proper types
    const alfie_msgs__srv__BackRequestCalibration_Request *req = 
        (const alfie_msgs__srv__BackRequestCalibration_Request *)req_msg;
    alfie_msgs__srv__BackRequestCalibration_Response *res = 
        (alfie_msgs__srv__BackRequestCalibration_Response *)res_msg;

    // Check if already calibrating
    if (rp.calibration_in_progress) {
        res->success = false;
        return;
    }
    
    // Set calibration flag to disable BackCmd processing and motor control
    rp.calibration_in_progress = true;
    
    // Small delay to ensure any in-flight commands are settled and motor control stops
    delay(50);
    
    // Calibration parameters
    const uint8_t CALIBRATION_PWM = 100;        // Fixed PWM for slow downward motion
    const uint32_t CALIBRATION_TIMEOUT_MS = 20000;  // 20 second timeout
    const uint32_t LED_UPDATE_INTERVAL_MS = 100;    // Update LED every 100ms during calibration
    
    // Set LED to purple during calibration for visual feedback
    rp.statusLED.setColor(128, 0, 128); // Purple (R, G, B)
    
    // Check if limit switch is already triggered
    // Pin reads HIGH when switch is triggered (matches backstate.limit_switch_triggered logic)
    bool limit_switch_triggered = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);
    
    if (!limit_switch_triggered) {
        // Limit switch not triggered - need to move downward
        // Set direction for downward motion (DIR1=LOW, DIR2=HIGH)
        digitalWrite(MOTOR_DIR1_PIN, LOW);
        digitalWrite(MOTOR_DIR2_PIN, HIGH);
        analogWrite(MOTOR_PWM_PIN, CALIBRATION_PWM);
        
        // Update actuator state to show PWM is active during calibration
        rp.actuator_state.pwm_output = CALIBRATION_PWM;
        rp.new_actuator_state = true;
        
        // Poll limit switch until triggered or timeout
        uint32_t start_time = millis();
        uint32_t last_led_time = start_time;
        
        while (!limit_switch_triggered && (millis() - start_time) < CALIBRATION_TIMEOUT_MS) {
            // Read pin directly: HIGH = triggered (same as backstate logic)
            limit_switch_triggered = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);
            
            // Pulse LED during motion for visual feedback
            if (millis() - last_led_time >= LED_UPDATE_INTERVAL_MS) {
                last_led_time = millis();
                // Alternate between purple and dimmer purple
                if ((millis() / LED_UPDATE_INTERVAL_MS) % 2 == 0) {
                    rp.statusLED.setColor(128, 0, 128); // Brighter purple
                } else {
                    rp.statusLED.setColor(64, 0, 64);   // Dimmer purple
                }
            }
            
            delay(1); // Small delay to prevent tight loop
        }
        
        // Immediate stop
        analogWrite(MOTOR_PWM_PIN, 0);
        digitalWrite(MOTOR_DIR1_PIN, LOW);
        digitalWrite(MOTOR_DIR2_PIN, LOW);
        
        // Check if we timed out
        if (!limit_switch_triggered) {
            // Timeout - calibration failed
            rp.calibration_in_progress = false;
            res->success = false;
            return;
        }
        
        // Small delay to ensure motor has fully stopped
        delay(100);
    }
    
    // Limit switch is now triggered - reset position to zero
    rp.resetEncoders();
    
    // Reset command position to 0 to prevent unwanted motion after calibration
    rp.actuator_cmd.position = 0.0;
    rp.actuator_cmd.velocity = 0.0;
    rp.actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
    rp.actuator_cmd.timestamp = millis();
    
    // Clear PID controller state to prevent integral windup or derivative jumps
    rp.motor.velocity_error_integral = 0.0;
    rp.motor.velocity_error_previous = 0.0;
    rp.motor.ramped_velocity = 0.0;
    rp.motor.target_velocity = 0.0;
    rp.motor.target_position = 0.0;
    rp.motor.pwm_output = 0;
    
    // Set calibrated flag
    rp.actuator_state.is_calibrated = true;
    
    // Small delay before re-enabling motor control to ensure everything is settled
    delay(50);
    
    // Re-enable BackCmd processing
    rp.calibration_in_progress = false;
    
    // Calibration successful
    res->success = true;
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
        back_state_msg.pwm_output = rp.actuator_state.pwm_output;
        back_state_msg.is_calibrated = rp.actuator_state.is_calibrated;
        
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