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
rcl_subscription_t neck_power_subscriber;
rcl_publisher_t state_publisher;

// Services
rcl_service_t calibration_service;

// Message instances
alfie_msgs__msg__BackCmd back_cmd_msg;
alfie_msgs__msg__BackState back_state_msg;
std_msgs__msg__Empty neck_power_msg;

// Compass-decouple timestamps (millis of last "power applied" signal). 0 = never.
uint32_t last_neck_power_ms = 0;  // from the neck_power heartbeat (remote neck servo 0)
uint32_t last_back_power_ms = 0;  // from local actuator PWM being non-zero

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
    ret = rclc_node_init_default(&node, "back_drive_controller", NAMESPACE, &support);
    if (ret != RCL_RET_OK) {
        rclc_support_fini(&support);
        return false;
    }
    
    // Create subscriber for backcmd topic (uses node namespace)
    // Using best effort QoS for low-latency communication
    ret = rclc_subscription_init_best_effort(
        &back_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, BackCmd),
        "backcmd"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // Create subscriber for neck_power topic (uses node namespace)
    // std_msgs/Empty heartbeat from master_low_status; presence within
    // COMPASS_DECOUPLE_MS means neck servo 0 is drawing power.
    ret = rclc_subscription_init_best_effort(
        &neck_power_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "neck_power"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // Create publisher for back state (uses node namespace)
    // Using best effort QoS for low-latency communication
    ret = rclc_publisher_init_best_effort(
        &state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(alfie_msgs, msg, BackState),
        "backstate"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }
    
    // Create calibration service (uses node namespace)
    ret = rclc_service_init_default(
        &calibration_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(alfie_msgs, srv, BackRequestCalibration),
        "calibrate_back"
    );
    if (ret != RCL_RET_OK) {
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // Create executor (3 handles: 2 subscriptions + 1 service)
    ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if (ret != RCL_RET_OK) {
        (void)rcl_service_fini(&calibration_service, &node);
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // Add backcmd subscription to executor
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
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
        (void)rcl_subscription_fini(&back_subscriber, &node);
        (void)rcl_node_fini(&node);
        rclc_support_fini(&support);
        return false;
    }

    // Add neck_power subscription to executor
    ret = rclc_executor_add_subscription(
        &executor,
        &neck_power_subscriber,
        &neck_power_msg,
        &neckPowerCallback,
        ON_NEW_DATA
    );
    if (ret != RCL_RET_OK) {
        rclc_executor_fini(&executor);
        (void)rcl_service_fini(&calibration_service, &node);
        (void)rcl_publisher_fini(&state_publisher, &node);
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
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
        (void)rcl_subscription_fini(&neck_power_subscriber, &node);
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
        rcl_subscription_fini(&neck_power_subscriber, &node);
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
            // inline stalled the BackState publisher for up to 1s per check).
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
            // Safe-hold before tearing the session down: handleWatchdog only
            // fires while CONNECTED, so without this a disconnect mid-move could
            // leave the actuator chasing a stale setpoint with no commander.
            // Mirrors the watchdog action: hold the current position.
            rp.actuator_cmd.position = rp.motor.current_position;
            rp.actuator_cmd.velocity = 0.0f;
            rp.actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
            rp.actuator_cmd.timestamp = getSystemTimeMs();
            rp.new_actuator_command = true;

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
 * @brief Callback function for /backcmd topic subscriber
 * Receives alfie_msgs/BackCmd messages and updates actuator command
 * Commands are ignored when calibration is in progress.
 * 
 * @param msgin Pointer to incoming BackCmd message
 */
void backDriveCallback(const void *msgin) {
    const alfie_msgs__msg__BackCmd *msg = (const alfie_msgs__msg__BackCmd *)msgin;
    
    // Ignore commands until calibrated
    if (!rp.actuator_state.is_calibrated) {
        return;
    }
    
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
 * @brief Callback for the neck_power topic subscriber
 * Records the arrival time of the neck-servo-0 power heartbeat so the compass
 * stays decoupled for COMPASS_DECOUPLE_MS after the last one.
 *
 * @param msgin Pointer to incoming std_msgs/Empty message (unused)
 */
void neckPowerCallback(const void *msgin) {
    (void)msgin;
    last_neck_power_ms = millis();
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
    // Held to VELOCITY_PID_OUTPUT_LIMIT: this path bypasses the PID, so the duty
    // ceiling set in config.h does not otherwise apply here.
    //
    // THIS PATH CARRIES ITS OWN GUARDS - do not assume the usual ones apply.
    //
    // Both stall and runaway detection key off motor.pwm_output, which is stale
    // here because calibration drives the bridge directly while normal motor
    // control is suspended. Runaway is skipped explicitly; stall silently never
    // fires because pwm_mag reads 0. So the descent loop below implements its
    // own encoder-motion stall guard and its own direction guard, sized by
    // CALIBRATION_STALL_MS / CALIBRATION_MIN_COUNTS / CALIBRATION_REVERSE_COUNTS.
    //
    // Before those existed the only bound was CALIBRATION_TIMEOUT_MS: up to 20 s
    // driving into the bottom stop at the full duty ceiling, with the DRV8876's
    // ITRIP no longer providing a current limit either (see config.h, WHY THERE
    // IS NO CURRENT TRIP). Now a jammed or already-down axis aborts in ~400 ms.
    const uint8_t CALIBRATION_PWM = VELOCITY_PID_OUTPUT_LIMIT;
    const uint32_t CALIBRATION_TIMEOUT_MS = 20000;  // 20 second timeout
    const uint32_t LED_UPDATE_INTERVAL_MS = 100;    // Update LED every 100ms during calibration
    
    // Set LED to purple during calibration for visual feedback
    rp.statusLED.setColor(128, 0, 128); // Purple (R, G, B)
    
    // Check if limit switch is already triggered.
    //
    // The switch is NORMALLY CLOSED, so HIGH = at the bottom. See
    // LIMIT_SWITCH_ACTIVE_LEVEL in config.h. This read was `== LOW`, which is
    // inverted: during ordinary travel the pin sits LOW, so the test below
    // ("not triggered? then drive down") failed immediately and calibration
    // SKIPPED THE DESCENT ENTIRELY, zeroing wherever the actuator happened to
    // be. Silent false datum - no error, no timeout, just a wrong origin.
    bool limit_switch_triggered =
        (digitalRead(LIMIT_SWITCH_PIN) == LIMIT_SWITCH_ACTIVE_LEVEL);
    
    if (!limit_switch_triggered) {
        // Limit switch not triggered - need to move downward toward the switch
        digitalWrite(MOTOR_PH_PIN, MOTOR_PH_DOWN);
        analogWrite(MOTOR_EN_PIN, CALIBRATION_PWM);
        
        // Update actuator state to show PWM is active during calibration
        rp.actuator_state.pwm_output = CALIBRATION_PWM;
        rp.new_actuator_state = true;
        
        // Poll limit switch until triggered, stalled, reversed, or timed out.
        uint32_t start_time = millis();
        uint32_t last_led_time = start_time;

        // Encoder-motion guards. This loop is the only powered motion path with
        // no stall or runaway detection behind it, so it carries its own.
        const int32_t descent_start_count = rp.motor.encoder_count;
        int32_t  progress_count = descent_start_count;
        uint32_t progress_ms    = start_time;
        bool     stalled        = false;
        bool     reversed       = false;

        while (!limit_switch_triggered && (millis() - start_time) < CALIBRATION_TIMEOUT_MS) {
            limit_switch_triggered =
                (digitalRead(LIMIT_SWITCH_PIN) == LIMIT_SWITCH_ACTIVE_LEVEL);

            const int32_t count = rp.motor.encoder_count;

            // Commanded DOWN, so the count must fall. Rising past the threshold
            // means the motor or encoder is wired backwards - abort before a
            // gravity-loaded axis is driven into the top stop.
            if (count - descent_start_count > CALIBRATION_REVERSE_COUNTS) {
                reversed = true;
                break;
            }

            // Progress is downward motion; measure magnitude either way so a
            // jittering axis cannot masquerade as movement.
            const int32_t delta = count - progress_count;
            if (delta <= -CALIBRATION_MIN_COUNTS) {
                progress_count = count;
                progress_ms    = millis();
            } else if ((millis() - progress_ms) >= CALIBRATION_STALL_MS) {
                stalled = true;
                break;
            }

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

        // Immediate stop (EN=0 is brake on the DRV8876; nSLEEP stays HIGH)
        analogWrite(MOTOR_EN_PIN, 0);
        rp.actuator_state.pwm_output = 0;
        rp.new_actuator_state = true;

        // Any exit other than "switch asserted" is a failure. Classify it so the
        // caller learns something more useful than "calibration failed": a
        // reversal is a wiring fault and must not be retried, a stall means the
        // axis is already down or obstructed, a timeout means it never arrived.
        if (reversed) {
            rp.motor.runaway_detected = true;      // latches; only a reset clears
            rp.motor.fault_detected   = true;
            rp.calibration_in_progress = false;
            res->success = false;
            return;
        }
        if (stalled || !limit_switch_triggered) {
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
        back_state_msg.error_code = rp.actuator_state.error_code;
        back_state_msg.stall_position = rp.actuator_state.stall_position;
        back_state_msg.stall_count = rp.actuator_state.stall_count;
        back_state_msg.fault_latched = rp.actuator_state.fault_latched;

        // Compass decouple: while the local actuator is driving (PWM != 0) or the
        // neck servo 0 heartbeat is fresh, the magnetometer is corrupted, so
        // publish the compass-free game rotation vector instead of the
        // mag-referenced rotation vector. Re-couple COMPASS_DECOUPLE_MS after the
        // last power signal. The != 0 guards avoid a spurious window at boot.
        uint32_t now = millis();
        if (rp.actuator_state.pwm_output != 0) {
            last_back_power_ms = now;
        }
        bool decouple =
            (last_neck_power_ms != 0 && (uint32_t)(now - last_neck_power_ms) < COMPASS_DECOUPLE_MS) ||
            (last_back_power_ms != 0 && (uint32_t)(now - last_back_power_ms) < COMPASS_DECOUPLE_MS);

        // BNO085 IMU telemetry (latest snapshot from Core 0)
        // Orientation quaternion: rotation vector (mag-referenced) normally, or
        // the game rotation vector (compass-free) while decoupled. The
        // orientation_reliable flag lets absolute-yaw consumers drop the
        // heading during the decouple window instead of following the swap.
        if (decouple) {
            back_state_msg.imu.orientation_x = rp.imu_data.game_qx;
            back_state_msg.imu.orientation_y = rp.imu_data.game_qy;
            back_state_msg.imu.orientation_z = rp.imu_data.game_qz;
            back_state_msg.imu.orientation_w = rp.imu_data.game_qw;
        } else {
            back_state_msg.imu.orientation_x = rp.imu_data.qx;
            back_state_msg.imu.orientation_y = rp.imu_data.qy;
            back_state_msg.imu.orientation_z = rp.imu_data.qz;
            back_state_msg.imu.orientation_w = rp.imu_data.qw;
        }
        back_state_msg.imu.orientation_reliable = !decouple;
        // Angular velocity (rad/s)
        back_state_msg.imu.angular_velocity_x = rp.imu_data.gyro_x;
        back_state_msg.imu.angular_velocity_y = rp.imu_data.gyro_y;
        back_state_msg.imu.angular_velocity_z = rp.imu_data.gyro_z;
        // Linear acceleration (m/s^2, gravity included)
        back_state_msg.imu.linear_acceleration_x = rp.imu_data.accel_x;
        back_state_msg.imu.linear_acceleration_y = rp.imu_data.accel_y;
        back_state_msg.imu.linear_acceleration_z = rp.imu_data.accel_z;
        rp.new_imu_data = false;

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