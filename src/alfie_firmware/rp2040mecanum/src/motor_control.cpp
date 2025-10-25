/**
 * @file motor_control.cpp
 * @brief Motor control and peripheral management implementation
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#include "motor_control.h"
#include "driverboard.h"

// =============================================================================
// DRIVERBOARD CLASS IMPLEMENTATION
// =============================================================================

/**
 * @brief DriverBoard constructor - initializes all state variables
 */
DriverBoard::DriverBoard() {
    // Initialize motor states
    for (int i = 0; i < 4; i++) {
        motors[i].target_velocity = 0.0;
        motors[i].current_velocity = 0.0;
        motors[i].current_acceleration = 0.0;
        motors[i].encoder_count = 0;
        motors[i].last_pulse_time = 0;
        motors[i].pwm_output = 0.0;
        motors[i].fault_detected = false;
        motors[i].is_moving = false;
        motors[i].encoder_a_state = false;
        motors[i].encoder_b_state = false;
    }
    
    // Initialize robot status
    robot_status = ERROR_NONE;
    
    // Initialize encoder data
    encoders[0] = {0, 0, true, MOTOR_FL_ENCODER_A, MOTOR_FL_ENCODER_B};  // Front Left
    encoders[1] = {0, 0, true, MOTOR_FR_ENCODER_A, MOTOR_FR_ENCODER_B};  // Front Right  
    encoders[2] = {0, 0, true, MOTOR_RL_ENCODER_A, MOTOR_RL_ENCODER_B};  // Rear Left
    encoders[3] = {0, 0, true, MOTOR_RR_ENCODER_A, MOTOR_RR_ENCODER_B};  // Rear Right
    
    // Initialize ROS state machine variables
    agent_state = WAITING_AGENT;
    last_state_time = 0;
    ros_entities_created = false;
    
    // Initialize ROS communication state
    micro_ros_initialized = false;
    last_command_time = 0;
    
    // Initialize inter-core communication
    velocity_cmd.linear_x = 0.0;
    velocity_cmd.linear_y = 0.0;
    velocity_cmd.angular_z = 0.0;
    velocity_cmd.timestamp = 0;
    
    odometry.position_x = 0.0;
    odometry.position_y = 0.0;
    odometry.orientation = 0.0;
    odometry.linear_velocity_x = 0.0;
    odometry.linear_velocity_y = 0.0;
    odometry.angular_velocity = 0.0;
    odometry.timestamp = 0;
    
    new_velocity_command = false;
    new_odometry_data = false;
}

// =============================================================================
// MOTOR CONTROL IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize all hardware peripherals
 * Sets up motors, encoders, and status LED
 */
void DriverBoard::initializePeripherals(void) {
    // TODO: Initialize status LED when library is available
    // status_led.begin();
    // status_led.setPixelColor(0, LED_COLOR_BLUE);
    // status_led.show();
    
    // Initialize motor control pins
    // Front Left Motor
    pinMode(MOTOR_FL_PWM_PIN, OUTPUT);
    pinMode(MOTOR_FL_DIR1_PIN, OUTPUT);
    pinMode(MOTOR_FL_DIR2_PIN, OUTPUT);
    pinMode(MOTOR_FL_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_FL_ENCODER_B, INPUT_PULLUP);
    
    // Front Right Motor
    pinMode(MOTOR_FR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_FR_DIR1_PIN, OUTPUT);
    pinMode(MOTOR_FR_DIR2_PIN, OUTPUT);
    pinMode(MOTOR_FR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_FR_ENCODER_B, INPUT_PULLUP);
    
    // Rear Left Motor
    pinMode(MOTOR_RL_PWM_PIN, OUTPUT);
    pinMode(MOTOR_RL_DIR1_PIN, OUTPUT);
    pinMode(MOTOR_RL_DIR2_PIN, OUTPUT);
    pinMode(MOTOR_RL_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_RL_ENCODER_B, INPUT_PULLUP);
    
    // Rear Right Motor
    pinMode(MOTOR_RR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_RR_DIR1_PIN, OUTPUT);
    pinMode(MOTOR_RR_DIR2_PIN, OUTPUT);
    pinMode(MOTOR_RR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_RR_ENCODER_B, INPUT_PULLUP);
    
    // Initialize PWM frequency for motor control (platform-specific)
    // analogWriteFreq(PWM_FREQUENCY); // TODO: Check if this function exists for RP2040
    
    // Setup encoder interrupts
    setupEncoderInterrupts();
    
    // Stop all motors initially
    emergencyStop();
}

/**
 * @brief Main peripheral management loop
 * Should be called regularly from Core 0 loop
 */
void DriverBoard::updatePeripherals(void) {
    // Read encoder values
    readEncoders();
    
    // Update motor control (PID control, PWM output)
    updateMotorControl();
    
    // Calculate odometry from wheel velocities
    updateOdometry();
    
    // Check for motor faults and safety conditions
    handleMotorSafety();
    
    // Update status LED based on robot state
    updateStatusLED();
}

/**
 * @brief Update motor control (PID control, PWM output)
 * Processes velocity commands and applies motor control
 */
void DriverBoard::updateMotorControl(void) {
    // TODO: Implement motor PID control
    
    // Get current velocity command
    float linear_x = velocity_cmd.linear_x;
    float linear_y = velocity_cmd.linear_y;
    float angular_z = velocity_cmd.angular_z;
    
    // Calculate target wheel velocities using inverse kinematics
    float wheel_velocities[4];
    mecanumDriveKinematics(linear_x, linear_y, angular_z, wheel_velocities);
    
    // Update target velocities for each motor
    for (int i = 0; i < 4; i++) {
        motors[i].target_velocity = wheel_velocities[i];
    }
    
    // Apply PID control and PWM output
    // TODO: Implement PID controllers for each wheel
    // For now, simple proportional control as placeholder
    for (int i = 0; i < 4; i++) {
        // Simple proportional control (placeholder)
        float error = motors[i].target_velocity - motors[i].current_velocity;
        motors[i].pwm_output = constrain(error * 100.0, -255, 255);
        
        // Apply PWM to motors (placeholder - needs proper direction control)
        switch (i) {
            case 0: // Front Left
                analogWrite(MOTOR_FL_PWM_PIN, abs((int)motors[i].pwm_output));
                digitalWrite(MOTOR_FL_DIR1_PIN, motors[i].pwm_output >= 0 ? HIGH : LOW);
                digitalWrite(MOTOR_FL_DIR2_PIN, motors[i].pwm_output >= 0 ? LOW : HIGH);
                break;
            case 1: // Front Right
                analogWrite(MOTOR_FR_PWM_PIN, abs((int)motors[i].pwm_output));
                digitalWrite(MOTOR_FR_DIR1_PIN, motors[i].pwm_output >= 0 ? HIGH : LOW);
                digitalWrite(MOTOR_FR_DIR2_PIN, motors[i].pwm_output >= 0 ? LOW : HIGH);
                break;
            case 2: // Rear Left
                analogWrite(MOTOR_RL_PWM_PIN, abs((int)motors[i].pwm_output));
                digitalWrite(MOTOR_RL_DIR1_PIN, motors[i].pwm_output >= 0 ? HIGH : LOW);
                digitalWrite(MOTOR_RL_DIR2_PIN, motors[i].pwm_output >= 0 ? LOW : HIGH);
                break;
            case 3: // Rear Right
                analogWrite(MOTOR_RR_PWM_PIN, abs((int)motors[i].pwm_output));
                digitalWrite(MOTOR_RR_DIR1_PIN, motors[i].pwm_output >= 0 ? HIGH : LOW);
                digitalWrite(MOTOR_RR_DIR2_PIN, motors[i].pwm_output >= 0 ? LOW : HIGH);
                break;
        }
    }
}

/**
 * @brief Read encoder values from all motors
 * Updates encoder counts for odometry calculations
 */
void DriverBoard::readEncoders(void) {
    static uint32_t last_encoder_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_encoder_time) / 1000.0; // Convert to seconds
    
    if (dt > 0.01) { // Update at 100Hz max
        // Process encoder data from interrupt handlers
        for (int i = 0; i < 4; i++) {
            // Copy encoder count from volatile interrupt data (atomic read)
            noInterrupts();
            int32_t current_count = encoders[i].count;
            interrupts();
            
            // Calculate velocity from encoder pulses
            static int32_t last_counts[4] = {0, 0, 0, 0};
            
            int32_t count_diff = current_count - last_counts[i];
            last_counts[i] = current_count;
            
            // Store previous velocity for acceleration calculation
            float previous_velocity = motors[i].current_velocity;
            
            // Convert encoder pulses to velocity
            // Using GEARED_COUNTS_PER_REV which accounts for gear ratio and quadrature encoding
            // and WHEEL_DIAMETER_MM wheel diameter
            if (dt > 0) {
                float revolutions_per_sec = (float)count_diff / (GEARED_COUNTS_PER_REV * dt);
                float wheel_circumference = PI * (WHEEL_DIAMETER_MM / 1000.0); // Convert to meters
                motors[i].current_velocity = revolutions_per_sec * wheel_circumference;
                
                // Calculate acceleration (change in velocity over time)
                motors[i].current_acceleration = (motors[i].current_velocity - previous_velocity) / dt;
                
                // Determine if motor is moving (velocity threshold to avoid noise)
                const float VELOCITY_THRESHOLD = 0.01; // 1 cm/s threshold
                motors[i].is_moving = (fabs(motors[i].current_velocity) > VELOCITY_THRESHOLD);
            }
            
            // Update motor encoder count
            motors[i].encoder_count = current_count;
        }
        
        last_encoder_time = current_time;
    }
}

/**
 * @brief Calculate odometry from wheel velocities
 * Updates robot position and velocity estimates
 */
void DriverBoard::updateOdometry(void) {
    // TODO: Implement proper odometry calculations
    
    // Get wheel velocities
    float wheel_velocities[4] = {
        motors[0].current_velocity, // FL
        motors[1].current_velocity, // FR
        motors[2].current_velocity, // RL
        motors[3].current_velocity  // RR
    };
    
    // Calculate robot velocities using forward kinematics
    float linear_x, linear_y, angular_z;
    mecanumDriveOdometry(wheel_velocities, &linear_x, &linear_y, &angular_z);
    
    // Update odometry structure
    static uint32_t last_odom_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_odom_time) / 1000.0;
    
    if (dt > 0.01) { // Update at 100Hz max
        // Update velocities
        odometry.linear_velocity_x = linear_x;
        odometry.linear_velocity_y = linear_y;
        odometry.angular_velocity = angular_z;
        
        // Integrate to get position (simple Euler integration)
        float cos_theta = cos(odometry.orientation);
        float sin_theta = sin(odometry.orientation);
        
        odometry.position_x += dt * (linear_x * cos_theta - linear_y * sin_theta);
        odometry.position_y += dt * (linear_x * sin_theta + linear_y * cos_theta);
        odometry.orientation += dt * angular_z;
        
        odometry.timestamp = current_time;
        new_odometry_data = true;
        
        last_odom_time = current_time;
    }
}

/**
 * @brief Check for motor faults and safety conditions
 * Monitors motor currents, temperatures, and error conditions
 */
void DriverBoard::handleMotorSafety(void) {
    // TODO: Implement safety monitoring
    // Check for overcurrent, overtemperature, encoder faults, etc.
    
    for (int i = 0; i < 4; i++) {
        // Reset fault flags (placeholder)
        motors[i].fault_detected = false;
    }
    
    robot_status = ERROR_NONE;
}

/**
 * @brief Update status LED based on robot state
 * Provides visual feedback of robot status
 */
void DriverBoard::updateStatusLED(void) {
    // TODO: Implement LED status updates when library is available
    
    // Set LED color based on robot status
    // switch (g_robot_status) {
    //     case ERROR_NONE:
    //         status_led.setPixelColor(0, LED_COLOR_GREEN);
    //         break;
    //     case ERROR_MOTOR_FAULT:
    //         status_led.setPixelColor(0, LED_COLOR_RED);
    //         break;
    //     default:
    //         status_led.setPixelColor(0, LED_COLOR_YELLOW);
    //         break;
    // }
    // status_led.show();
}

/**
 * @brief Emergency stop all motors
 * Immediately stops all motor outputs for safety
 */
void DriverBoard::emergencyStop(void) {
    // Stop all motors by setting PWM to 0
    analogWrite(MOTOR_FL_PWM_PIN, 0);
    analogWrite(MOTOR_FR_PWM_PIN, 0);
    analogWrite(MOTOR_RL_PWM_PIN, 0);
    analogWrite(MOTOR_RR_PWM_PIN, 0);
    
    // Reset velocity commands (field-by-field assignment for volatile)
    velocity_cmd.linear_x = 0.0f;
    velocity_cmd.linear_y = 0.0f;
    velocity_cmd.angular_z = 0.0f;
    velocity_cmd.timestamp = 0;
}

/**
 * @brief Reset all encoder counters to zero
 */
void DriverBoard::resetEncoders(void) {
    for (int i = 0; i < 4; i++) {
        motors[i].encoder_count = 0;
    }
}

// =============================================================================
// MECANUM DRIVE KINEMATICS
// =============================================================================

/**
 * @brief Convert robot velocities to individual wheel velocities
 * 
 * @param linear_x Forward/backward velocity (m/s)
 * @param linear_y Left/right strafe velocity (m/s)
 * @param angular_z Rotational velocity (rad/s)
 * @param wheel_velocities Output array for wheel velocities [FL, FR, RL, RR]
 */
void mecanumDriveKinematics(float linear_x, float linear_y, float angular_z, 
                           float wheel_velocities[4]) {
    // Mecanum wheel kinematics
    // Based on robot geometry and wheel arrangement
    
    float wheel_separation_x = ROBOT_CENTER_TO_WHEEL_X / 1000.0; // Convert mm to m
    float wheel_separation_y = ROBOT_CENTER_TO_WHEEL_Y / 1000.0; // Convert mm to m
    
    // Calculate wheel velocities using mecanum kinematics equations
    wheel_velocities[0] = linear_x - linear_y - angular_z * (wheel_separation_x + wheel_separation_y); // FL
    wheel_velocities[1] = linear_x + linear_y + angular_z * (wheel_separation_x + wheel_separation_y); // FR
    wheel_velocities[2] = linear_x + linear_y - angular_z * (wheel_separation_x + wheel_separation_y); // RL
    wheel_velocities[3] = linear_x - linear_y + angular_z * (wheel_separation_x + wheel_separation_y); // RR
}

/**
 * @brief Convert wheel velocities to robot velocities (forward kinematics)
 * 
 * @param wheel_velocities Input array of wheel velocities [FL, FR, RL, RR]
 * @param linear_x Output forward/backward velocity (m/s)
 * @param linear_y Output left/right strafe velocity (m/s)
 * @param angular_z Output rotational velocity (rad/s)
 */
void mecanumDriveOdometry(float wheel_velocities[4], float *linear_x, 
                         float *linear_y, float *angular_z) {
    // Mecanum wheel forward kinematics
    // Convert individual wheel velocities back to robot velocities
    
    float wheel_separation_x = ROBOT_CENTER_TO_WHEEL_X / 1000.0; // Convert mm to m
    float wheel_separation_y = ROBOT_CENTER_TO_WHEEL_Y / 1000.0; // Convert mm to m
    
    // Calculate robot velocities using forward kinematics equations
    *linear_x = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4.0;
    *linear_y = (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4.0;
    *angular_z = (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]) / 
                 (4.0 * (wheel_separation_x + wheel_separation_y));
}

// =============================================================================
// ENCODER INTERRUPT FUNCTIONS
// =============================================================================

/**
 * @brief Setup encoder interrupts for all motors
 * Attaches interrupt handlers to encoder pins
 */
void DriverBoard::setupEncoderInterrupts(void) {
    // Attach interrupts for Front Left encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENCODER_A), encoderISR_FL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FL_ENCODER_B), encoderISR_FL_B, CHANGE);
    
    // Attach interrupts for Front Right encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENCODER_A), encoderISR_FR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FR_ENCODER_B), encoderISR_FR_B, CHANGE);
    
    // Attach interrupts for Rear Left encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENCODER_A), encoderISR_RL_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RL_ENCODER_B), encoderISR_RL_B, CHANGE);
    
    // Attach interrupts for Rear Right encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENCODER_A), encoderISR_RR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RR_ENCODER_B), encoderISR_RR_B, CHANGE);
}

/**
 * @brief Process encoder interrupt for a specific motor
 * @param motor_index Motor index (0=FL, 1=FR, 2=RL, 3=RR)
 * @param pin_state Current state of the encoder pin that triggered interrupt
 * @param is_pin_a True if pin A triggered interrupt, false if pin B
 */
void DriverBoard::processEncoderInterrupt(uint8_t motor_index, bool pin_state, bool is_pin_a) {
    if (motor_index >= 4) return; // Safety check
    
    EncoderData_t* encoder = &encoders[motor_index];
    MotorState_t* motor = &motors[motor_index];
    
    // Read current states of both encoder pins
    bool state_a = digitalRead(encoder->pin_a);
    bool state_b = digitalRead(encoder->pin_b);
    
    // Store current states in motor structure
    motor->encoder_a_state = state_a;
    motor->encoder_b_state = state_b;
    
    // Determine direction based on quadrature encoding
    // For hall effect encoders, we typically see a 90-degree phase shift between A and B
    bool direction_forward;
    
    if (is_pin_a) {
        // Pin A changed - determine direction based on B state
        direction_forward = (state_a == state_b) ? false : true;
    } else {
        // Pin B changed - determine direction based on A state  
        direction_forward = (state_a == state_b) ? true : false;
    }
    
    // Update encoder count based on direction
    if (direction_forward) {
        encoder->count++;
    } else {
        encoder->count--;
    }
    
    // Store direction and timing information
    encoder->direction = direction_forward;
    encoder->last_time = micros();
    motor->last_pulse_time = encoder->last_time;
    
    // Update motor encoder count (thread-safe copy)
    motor->encoder_count = encoder->count;
}

// Interrupt Service Routines for each encoder pin
void encoderISR_FL_A(void) { rp.processEncoderInterrupt(0, digitalRead(MOTOR_FL_ENCODER_A), true); }
void encoderISR_FL_B(void) { rp.processEncoderInterrupt(0, digitalRead(MOTOR_FL_ENCODER_B), false); }
void encoderISR_FR_A(void) { rp.processEncoderInterrupt(1, digitalRead(MOTOR_FR_ENCODER_A), true); }
void encoderISR_FR_B(void) { rp.processEncoderInterrupt(1, digitalRead(MOTOR_FR_ENCODER_B), false); }
void encoderISR_RL_A(void) { rp.processEncoderInterrupt(2, digitalRead(MOTOR_RL_ENCODER_A), true); }
void encoderISR_RL_B(void) { rp.processEncoderInterrupt(2, digitalRead(MOTOR_RL_ENCODER_B), false); }
void encoderISR_RR_A(void) { rp.processEncoderInterrupt(3, digitalRead(MOTOR_RR_ENCODER_A), true); }
void encoderISR_RR_B(void) { rp.processEncoderInterrupt(3, digitalRead(MOTOR_RR_ENCODER_B), false); }