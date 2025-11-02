/**
 * @file motor_control.cpp
 * @brief Motor control and peripheral management implementation for linear actuator
 * 
 * @author Alfie Bot Project
 * @date 2025-10-27
 */

#include <Arduino.h>
#include <motor_control.h>
#include <../../include/driverboard.h>
#include "hardware/adc.h"

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Read RP2040 internal temperature sensor
 * @return Temperature in degrees Celsius (rounded to uint8_t)
 */
uint8_t readBoardTemperature() {
    // Enable temperature sensor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4); // ADC4 is the temperature sensor
    
    // Read raw ADC value (12-bit)
    uint16_t raw = adc_read();
    
    // Convert to voltage (3.3V reference, 12-bit ADC)
    const float conversion_factor = 3.3f / (1 << 12);
    float voltage = raw * conversion_factor;
    
    // Convert to temperature using RP2040 formula
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;
    
    // Round and clamp to uint8_t range (0-255°C)
    if (temperature < 0) temperature = 0;
    if (temperature > 255) temperature = 255;
    
    return (uint8_t)(temperature + 0.5f); // Round to nearest integer
}

// =============================================================================
// DRIVERBOARD CLASS IMPLEMENTATION
// =============================================================================

/**
 * @brief DriverBoard constructor - initializes all state variables
 */
DriverBoard::DriverBoard() 
    : statusLED(WS2812B_PIN, 800000) {  // Initialize WS2812B on pin 16 at 800kHz
    // Initialize motor state
    motor.target_position = 0.0;
    motor.target_velocity = 0.0;
    motor.max_acceleration = MAX_ACTUATOR_ACCELERATION;
    motor.current_position = 0.0;
    motor.current_velocity = 0.0;
    motor.current_acceleration = 0.0;
    motor.encoder_count = 0;
    motor.last_pulse_time = 0;
    motor.pwm_output = 0;
    motor.velocity_error_integral = 0.0;
    motor.velocity_error_previous = 0.0;
    motor.ramped_velocity = 0.0;
    motor.fault_detected = false;
    motor.is_moving = false;
    motor.encoder_a_state = false;
    motor.encoder_b_state = false;
    
    // Initialize robot status
    robot_status = ERROR_NONE;
    
    // Initialize encoder data
    encoder.count = 0;
    encoder.last_time = 0;
    encoder.direction = true;
    encoder.pin_a = MOTOR_ENCODER_A;
    encoder.pin_b = MOTOR_ENCODER_B;
    
    // Initialize inter-core communication
    actuator_cmd.position = 0.0;
    actuator_cmd.velocity = 0.0;
    actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
    actuator_cmd.timestamp = 0;
    
    actuator_state.board_temp = 0;
    actuator_state.limit_switch_triggered = false;
    actuator_state.command_position = 0.0;
    actuator_state.command_velocity = 0.0;
    actuator_state.command_acceleration = 0.0;
    actuator_state.is_moving = false;
    actuator_state.current_position = 0.0;
    actuator_state.current_velocity = 0.0;
    actuator_state.current_acceleration = 0.0;
    actuator_state.pulses = 0;
    actuator_state.is_calibrated = false;
    actuator_state.timestamp = 0;
    
    new_actuator_command = false;
    new_actuator_state = false;
    
    // Initialize calibration state
    calibration_in_progress = false;
    
}

// =============================================================================
// MOTOR CONTROL IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize all hardware peripherals
 * Sets up motor, encoder, and status LED
 */
void DriverBoard::initializePeripherals(void) {

    // Initialize WS2812B RGB LED
    if (statusLED.begin()) {
        // Set initial color to dim blue (initializing)
        statusLED.setColor(0, 0, 50);
    }
    
    // Initialize TB6612FNG motor driver power and control pins
    // VCC pin - power the motor driver logic (must be HIGH)
    pinMode(MOTOR_VCC_PIN, OUTPUT);
    digitalWrite(MOTOR_VCC_PIN, HIGH);
    
    // Standby pin - must be HIGH for normal operation, LOW puts driver in standby mode
    pinMode(MOTOR_STANDBY_PIN, OUTPUT);
    digitalWrite(MOTOR_STANDBY_PIN, HIGH);
    
    // Motor control pins (direction and PWM)
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR1_PIN, OUTPUT);
    pinMode(MOTOR_DIR2_PIN, OUTPUT);
    
    // Encoder pins with pull-up resistors
    pinMode(MOTOR_ENCODER_A, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B, INPUT_PULLUP);
    
    // Limit switch pin - active high, goes LOW when triggered (at lower position limit)
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    
    // Setup encoder interrupts
    setupEncoderInterrupts();
    
    // Stop motor initially
    emergencyStop();
}

/**
 * @brief Main peripheral management loop
 * Should be called regularly from Core 0 loop
 */
void DriverBoard::updatePeripherals(void) {
    // Read encoder value
    readEncoder();
    
    // Update actuator state (position, velocity, acceleration)
    updateActuatorState();
    
    // Update motor control (PID control, PWM output)
    updateMotorControl();
    
    // Check for motor faults and safety conditions
    handleMotorSafety();
    
    // Update RGB LED status
    updateRgbLED();    // Update status LED based on robot state
    //updateStatusLED();
}

/**
 * @brief Update motor control (PID control, PWM output)
 * Processes actuator commands and applies motor control with velocity PID and acceleration limiting
 * 
 * CONTROL STRATEGY:
 * - Position is the primary control target (actuator_cmd.position)
 * - Velocity is a CONSTRAINT specifying maximum speed to reach position (actuator_cmd.velocity)
 * - Acceleration is a CONSTRAINT specifying maximum acceleration (actuator_cmd.acceleration)
 * - The controller generates velocity setpoints from position error, clamped by velocity/accel limits
 * 
 * NOTE: Motor control is suspended during calibration to prevent interference
 */
void DriverBoard::updateMotorControl(void) {
    // Skip motor control during calibration
    if (calibration_in_progress) {
        return;
    }
    
    static uint32_t last_control_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_control_time) / 1000.0; // Convert to seconds
    
    // Initialize timing on first call
    if (last_control_time == 0) {
        last_control_time = current_time;
        return;
    }
    
    if (dt >= (CONTROL_LOOP_PERIOD_MS / 1000.0)) {
        last_control_time = current_time;
        
        // Get current actuator command
        motor.target_position = actuator_cmd.position;
        motor.max_acceleration = actuator_cmd.acceleration;
        
        // Clamp target position to limits
        motor.target_position = constrain(motor.target_position,
                                         ACTUATOR_MIN_POSITION,
                                         ACTUATOR_MAX_POSITION);
        
        // Clamp acceleration to maximum
        motor.max_acceleration = constrain(motor.max_acceleration, 
                                          0.0, 
                                          MAX_ACTUATOR_ACCELERATION);
        
        // Position control: Calculate velocity needed to reach target position
        // Velocity and acceleration from command are constraints, not direct control inputs
        float position_error = motor.target_position - motor.current_position;
        const float POSITION_TOLERANCE = 0.002; // 2mm tolerance
        
        // Simple proportional position control to generate velocity setpoint
        const float POSITION_KP = 5.0; // Position gain - increased from 3.0 for faster settling
        float desired_velocity = POSITION_KP * position_error;
        
        // Limit velocity based on commanded max velocity (if non-zero) or system max
        // actuator_cmd.velocity specifies the MAXIMUM velocity to use, not a direct velocity command
        float max_vel = (actuator_cmd.velocity != 0.0) ? fabs(actuator_cmd.velocity) : MAX_ACTUATOR_VELOCITY;
        desired_velocity = constrain(desired_velocity, -max_vel, max_vel);
        
        // Stop if within tolerance
        if (fabs(position_error) < POSITION_TOLERANCE) {
            desired_velocity = 0.0;
        }
        
        motor.target_velocity = desired_velocity;
        
        // Clamp target velocity to maximum
        motor.target_velocity = constrain(motor.target_velocity, 
                                         -MAX_ACTUATOR_VELOCITY, 
                                         MAX_ACTUATOR_VELOCITY);
        
        // Apply acceleration limiting to get ramped velocity
        motor.ramped_velocity = applyAccelerationLimit(motor.target_velocity, 
                                                       motor.ramped_velocity,
                                                       motor.max_acceleration, 
                                                       dt);
        
        // Apply velocity PID controller
        motor.pwm_output = applyVelocityPID(motor.ramped_velocity, 
                                           motor.current_velocity, 
                                           dt);
        
        
        // Apply PWM to motor
        analogWrite(MOTOR_PWM_PIN, abs((int)motor.pwm_output));
        digitalWrite(MOTOR_DIR1_PIN, motor.pwm_output >= 0 ? HIGH : LOW);
        digitalWrite(MOTOR_DIR2_PIN, motor.pwm_output >= 0 ? LOW : HIGH);
    }
}

/**
 * @brief Read encoder value from motor
 * Updates encoder count for position/velocity calculations
 */
void DriverBoard::readEncoder(void) {
    // Copy encoder count from volatile interrupt data (atomic read)
    noInterrupts();
    int32_t current_count = encoder.count;
    interrupts();
    
    // Update motor encoder count
    motor.encoder_count = current_count;
}

/**
 * @brief Calculate actuator state from encoder
 * Updates position, velocity, and acceleration estimates
 */
void DriverBoard::updateActuatorState(void) {
    static uint32_t last_update_time = 0;
    static uint32_t last_temp_update = 0;
    static int32_t last_count = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
    
    // Initialize timing on first call
    if (last_update_time == 0) {
        last_update_time = current_time;
        last_temp_update = current_time;
        return;
    }
    
    if (dt >= 0.002) { // Update at 500Hz (2ms period)
        // Calculate position from encoder counts
        motor.current_position = (float)motor.encoder_count / ENCODER_COUNTS_PER_METER;
        
        // Calculate velocity from encoder pulses
        int32_t count_diff = motor.encoder_count - last_count;
        last_count = motor.encoder_count;
        
        // Store previous velocity for acceleration calculation
        float previous_velocity = motor.current_velocity;
        
        // Convert encoder counts to velocity (m/s)
        float raw_velocity = 0.0;
        if (dt > 0) {
            raw_velocity = ((float)count_diff / ENCODER_COUNTS_PER_METER) / dt;
            
            // Apply low-pass filter to smooth velocity (reduce encoder quantization noise)
            // Exponential moving average: filtered = alpha * new + (1-alpha) * old
            const float VELOCITY_FILTER_ALPHA = 0.3; // 0.3 = moderate filtering
            motor.current_velocity = VELOCITY_FILTER_ALPHA * raw_velocity + 
                                    (1.0 - VELOCITY_FILTER_ALPHA) * motor.current_velocity;
            
            // Calculate acceleration (change in velocity over time)
            motor.current_acceleration = (motor.current_velocity - previous_velocity) / dt;
            
            // Determine if motor is moving (velocity threshold to avoid noise)
            const float VELOCITY_THRESHOLD = 0.001; // 1 mm/s threshold
            motor.is_moving = (fabs(motor.current_velocity) > VELOCITY_THRESHOLD);
        }
        
        // Read board temperature every 1000ms (1 second) to avoid excessive ADC reads
        if (current_time - last_temp_update >= 1000) {
            actuator_state.board_temp = readBoardTemperature();
            last_temp_update = current_time;
        }
        
        // Read limit switch state (active high, goes LOW when triggered)
        // Reading inverted: LOW (triggered) = true, HIGH (not triggered) = false
        actuator_state.limit_switch_triggered = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);
        
        // Populate actuator state for ROS publishing
        actuator_state.command_position = actuator_cmd.position;
        actuator_state.command_velocity = actuator_cmd.velocity;
        actuator_state.command_acceleration = actuator_cmd.acceleration;
        actuator_state.is_moving = motor.is_moving;
        actuator_state.current_position = motor.current_position;
        actuator_state.current_velocity = motor.current_velocity;
        actuator_state.current_acceleration = motor.current_acceleration;
        actuator_state.pulses = motor.encoder_count;
        actuator_state.pwm_output = (uint8_t)abs(motor.pwm_output);
        actuator_state.timestamp = current_time;
        new_actuator_state = true;
        
        last_update_time = current_time;
    }
}

/**
 * @brief Check for motor faults and safety conditions
 * Monitors motor currents, temperatures, and error conditions
 */
void DriverBoard::handleMotorSafety(void) {
    // TODO: Implement safety monitoring
    // Check for overcurrent, overtemperature, encoder faults, etc.
    
    // Reset fault flag (placeholder)
    motor.fault_detected = false;
    
    robot_status = ERROR_NONE;
}

/**
 * @brief Update status LED based on robot state
 * Provides visual feedback of robot status
 */
void DriverBoard::updateStatusLED(void) {
    // LED status is now handled in main.cpp Core 0 loop
}

/**
 * @brief Emergency stop motor
 * Immediately stops motor output for safety
 */
void DriverBoard::emergencyStop(void) {
    // Stop motor by setting PWM to 0 and both direction pins LOW (brake mode for TB6612FNG)
    analogWrite(MOTOR_PWM_PIN, 0);
    digitalWrite(MOTOR_DIR1_PIN, LOW);
    digitalWrite(MOTOR_DIR2_PIN, LOW);
    // Note: MOTOR_STANDBY_PIN remains HIGH to keep driver active
    // Set it LOW only if you want to disable the driver completely
    
    // Reset command (field-by-field assignment for volatile)
    actuator_cmd.position = motor.current_position;
    actuator_cmd.velocity = 0.0f;
    actuator_cmd.acceleration = MAX_ACTUATOR_ACCELERATION;
    actuator_cmd.timestamp = 0;
    
    // Reset PID state
    motor.velocity_error_integral = 0.0;
    motor.velocity_error_previous = 0.0;
    motor.ramped_velocity = 0.0;
}

/**
 * @brief Reset encoder counter to zero
 */
void DriverBoard::resetEncoders(void) {
    // Reset volatile encoder count (with interrupts disabled for atomic access)
    noInterrupts();
    encoder.count = 0;
    interrupts();
    
    // Reset motor encoder count and position
    motor.encoder_count = 0;
    motor.current_position = 0.0;
}


// =============================================================================
// ENCODER INTERRUPT FUNCTIONS
// =============================================================================

/**
 * @brief Setup encoder interrupts for motor
 * Attaches interrupt handlers to encoder pins
 */
void DriverBoard::setupEncoderInterrupts(void) {
    // Attach interrupts for encoder
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_A), encoderISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_ENCODER_B), encoderISR_B, CHANGE);
}

/**
 * @brief Process encoder interrupt
 * @param pin_state Current state of the encoder pin that triggered interrupt
 * @param is_pin_a True if pin A triggered interrupt, false if pin B
 */
void DriverBoard::processEncoderInterrupt(bool pin_state, bool is_pin_a) {
    // Read current states of both encoder pins
    bool state_a = digitalRead(encoder.pin_a);
    bool state_b = digitalRead(encoder.pin_b);
    
    // Store current states in motor structure
    motor.encoder_a_state = state_a;
    motor.encoder_b_state = state_b;
    
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
        encoder.count++;
    } else {
        encoder.count--;
    }
    
    // Store direction and timing information
    encoder.direction = direction_forward;
    encoder.last_time = micros();
    motor.last_pulse_time = encoder.last_time;
    
    // Update motor encoder count (thread-safe copy)
    motor.encoder_count = encoder.count;
}

// Interrupt Service Routines for encoder pins
void encoderISR_A(void) { rp.processEncoderInterrupt(digitalRead(MOTOR_ENCODER_A), true); }
void encoderISR_B(void) { rp.processEncoderInterrupt(digitalRead(MOTOR_ENCODER_B), false); }

// =============================================================================
// LINEAR ACTUATOR CONTROL FUNCTIONS
// =============================================================================

/**
 * @brief Apply velocity PID controller
 * 
 * @param target_velocity Desired velocity (m/s)
 * @param current_velocity Measured velocity (m/s)
 * @param dt Time step (seconds)
 * @return PWM output value (-255 to 255)
 */
int16_t applyVelocityPID(float target_velocity, float current_velocity, float dt) {
    // Calculate velocity error
    float error = target_velocity - current_velocity;
    
    // Proportional term
    float p_term = VELOCITY_PID_KP * error;
    
    // Integral term with anti-windup and reset when target velocity is zero
    // Reset integral when stopped to prevent wind-up during settling
    if (target_velocity == 0.0 && fabs(current_velocity) < 0.003) {
        // Reset integral when we want to be stopped and we're nearly stopped
        rp.motor.velocity_error_integral = 0.0;
    } else {
        rp.motor.velocity_error_integral += error * dt;
        rp.motor.velocity_error_integral = constrain(rp.motor.velocity_error_integral, 
                                                      -VELOCITY_PID_INTEGRAL_LIMIT, 
                                                      VELOCITY_PID_INTEGRAL_LIMIT);
    }
    float i_term = VELOCITY_PID_KI * rp.motor.velocity_error_integral;
    
    // Derivative term
    float d_term = 0.0;
    if (dt > 0) {
        float error_derivative = (error - rp.motor.velocity_error_previous) / dt;
        d_term = VELOCITY_PID_KD * error_derivative;
    }
    rp.motor.velocity_error_previous = error;
    
    // Calculate total PID output
    float pid_output = p_term + i_term + d_term;
    
    // Smoother deadband compensation to overcome static friction
    // Only add compensation when starting from zero velocity, not continuously
    // This prevents oscillation while still helping overcome initial stiction
    const float MIN_PWM_OFFSET_UP = 30.0;   // Offset for upward motion (against gravity)
    const float MIN_PWM_OFFSET_DOWN = 15.0; // Offset for downward motion (with gravity)
    const float VELOCITY_DEADZONE = 0.005;  // Velocity threshold for deadband application (m/s)
    
    // Only apply deadband compensation when:
    // 1. Target velocity is non-zero (we want to move)
    // 2. Current velocity is near zero (we're starting or stopped)
    // 3. PID output is small (might not overcome friction)
    if (target_velocity != 0.0 && fabs(current_velocity) < VELOCITY_DEADZONE) {
        if (pid_output > 0 && pid_output < MIN_PWM_OFFSET_UP) {
            // Add offset to help overcome friction when starting upward motion
            pid_output += MIN_PWM_OFFSET_UP;
        } else if (pid_output < 0 && pid_output > -MIN_PWM_OFFSET_DOWN) {
            // Add offset to help overcome friction when starting downward motion
            pid_output -= MIN_PWM_OFFSET_DOWN;
        }
    }
    
    // Clamp output to PWM range
    int16_t pwm_output = (int16_t)constrain(pid_output, 
                                            -VELOCITY_PID_OUTPUT_LIMIT, 
                                            VELOCITY_PID_OUTPUT_LIMIT);
    
    // Apply minimum PWM thresholds when any non-zero PWM is commanded
    // This ensures motor always has enough power to overcome friction and gravity
    if (pwm_output > 0 && pwm_output < MIN_PWM_UPWARD) {
        pwm_output = MIN_PWM_UPWARD;
    } else if (pwm_output < 0 && pwm_output > -MIN_PWM_DOWNWARD) {
        pwm_output = -MIN_PWM_DOWNWARD;
    }
    
    return pwm_output;
}

/**
 * @brief Apply acceleration limiting to velocity command
 * 
 * @param target_velocity Desired velocity (m/s)
 * @param current_velocity Current ramped velocity (m/s)
 * @param max_acceleration Maximum allowed acceleration (m/s²)
 * @param dt Time step (seconds)
 * @return Ramped velocity respecting acceleration limits (m/s)
 */
float applyAccelerationLimit(float target_velocity, float current_velocity, 
                             float max_acceleration, float dt) {
    if (dt <= 0) {
        return current_velocity;
    }
    
    // Calculate maximum velocity change allowed in this time step
    float max_velocity_change = max_acceleration * dt;
    
    // Calculate desired velocity change
    float velocity_diff = target_velocity - current_velocity;
    
    // Limit velocity change to maximum acceleration
    float limited_velocity_change = constrain(velocity_diff, 
                                              -max_velocity_change, 
                                              max_velocity_change);
    
    // Calculate ramped velocity
    float ramped_velocity = current_velocity + limited_velocity_change;
    
    return ramped_velocity;
}
/**
 * @brief Update RGB LED based on robot and ROS state
 * Provides visual feedback using WS2812B RGB LED with color coding:
 * - Blue: Initializing/Waiting for ROS agent
 * - Yellow: ROS agent connected, creating entities
 * - Green: Fully operational (agent connected, entities ready)
 * - Red: Error state or fault detected
 * - Cyan: Moving (when actuator is in motion)
 */
void DriverBoard::updateRgbLED(void) {
    static uint32_t last_update_time = 0;
    static uint8_t led_step = 0;
    uint32_t current_time = millis();
    
    // Use the global agent_state from ros_interface.cpp (managed by Core 1)
    // This is declared as extern in ros_interface.h
    extern RosAgentState_t agent_state;
    extern bool ros_entities_created;
    
    // Check for fault conditions first
    if (motor.fault_detected || robot_status != ERROR_NONE) {
        // Solid red for errors
        statusLED.setColor(100, 0, 0);
        return;
    }
    
    // Update LED pattern every LED_BLINK_PERIOD_MS (125ms) to match main.cpp pattern timing
    if (current_time - last_update_time >= LED_BLINK_PERIOD_MS) {
        last_update_time = current_time;
        
        // Define blink patterns (same as in main.cpp)
        // Normal pattern: 1,0,1,0,1,0,1,0 (when system loaded but ROS not connected)
        // Heartbeat pattern: 1,0,1,0,0,0,0,0 (when ROS is connected)
        const uint8_t normal_pattern[] = {1, 0, 1, 0, 1, 0, 1, 0};
        const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
        
        // Show state based on ROS connection and activity
        switch (agent_state) {
            case WAITING_AGENT:
                // Dim blue - waiting for ROS agent connection
                statusLED.setColor(0, 0, 50);
                break;
                
            case AGENT_AVAILABLE:
                // Yellow - agent connected, creating entities
                statusLED.setColor(100, 80, 0);
                break;
                
            case AGENT_CONNECTED:
                if (!ros_entities_created) {
                    // Yellow - agent connected, creating entities
                    statusLED.setColor(100, 80, 0);
                } else {
                    // Check if motor is moving
                    if (motor.is_moving) {
                        // Cyan - actively moving
                        statusLED.setColor(0, 100, 100);
                    } else {
                        // Green heartbeat - fully operational and idle
                        // Use heartbeat pattern: 1,0,1,0,0,0,0,0
                        if (heartbeat_pattern[led_step]) {
                            statusLED.setColor(0, 100, 0);  // Green ON
                        } else {
                            statusLED.setColor(0, 0, 0);    // OFF
                        }
                    }
                }
                break;
                
            case AGENT_DISCONNECTED:
                // Orange - agent disconnected
                statusLED.setColor(100, 30, 0);
                break;
                
            default:
                // Dim white - unknown state
                statusLED.setColor(20, 20, 20);
                break;
        }
        
        // Advance to next step (8 steps total, wraps around)
        led_step = (led_step + 1) % 8;
    }
}
