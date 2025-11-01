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
    
    // Initialize ROS state machine variables
    agent_state = WAITING_AGENT;
    last_state_time = 0;
    ros_entities_created = false;
    
    // Initialize ROS communication state
    micro_ros_initialized = false;
    last_command_time = 0;
    
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
    actuator_state.timestamp = 0;
    
    new_actuator_command = false;
    new_actuator_state = false;
}

// =============================================================================
// MOTOR CONTROL IMPLEMENTATION
// =============================================================================

/**
 * @brief Initialize all hardware peripherals
 * Sets up motor, encoder, and status LED
 */
void DriverBoard::initializePeripherals(void) {
    // Initialize built-in LED for status indication
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
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
 */
void DriverBoard::updateMotorControl(void) {
    static uint32_t last_control_time = 0;
    uint32_t current_time = millis();
    float dt = (current_time - last_control_time) / 1000.0; // Convert to seconds
    
    if (dt >= (CONTROL_LOOP_PERIOD_MS / 1000.0)) {
        last_control_time = current_time;
        
        // Get current actuator command
        motor.target_velocity = actuator_cmd.velocity;
        motor.max_acceleration = actuator_cmd.acceleration;
        
        // Clamp target velocity to maximum
        motor.target_velocity = constrain(motor.target_velocity, 
                                         -MAX_ACTUATOR_VELOCITY, 
                                         MAX_ACTUATOR_VELOCITY);
        
        // Clamp acceleration to maximum
        motor.max_acceleration = constrain(motor.max_acceleration, 
                                          0.0, 
                                          MAX_ACTUATOR_ACCELERATION);
        
        // Apply acceleration limiting to get ramped velocity
        motor.ramped_velocity = applyAccelerationLimit(motor.target_velocity, 
                                                       motor.ramped_velocity,
                                                       motor.max_acceleration, 
                                                       dt);
        
        // Apply velocity PID controller
        motor.pwm_output = applyVelocityPID(motor.ramped_velocity, 
                                           motor.current_velocity, 
                                           dt);
        
        // Check position limits and stop if exceeded
        if (motor.current_position <= ACTUATOR_MIN_POSITION && motor.pwm_output < 0) {
            motor.pwm_output = 0; // Stop if at minimum and trying to go lower
        }
        if (motor.current_position >= ACTUATOR_MAX_POSITION && motor.pwm_output > 0) {
            motor.pwm_output = 0; // Stop if at maximum and trying to go higher
        }
        
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
    
    if (dt >= 0.002) { // Update at 500Hz (2ms period)
        // Calculate position from encoder counts
        motor.current_position = (float)motor.encoder_count / ENCODER_COUNTS_PER_METER;
        
        // Calculate velocity from encoder pulses
        int32_t count_diff = motor.encoder_count - last_count;
        last_count = motor.encoder_count;
        
        // Store previous velocity for acceleration calculation
        float previous_velocity = motor.current_velocity;
        
        // Convert encoder counts to velocity (m/s)
        if (dt > 0) {
            motor.current_velocity = ((float)count_diff / ENCODER_COUNTS_PER_METER) / dt;
            
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
        actuator_state.limit_switch_triggered = (digitalRead(LIMIT_SWITCH_PIN) == LOW);
        
        // Populate actuator state for ROS publishing
        actuator_state.command_position = actuator_cmd.position;
        actuator_state.command_velocity = actuator_cmd.velocity;
        actuator_state.command_acceleration = actuator_cmd.acceleration;
        actuator_state.is_moving = motor.is_moving;
        actuator_state.current_position = motor.current_position;
        actuator_state.current_velocity = motor.current_velocity;
        actuator_state.current_acceleration = motor.current_acceleration;
        actuator_state.pulses = motor.encoder_count;
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
    
    // Integral term with anti-windup
    rp.motor.velocity_error_integral += error * dt;
    rp.motor.velocity_error_integral = constrain(rp.motor.velocity_error_integral, 
                                                  -VELOCITY_PID_INTEGRAL_LIMIT, 
                                                  VELOCITY_PID_INTEGRAL_LIMIT);
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
    
    // Clamp output to PWM range
    int16_t pwm_output = (int16_t)constrain(pid_output, 
                                            -VELOCITY_PID_OUTPUT_LIMIT, 
                                            VELOCITY_PID_OUTPUT_LIMIT);
    
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
    static uint8_t breathing_phase = 0;
    uint32_t current_time = millis();
    
    // Update LED color every 50ms for smooth breathing effect
    if (current_time - last_update_time < 50) {
        return;
    }
    last_update_time = current_time;
    
    // Check for fault conditions first
    if (motor.fault_detected || robot_status != ERROR_NONE) {
        // Pulsing red for errors
        uint8_t brightness = 50 + (abs(128 - breathing_phase) >> 1);
        statusLED.setColor(brightness, 0, 0);
        breathing_phase = (breathing_phase + 10) % 256;
        return;
    }
    
    // Show state based on ROS connection and activity
    switch (agent_state) {
        case WAITING_AGENT:
            // Dim blue - waiting for ROS agent connection
            statusLED.setColor(0, 0, 50);
            break;
            
        case AGENT_AVAILABLE:
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
                    // Green - fully operational and idle
                    // Gentle breathing effect
                    uint8_t brightness = 30 + (abs(128 - breathing_phase) >> 2);
                    statusLED.setColor(0, brightness, 0);
                    breathing_phase = (breathing_phase + 5) % 256;
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
}
