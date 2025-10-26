#pragma once

#include "config.h"

/**
 * @brief Initializes motor control hardware including PWM channels and encoder inputs
 * 
 * Configures the ESP32 GPIO pins for the TB6612FNG motor driver including:
 * - Direction control pins (AIN1, AIN2, BIN1, BIN2) as outputs
 * - PWM output pins (PWMA, PWMB) as outputs
 * - Sets up LEDC PWM channels with configured frequency and resolution
 * - Binds PWM channels to physical pins
 * - Configures encoder input pins with internal pull-up resistors (if DRIVERBOARD == 0)
 * 
 * Internal pull-up resistors are enabled on encoder pins to prevent floating/indeterminate
 * states and ensure reliable signal detection from open-drain/open-collector encoder outputs.
 * 
 * @note Should be called once during system initialization
 * @note Encoder setup is conditional on DRIVERBOARD == 0
 * @see CHANNEL_A, CHANNEL_B for PWM channel definitions
 * @see PWMFREQUENCY, RESOLUTION for PWM configuration
 */
void initMotors();


/**
 * @brief Drives Motor A with specified PWM duty cycle and direction
 * 
 * Controls Motor A speed and direction by setting direction pins and PWM output.
 * Positive values drive forward, negative values drive backward. PWM is constrained
 * to the range -255 to +255.
 * 
 * Special handling: If DRIVERBOARD == 1 (eye driver board), the motor wiring is
 * reversed so PWM is negated to compensate.
 * 
 * @param pwm PWM duty cycle value: -255 (full reverse) to +255 (full forward), 0 = stop
 * @note Uses digital pins AIN1, AIN2 for direction control
 * @note Uses LEDC channel CHANNEL_A for PWM output on pin PWMA
 * @warning PWM values outside ±255 are automatically constrained
 */
void DriveA(int16_t pwm);


/**
 * @brief Drives Motor B with specified PWM duty cycle and direction
 * 
 * Controls Motor B speed and direction by setting direction pins and PWM output.
 * Positive values drive forward, negative values drive backward. PWM is constrained
 * to the range -255 to +255.
 * 
 * Special handling: If DRIVERBOARD == 1 (eye driver board), the motor wiring is
 * reversed so PWM is negated to compensate.
 * 
 * @param pwm PWM duty cycle value: -255 (full reverse) to +255 (full forward), 0 = stop
 * @note Uses digital pins BIN1, BIN2 for direction control
 * @note Uses LEDC channel CHANNEL_B for PWM output on pin PWMB
 * @warning PWM values outside ±255 are automatically constrained
 */
void DriveB(int16_t pwm);


/**
 * @brief Computes PID control output for wheel velocity control
 * 
 * Implements a PID (Proportional-Integral-Derivative) controller to convert
 * a target velocity (m/s) into a PWM duty cycle command using encoder feedback.
 * 
 * The controller:
 * - Proportional term: Responds to current velocity error
 * - Integral term: Eliminates steady-state error (with anti-windup)
 * - Derivative term: Reduces overshoot and oscillation
 * 
 * Features:
 * - Anti-windup: Integral term is clamped to prevent excessive accumulation
 * - Dead zone: Velocities below MIN_VELOCITY_THRESHOLD result in 0 PWM
 * - Output saturation: PWM is constrained to [-255, +255]
 * 
 * @param target_velocity Desired velocity in meters per second (m/s)
 * @param current_velocity Measured velocity from encoders in m/s
 * @param error_integral Reference to integral error accumulator (modified)
 * @param error_previous Reference to previous error for derivative term (modified)
 * @param dt Time delta since last update in seconds
 * @return int16_t PWM duty cycle command [-255, +255]
 * @note Uses PID gains defined in config.h: VELOCITY_KP, VELOCITY_KI, VELOCITY_KD
 * @see VELOCITY_INTEGRAL_MAX for anti-windup limit
 */
int16_t calculateVelocityPID(float target_velocity, float current_velocity, 
                             float &error_integral, float &error_previous, float dt);


/**
 * @brief Sets target velocity for Motor A
 * 
 * Updates the commanded target velocity for Motor A. This velocity will be
 * achieved through closed-loop PID control using encoder feedback.
 * 
 * @param velocity Target velocity in meters per second (m/s), positive = forward
 * @note Velocity is clamped to [-MAX_VELOCITY_MPS, +MAX_VELOCITY_MPS]
 */
void setMotorATargetVelocity(float velocity);


/**
 * @brief Sets target velocity for Motor B
 * 
 * Updates the commanded target velocity for Motor B. This velocity will be
 * achieved through closed-loop PID control using encoder feedback.
 * 
 * @param velocity Target velocity in meters per second (m/s), positive = forward
 * @note Velocity is clamped to [-MAX_VELOCITY_MPS, +MAX_VELOCITY_MPS]
 */
void setMotorBTargetVelocity(float velocity);


/**
 * @brief Applies current motor commands from command buffer to both motors
 * 
 * Reads PWM values from the global driver command buffer (drivercmdbuf[0] and
 * drivercmdbuf[1]) and applies them to Motor A and Motor B respectively by
 * calling DriveA() and DriveB().
 * 
 * @note Called from main control loop to execute motor commands
 * @note Command buffer is typically updated by ROS subscription callback
 * @see DriveA() for Motor A control
 * @see DriveB() for Motor B control
 */
void driveMotors();


/**
 * @brief Executes PID velocity control loop for both motors
 * 
 * Performs closed-loop velocity control using encoder feedback:
 * 1. Reads target velocities from DriverBoard (A_target_velocity, B_target_velocity)
 * 2. Reads current velocities from encoder measurements (A_velocity, B_velocity)
 * 3. Computes PID control output to minimize velocity error
 * 4. Applies PWM commands to motors
 * 
 * Should be called at regular intervals (e.g., 50Hz) for stable control.
 * Uses calculateVelocityPID() for each motor independently.
 * 
 * @note Updates motor PWM outputs directly via DriveA() and DriveB()
 * @note Requires calculateMotorDynamics() to be called to update velocity measurements
 * @see calculateVelocityPID() for PID implementation details
 * @see VELOCITY_CONTROL_HZ for recommended update rate
 */
void driveMotorsWithVelocityControl();


/**
 * @brief Applies current motor commands from command buffer to both motors
 * 
 * Reads PWM values from the global driver command buffer (drivercmdbuf[0] and
 * drivercmdbuf[1]) and applies them to Motor A and Motor B respectively by
 * calling DriveA() and DriveB().
 * 
 * @note Called from main control loop to execute motor commands
 * @note Command buffer is typically updated by ROS subscription callback
 * @deprecated Use driveMotorsWithVelocityControl() for closed-loop velocity control
 * @see DriveA() for Motor A control
 * @see DriveB() for Motor B control
 */
void driveMotorsDirectPWM();


/**
 * @brief Disables both motors by clearing command buffer
 * 
 * Sets both motor command values in the driver command buffer to 0, effectively
 * preparing motors to be stopped on the next driveMotors() call. This does not
 * immediately stop the motors - it only stages the stop command.
 * 
 * @note Only updates command buffer - motors continue until next driveMotors() call
 * @note Used for emergency stops and watchdog timeout conditions
 * @see driveMotors() for actual motor control execution
 */
void disableAllMotors();


/**
 * @brief Interrupt service routine for Motor A encoder channel B pulses
 * 
 * Called automatically when the Motor A encoder channel B (AENCB) signal transitions
 * from LOW to HIGH. Increments or decrements the pulse counter based on the state
 * of encoder channel A (AENCA) to determine rotation direction.
 * 
 * IRAM_ATTR places this function in instruction RAM (IRAM) for faster execution,
 * which is critical for high-frequency interrupt handlers to minimize latency.
 * 
 * @note Automatically called by interrupt hardware - do not call directly
 * @note Updates global b.A_wheel_pulse_count
 * @note Must be attached to AENCB pin using attachInterrupt() during initialization
 * @warning Keep code minimal and fast - executes in interrupt context
 */
void IRAM_ATTR A_wheel_pulse();


/**
 * @brief Interrupt service routine for Motor B encoder channel B pulses
 * 
 * Called automatically when the Motor B encoder channel B (BENCB) signal transitions
 * from LOW to HIGH. Increments or decrements the pulse counter based on the state
 * of encoder channel A (BENCA) to determine rotation direction.
 * 
 * IRAM_ATTR places this function in instruction RAM (IRAM) for faster execution,
 * which is critical for high-frequency interrupt handlers to minimize latency.
 * 
 * @note Automatically called by interrupt hardware - do not call directly
 * @note Updates global b.B_wheel_pulse_count
 * @note Must be attached to BENCB pin using attachInterrupt() during initialization
 * @warning Keep code minimal and fast - executes in interrupt context
 */
void IRAM_ATTR B_wheel_pulse();


/**
 * @brief Calculates velocity, acceleration, and movement status for both motors
 * 
 * Computes real-time motor dynamics from encoder pulse counts including:
 * - Velocity in meters per second (m/s) using METERS_PER_PULSE conversion
 * - Acceleration in meters per second squared (m/s²)
 * - Movement detection using velocity threshold (>5 pulses/sec ≈ 0.001 m/s)
 * 
 * Calculations are performed for both motors independently using time deltas
 * between calls. First call initializes tracking variables. Updates global
 * DriverBoard structure with current dynamics.
 * 
 * Algorithm:
 * 1. Calculate time delta since last call
 * 2. Calculate pulse count delta
 * 3. Compute velocity = delta_pulses / delta_time * METERS_PER_PULSE
 * 4. Compute acceleration = (current_velocity - last_velocity) / delta_time
 * 5. Determine movement status based on velocity threshold
 * 
 * @note Called periodically from main control loop
 * @note Updates DriverBoard members: A_velocity, A_acceleration, A_is_moving, B_velocity, B_acceleration, B_is_moving
 * @note Requires encoder interrupts to be active and updating pulse counts
 * @see A_wheel_pulse() and B_wheel_pulse() for encoder counting
 * @see METERS_PER_PULSE for velocity conversion constant
 */
void calculateMotorDynamics();
