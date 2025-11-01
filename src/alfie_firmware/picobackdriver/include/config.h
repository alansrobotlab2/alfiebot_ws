/**
 * @file config.h
 * @brief Configuration file for RP2040 back Drive Robot Base
 * 
 * This file contains all configuration defines for a complete back drive
 * robot implementation including pin assignments, communication settings,
 * physical dimensions, and operational parameters.
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#pragma once

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

/**
 * @brief Serial Communication Configuration
 */
#define SERIAL_BAUD_RATE        1500000  ///< Serial port baud rate for ROS2 communication
#define SERIAL_TIMEOUT_MS       1000    ///< Serial communication timeout

/**
 * @brief Status LED Configuration
 * Built-in LED for status indication
 */
#define STATUS_LED_PIN          LED_BUILTIN  ///< Built-in LED pin
#define LED_BLINK_PERIOD_MS     125          ///< LED blink timing unit (125ms = 8 steps per second)


// ==========================================================================
// LIMIT SWITCH PIN ASSIGNMENT
// ==========================================================================
#define LIMIT_SWITCH_PIN    10       ///< Minimum position limit switch pin

// ============================================================================
// TEMPERATURE SENSOR CONFIGURATION
// ============================================================================
#define TEMP_SENSOR_PIN        26      ///< Temperature sensor pin (ADC)

// =============================================================================
// MOTOR DRIVER PIN ASSIGNMENTS
// =============================================================================

/**
 * @brief Motor Driver Pin Configuration
 * TB6612FNG motor driver for linear actuator control
 */

// Linear Actuator Motor - TB6612FNG Control Pins
#define MOTOR_PWM_PIN           2       ///< Linear actuator motor PWM pin (PWMA)
#define MOTOR_DIR1_PIN          3       ///< Linear actuator motor direction pin 1 (AIN1)
#define MOTOR_DIR2_PIN          4       ///< Linear actuator motor direction pin 2 (AIN2)
#define MOTOR_STANDBY_PIN       5       ///< Motor driver standby pin (STBY - HIGH=active, LOW=standby)
#define MOTOR_VCC_PIN           15      ///< Motor driver VCC power pin (logic power supply)

// Linear Actuator Encoder Pins
#define MOTOR_ENCODER_A         28      ///< Linear actuator motor encoder A pin
#define MOTOR_ENCODER_B         29      ///< Linear actuator motor encoder B pin

// Status LED
#define WS2812B_PIN             16      ///< WS2812B RGB LED data pin

/**
 * @brief PWM Configuration for Motors
 */
#define PWM_FREQUENCY           512     ///< PWM frequency in Hz
#define PWM_RESOLUTION          255     ///< PWM resolution (8-bit)
#define PWM_MIN_DUTY            0       ///< Minimum PWM duty cycle
#define PWM_MAX_DUTY            255     ///< Maximum PWM duty cycle

// =============================================================================
// ROBOT PHYSICAL DIMENSIONS
// =============================================================================

/**
 * @brief Linear Actuator Configuration
 */
#define ACTUATOR_MIN_POSITION    0.0     ///< Minimum actuator position (meters)
#define ACTUATOR_MAX_POSITION    0.380     ///< Maximum actuator position (meters)
#define ACTUATOR_HOME_POSITION   0.0     ///< Home/zero position (meters)

// =============================================================================
// MOTOR AND GEARING SPECIFICATIONS
// =============================================================================

/**
 * @brief Motor Specifications
 */
#define MOTOR_VOLTAGE           12.0    ///< Motor operating voltage (V)
#define MOTOR_MAX_CURRENT_MA    2000    ///< Maximum motor current (mA)
#define MOTOR_STALL_TORQUE_NM   0.5     ///< Motor stall torque (N·m)
#define MOTOR_FREE_SPEED_RPM    111     ///< Motor free speed (RPM)

/**
 * @brief Gear Ratio Configuration
 */
#define GEAR_RATIO              169.0    ///< Gear reduction ratio (motor:output shaft)
#define OUTPUT_SHAFT_MAX_RPM    (MOTOR_FREE_SPEED_RPM / GEAR_RATIO)  ///< Max output shaft RPM

/**
 * @brief Linear Actuator Mechanical Configuration
 */
#define PULLEY_TEETH            20      ///< Number of teeth on drive pulley (20T spline gear)
#define BELT_PITCH_MM           2.0     ///< GT2 timing belt pitch (mm between tooth centers)
#define PULLEY_CIRCUMFERENCE_M  ((PULLEY_TEETH * BELT_PITCH_MM) / 1000.0)  ///< Pulley circumference in meters
#define METERS_PER_REVOLUTION   PULLEY_CIRCUMFERENCE_M  ///< Linear distance per output shaft revolution

/**
 * @brief Encoder Configuration
 */
#define ENCODER_PPR             11     ///< Encoder pulses per revolution
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4)  ///< Quadrature encoding (4x)
#define GEARED_COUNTS_PER_REV   (ENCODER_COUNTS_PER_REV * GEAR_RATIO)  ///< Counts per output shaft revolution
#define ENCODER_COUNTS_PER_METER (GEARED_COUNTS_PER_REV / METERS_PER_REVOLUTION)  ///< Encoder counts per meter of linear travel



// =============================================================================
// MOTION CONTROL PARAMETERS
// =============================================================================

/**
 * @brief Maximum Velocities and Accelerations for Linear Actuator
 */
#define MAX_ACTUATOR_VELOCITY       0.1     ///< Maximum linear actuator velocity (m/s)
#define MAX_ACTUATOR_ACCELERATION   0.5     ///< Maximum linear actuator acceleration (m/s²)

/**
 * @brief PID Controller Parameters for Velocity Control
 */
#define VELOCITY_PID_KP         100.0   ///< Velocity proportional gain
#define VELOCITY_PID_KI         10.0    ///< Velocity integral gain
#define VELOCITY_PID_KD         1.0     ///< Velocity derivative gain
#define VELOCITY_PID_OUTPUT_LIMIT   255     ///< PID output limit (matches PWM range)
#define VELOCITY_PID_INTEGRAL_LIMIT 100     ///< Integral windup limit

/**
 * @brief Control Loop Timing
 */
#define CONTROL_LOOP_PERIOD_MS  2       ///< Control loop period in milliseconds (500 Hz)
#define CONTROL_LOOP_FREQ_HZ    (1000 / CONTROL_LOOP_PERIOD_MS)  ///< Control frequency



// =============================================================================
// SAFETY AND LIMITS
// =============================================================================

/**
 * @brief Safety Parameters
 */
#define WATCHDOG_TIMEOUT_MS     500     ///< Watchdog timeout for safety stop
#define EMERGENCY_STOP_DECEL    5.0     ///< Emergency stop deceleration (m/s²)
#define MIN_BATTERY_VOLTAGE     10.5    ///< Minimum battery voltage (V)
#define MAX_MOTOR_TEMP_C        80      ///< Maximum motor temperature (°C)

/**
 * @brief Error Codes
 */
#define ERROR_NONE              0x00    ///< No error
#define ERROR_INVALID_COMMAND   0x01    ///< Invalid command received
#define ERROR_MOTOR_FAULT       0x02    ///< Motor driver fault
#define ERROR_ENCODER_FAULT     0x03    ///< Encoder fault
#define ERROR_LOW_BATTERY       0x04    ///< Low battery voltage
#define ERROR_OVERTEMPERATURE   0x05    ///< Motor overtemperature
#define ERROR_COMMUNICATION     0x06    ///< Communication timeout



// =============================================================================
// MATHEMATICAL CONSTANTS
// =============================================================================

#ifndef PI
#define PI                      3.14159265358979323846  ///< Pi constant
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG              (180.0 / PI)            ///< Radians to degrees conversion
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD              (PI / 180.0)            ///< Degrees to radians conversion
#endif

#define MM_TO_M                 0.001                   ///< Millimeters to meters conversion
#define M_TO_MM                 1000.0                  ///< Meters to millimeters conversion



// =============================================================================
// DEBUG AND DEVELOPMENT
// =============================================================================

/**
 * @brief Debug Configuration
 */
#define DEBUG_ENABLED           1       ///< Enable debug output (0=off, 1=on)
#define DEBUG_MOTOR_CONTROL     1       ///< Debug motor control (0=off, 1=on)
#define DEBUG_ODOMETRY          1       ///< Debug odometry calculations (0=off, 1=on)
#define DEBUG_SERIAL_COMM       1       ///< Debug serial communication (0=off, 1=on)

/**
 * @brief Development Flags
 */
#define SIMULATE_ENCODERS       0       ///< Simulate encoder input for testing (0=off, 1=on)
#define ENABLE_MOTOR_SAFETY     1       ///< Enable motor safety checks (0=off, 1=on)
#define ENABLE_BATTERY_MONITOR  1       ///< Enable battery monitoring (0=off, 1=on)



// =============================================================================
// ROS STATE MACHINE CONFIGURATION
// =============================================================================

/**
 * @brief ROS Agent Connection States
 */
typedef enum {
    WAITING_AGENT = 0,      ///< Waiting for micro-ROS agent connection
    AGENT_AVAILABLE,        ///< Agent is available, attempting to create entities
    AGENT_CONNECTED,        ///< Agent connected and entities created successfully
    AGENT_DISCONNECTED      ///< Agent disconnected, cleanup needed
} RosAgentState_t;

/**
 * @brief ROS State Machine Timing Configuration
 */
#define ROS_TASK_FREQUENCY_HZ   100     ///< ROS task frequency (Hz)
#define ROS_TASK_PERIOD_MS      10      ///< ROS task period (ms) = 1000/FREQUENCY
#define AGENT_PING_INTERVAL_MS  100     ///< Interval to ping agent when waiting (ms)
#define AGENT_HEALTH_CHECK_MS   200     ///< Interval to check connection health (ms)
#define AGENT_PING_TIMEOUT_MS   50      ///< Timeout for agent ping when waiting (ms)
#define AGENT_HEALTH_TIMEOUT_MS 100     ///< Timeout for health check ping (ms)
#define AGENT_PING_ATTEMPTS     1       ///< Number of ping attempts when waiting
#define AGENT_HEALTH_ATTEMPTS   10      ///< Number of ping attempts for health check

/**
 * @brief Utility macro for executing code at specific intervals
 * @param interval_ms Interval in milliseconds
 * @param code Code to execute
 */
#define EXECUTE_EVERY_N_MS(interval_ms, code) \
    do { \
        static uint32_t last_execution = 0; \
        uint32_t current_time = millis(); \
        if (current_time - last_execution >= interval_ms) { \
            last_execution = current_time; \
            code; \
        } \
    } while(0)
