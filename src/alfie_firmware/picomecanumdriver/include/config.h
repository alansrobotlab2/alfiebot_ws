/**
 * @file config.h
 * @brief Configuration file for RP2040 Mecanum Drive Robot Base
 * 
 * This file contains all configuration defines for a complete mecanum drive
 * robot implementation including pin assignments, communication settings,
 * physical dimensions, and operational parameters.
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

/**
 * @brief Serial Communication Configuration
 */
#define SERIAL_BAUD_RATE        115200  ///< Serial port baud rate for ROS2 communication
#define SERIAL_TIMEOUT_MS       1000    ///< Serial communication timeout

/**
 * @brief Status LED Configuration
 * Built-in LED for status indication
 */
#define STATUS_LED_PIN          LED_BUILTIN  ///< Built-in LED pin
#define LED_BLINK_PERIOD_MS     125          ///< LED blink timing unit (125ms = 8 steps per second)



// =============================================================================
// MOTOR DRIVER PIN ASSIGNMENTS
// =============================================================================

/**
 * @brief Motor Driver Pin Configuration
 * Assumes 4 motor drivers for mecanum wheels (Front-Left, Front-Right, Rear-Left, Rear-Right)
 */

// Front Left Motor (Motor 1)
#define MOTOR_FL_PWM_PIN        6       ///< Front-Left motor PWM pin
#define MOTOR_FL_DIR1_PIN       7       ///< Front-Left motor direction pin 1
#define MOTOR_FL_DIR2_PIN       8       ///< Front-Left motor direction pin 2
#define MOTOR_FL_ENCODER_A      9       ///< Front-Left motor encoder A pin
#define MOTOR_FL_ENCODER_B      10      ///< Front-Left motor encoder B pin

// Front Right Motor (Motor 2)
#define MOTOR_FR_PWM_PIN        11      ///< Front-Right motor PWM pin
#define MOTOR_FR_DIR1_PIN       12      ///< Front-Right motor direction pin 1
#define MOTOR_FR_DIR2_PIN       13      ///< Front-Right motor direction pin 2
#define MOTOR_FR_ENCODER_A      14      ///< Front-Right motor encoder A pin
#define MOTOR_FR_ENCODER_B      15      ///< Front-Right motor encoder B pin

// Rear Left Motor (Motor 3)
#define MOTOR_RL_PWM_PIN        18      ///< Rear-Left motor PWM pin
#define MOTOR_RL_DIR1_PIN       19      ///< Rear-Left motor direction pin 1
#define MOTOR_RL_DIR2_PIN       20      ///< Rear-Left motor direction pin 2
#define MOTOR_RL_ENCODER_A      21      ///< Rear-Left motor encoder A pin
#define MOTOR_RL_ENCODER_B      22      ///< Rear-Left motor encoder B pin

// Rear Right Motor (Motor 4)
#define MOTOR_RR_PWM_PIN        26      ///< Rear-Right motor PWM pin
#define MOTOR_RR_DIR1_PIN       27      ///< Rear-Right motor direction pin 1
#define MOTOR_RR_DIR2_PIN       28      ///< Rear-Right motor direction pin 2
#define MOTOR_RR_ENCODER_A      2       ///< Rear-Right motor encoder A pin
#define MOTOR_RR_ENCODER_B      3       ///< Rear-Right motor encoder B pin

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
 * @brief Wheel Configuration
 */
#define WHEEL_DIAMETER_MM       80.0   ///< Wheel diameter in millimeters
#define WHEEL_RADIUS_MM         (WHEEL_DIAMETER_MM / 2.0)  ///< Wheel radius in mm
#define WHEEL_CIRCUMFERENCE_MM  (PI * WHEEL_DIAMETER_MM)   ///< Wheel circumference in mm

/**
 * @brief Robot Base Dimensions
 */
#define WHEELBASE_LENGTH_MM     170.5   ///< Distance between front and rear axles (mm)
#define WHEELBASE_WIDTH_MM      183.0   ///< Distance between left and right wheels (mm)
#define ROBOT_CENTER_TO_WHEEL_X (WHEELBASE_LENGTH_MM / 2.0)  ///< X distance from center to wheel
#define ROBOT_CENTER_TO_WHEEL_Y (WHEELBASE_WIDTH_MM / 2.0)   ///< Y distance from center to wheel

/**
 * @brief Robot Weight and Load Capacity
 */
#define ROBOT_WEIGHT_KG         10.0     ///< Robot weight in kilograms
#define MAX_PAYLOAD_KG          1.0     ///< Maximum payload capacity in kg
#define TOTAL_WEIGHT_KG         (ROBOT_WEIGHT_KG + MAX_PAYLOAD_KG)



// =============================================================================
// MOTOR AND GEARING SPECIFICATIONS
// =============================================================================

/**
 * @brief Motor Specifications
 */
#define MOTOR_VOLTAGE           12.0    ///< Motor operating voltage (V)
#define MOTOR_MAX_CURRENT_MA    2000    ///< Maximum motor current (mA)
#define MOTOR_STALL_TORQUE_NM   0.5     ///< Motor stall torque (N·m)
#define MOTOR_FREE_SPEED_RPM    60     ///< Motor free speed (RPM)

/**
 * @brief Gear Ratio Configuration
 */
#define GEAR_RATIO              169.0    ///< Gear reduction ratio (motor:wheel)
#define WHEEL_MAX_RPM           (MOTOR_FREE_SPEED_RPM / GEAR_RATIO)  ///< Max wheel RPM

/**
 * @brief Encoder Configuration
 */
#define ENCODER_PPR             11     ///< Encoder pulses per revolution
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * 4)  ///< Quadrature encoding (4x)
#define GEARED_COUNTS_PER_REV   (ENCODER_COUNTS_PER_REV * GEAR_RATIO)  ///< Counts per wheel revolution



// =============================================================================
// MOTION CONTROL PARAMETERS
// =============================================================================

/**
 * @brief Maximum Velocities and Accelerations
 */
#define MAX_LINEAR_VELOCITY_MPS     1.0     ///< Maximum linear velocity (m/s)
#define MAX_ANGULAR_VELOCITY_RPS    2.0     ///< Maximum angular velocity (rad/s)
#define MAX_LINEAR_ACCELERATION     2.0     ///< Maximum linear acceleration (m/s²)
#define MAX_ANGULAR_ACCELERATION    4.0     ///< Maximum angular acceleration (rad/s²)

/**
 * @brief PID Controller Parameters
 */
#define PID_KP                  1.0     ///< Proportional gain
#define PID_KI                  0.1     ///< Integral gain
#define PID_KD                  0.05    ///< Derivative gain
#define PID_OUTPUT_LIMIT        255     ///< PID output limit (matches PWM range)
#define PID_INTEGRAL_LIMIT      100     ///< Integral windup limit

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

#endif // CONFIG_H