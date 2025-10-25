/**
 * @file driverboard.h
 * @brief DriverBoard class for RP2040 Mecanum Drive Robot
 * 
 * This class contains all robot state and control variables
 * exposed as public members for direct access.
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#ifndef DRIVERBOARD_H
#define DRIVERBOARD_H

#include <Arduino.h>
#include "config.h"
#include "ros_interface.h"

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief Motor control structure for individual wheel
 */
typedef struct {
    float target_velocity;      ///< Target wheel velocity (m/s)
    float current_velocity;     ///< Current wheel velocity (m/s)
    float current_acceleration; ///< Current wheel acceleration (m/s²)
    volatile int32_t encoder_count;    ///< Current encoder count (volatile for ISR access)
    volatile uint32_t last_pulse_time; ///< Last encoder pulse timestamp (microseconds)
    float pwm_output;           ///< PWM output value (0-255)
    bool fault_detected;        ///< Motor fault flag
    bool is_moving;             ///< True if motor is currently moving
    bool encoder_a_state;       ///< Current state of encoder A pin
    bool encoder_b_state;       ///< Current state of encoder B pin
} MotorState_t;

/**
 * @brief Encoder interrupt data structure
 */
typedef struct {
    volatile int32_t count;         ///< Encoder pulse count
    volatile uint32_t last_time;    ///< Last interrupt time (microseconds)
    volatile bool direction;        ///< Current rotation direction (true=forward, false=reverse)
    uint8_t pin_a;                  ///< Encoder A pin number
    uint8_t pin_b;                  ///< Encoder B pin number
} EncoderData_t;

// =============================================================================
// DRIVERBOARD CLASS
// =============================================================================

/**
 * @brief DriverBoard class containing all robot state and control variables
 * All variables are public for direct access without getters/setters
 */
class DriverBoard {
public:
    // Motor states for all 4 wheels (FL, FR, RL, RR)
    MotorState_t motors[4];
    
    // Robot status
    uint8_t robot_status;
    
    // Encoder interrupt data for all 4 wheels (FL, FR, RL, RR)
    EncoderData_t encoders[4];
    
    // ROS state machine variables
    RosAgentState_t agent_state;
    uint32_t last_state_time;
    bool ros_entities_created;
    
    // ROS communication state
    bool micro_ros_initialized;
    uint32_t last_command_time;
    
    // Inter-core communication
    volatile VelocityCommand_t velocity_cmd;
    volatile Odometry_t odometry;
    volatile bool new_velocity_command;
    volatile bool new_odometry_data;
    
    /**
     * @brief Constructor - initializes all state variables
     */
    DriverBoard();
    
    /**
     * @brief Initialize all hardware peripherals
     * Sets up motors, encoders, and status LED
     */
    void initializePeripherals(void);
    
    /**
     * @brief Main peripheral management loop
     * Should be called regularly from Core 0 loop
     */
    void updatePeripherals(void);
    
    /**
     * @brief Update motor control (PID control, PWM output)
     * Processes velocity commands and applies motor control
     */
    void updateMotorControl(void);
    
    /**
     * @brief Read encoder values from all motors
     * Updates encoder counts for odometry calculations
     */
    void readEncoders(void);
    
    /**
     * @brief Calculate odometry from wheel velocities
     * Updates robot position and velocity estimates
     */
    void updateOdometry(void);
    
    /**
     * @brief Check for motor faults and safety conditions
     * Monitors motor currents, temperatures, and error conditions
     */
    void handleMotorSafety(void);
    
    /**
     * @brief Update status LED based on robot state
     * Provides visual feedback of robot status
     */
    void updateStatusLED(void);
    
    /**
     * @brief Emergency stop all motors
     * Immediately stops all motor outputs for safety
     */
    void emergencyStop(void);
    
    /**
     * @brief Setup encoder interrupts for all motors
     * Attaches interrupt handlers to encoder pins
     */
    void setupEncoderInterrupts(void);
    
    /**
     * @brief Process encoder interrupt for a specific motor
     * @param motor_index Motor index (0=FL, 1=FR, 2=RL, 3=RR)
     * @param pin_state Current state of the encoder pin that triggered interrupt
     * @param is_pin_a True if pin A triggered interrupt, false if pin B
     */
    void processEncoderInterrupt(uint8_t motor_index, bool pin_state, bool is_pin_a);
    
    /**
     * @brief Reset all encoder counters to zero
     */
    void resetEncoders(void);
};

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance
extern DriverBoard rp;

#endif // DRIVERBOARD_H