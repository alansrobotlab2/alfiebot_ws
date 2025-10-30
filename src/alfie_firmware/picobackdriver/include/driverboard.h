/**
 * @file driverboard.h
 * @brief DriverBoard class for RP2040 back Drive Robot
 * 
 * This class contains all robot state and control variables
 * exposed as public members for direct access.
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include <ros_interface.h>
#include "../lib/ws2812/ws2812.h"

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief Motor control structure for linear actuator
 */
typedef struct {
    // Command values
    float target_position;      ///< Target actuator position (m)
    float target_velocity;      ///< Target actuator velocity (m/s)
    float max_acceleration;     ///< Maximum allowed acceleration (m/s²)
    
    // Current state
    float current_position;     ///< Current actuator position (m)
    float current_velocity;     ///< Current actuator velocity (m/s)
    float current_acceleration; ///< Current actuator acceleration (m/s²)
    
    // Encoder data
    volatile int32_t encoder_count;    ///< Current encoder count (volatile for ISR access)
    volatile uint32_t last_pulse_time; ///< Last encoder pulse timestamp (microseconds)
    
    // Control outputs
    int16_t pwm_output;         ///< PWM output value (-255 to 255, sign indicates direction)
    
    // PID controller state for velocity control
    float velocity_error_integral; ///< Integral of velocity error
    float velocity_error_previous; ///< Previous velocity error for derivative
    
    // Ramped velocity for acceleration limiting
    float ramped_velocity;      ///< Velocity after applying acceleration limits
    
    // Status flags
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

/**
 * @brief Linear actuator command structure (from ROS)
 */
typedef struct {
    float position;             ///< Target position (m)
    float velocity;             ///< Target velocity (m/s)
    float acceleration;         ///< Maximum acceleration (m/s²)
    uint32_t timestamp;         ///< Command timestamp
} ActuatorCommand_t;

/**
 * @brief Linear actuator state structure (to ROS)
 */
typedef struct {
    uint8_t board_temp;         ///< Board temperature from RP2040 internal sensor (°C)
    bool limit_switch_triggered; ///< True when lower limit switch is triggered (active low)
    float command_position;     ///< Commanded position (m)
    float command_velocity;     ///< Commanded velocity (m/s)
    float command_acceleration; ///< Commanded acceleration (m/s²)
    bool is_moving;             ///< True if actuator is moving
    float current_position;     ///< Current position (m)
    float current_velocity;     ///< Current velocity (m/s)
    float current_acceleration; ///< Current acceleration (m/s²)
    int64_t pulses;             ///< Encoder pulse count
    uint32_t timestamp;         ///< State timestamp
} ActuatorState_t;

// =============================================================================
// DRIVERBOARD CLASS
// =============================================================================

/**
 * @brief DriverBoard class containing all robot state and control variables
 * All variables are public for direct access without getters/setters
 */
class DriverBoard {
public:
    // Single motor state for linear actuator
    MotorState_t motor;
    
    // Robot status
    uint8_t robot_status;
    
    // Encoder interrupt data
    EncoderData_t encoder;
    
    // WS2812B RGB LED
    WS2812 statusLED;
    
    // ROS state machine variables
    RosAgentState_t agent_state;
    uint32_t last_state_time;
    bool ros_entities_created;
    
    // ROS communication state
    bool micro_ros_initialized;
    uint32_t last_command_time;
    
    // Inter-core communication
    volatile ActuatorCommand_t actuator_cmd;
    volatile ActuatorState_t actuator_state;
    volatile bool new_actuator_command;
    volatile bool new_actuator_state;
    
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
     * Processes actuator commands and applies motor control with velocity PID and acceleration limiting
     */
    void updateMotorControl(void);
    
    /**
     * @brief Read encoder value from motor
     * Updates encoder count for position/velocity calculations
     */
    void readEncoder(void);
    
    /**
     * @brief Calculate actuator state from encoder
     * Updates position, velocity, and acceleration estimates
     */
    void updateActuatorState(void);
    
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
     * @brief Update RGB LED based on robot and ROS state
     * Provides visual feedback using WS2812B RGB LED
     */
    void updateRgbLED(void);
    
    /**
     * @brief Emergency stop all motors
     * Immediately stops all motor outputs for safety
     */
    void emergencyStop(void);
    
    /**
     * @brief Setup encoder interrupts for motor
     * Attaches interrupt handlers to encoder pins
     */
    void setupEncoderInterrupts(void);
    
    /**
     * @brief Process encoder interrupt
     * @param pin_state Current state of the encoder pin that triggered interrupt
     * @param is_pin_a True if pin A triggered interrupt, false if pin B
     */
    void processEncoderInterrupt(bool pin_state, bool is_pin_a);
    
    /**
     * @brief Reset encoder counter to zero
     */
    void resetEncoders(void);
};

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance
extern DriverBoard rp;