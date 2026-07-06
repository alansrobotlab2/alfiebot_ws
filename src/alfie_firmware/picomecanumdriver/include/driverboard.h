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
#include "hiwonder_driver.h"

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
    int32_t encoder_count;      ///< Latest accumulated encoder count from the Hiwonder controller
    int16_t speed_cmd;          ///< Last commanded speed sent to the controller (pulses/10ms, signed)
    bool fault_detected;        ///< Motor fault flag
    bool is_moving;             ///< True if motor is currently moving
} MotorState_t;

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

    // Hiwonder 4-channel encoder motor controller (I2C)
    Hiwonder hw;

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
     * @brief Reset the cached encoder baseline to the current controller counts
     */
    void resetEncoders(void);
};

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance
extern DriverBoard rp;

#endif // DRIVERBOARD_H