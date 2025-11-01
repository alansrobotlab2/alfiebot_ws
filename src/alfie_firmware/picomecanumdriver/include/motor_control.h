/**
 * @file motor_control.h
 * @brief Motor control and peripheral management for mecanum drive robot
 * 
 * This module handles all peripheral management including:
 * - Motor control and PWM output
 * - Encoder reading and processing
 * - Odometry calculations
 * - Safety monitoring
 * - Status LED management
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include "config.h"
#include "ros_interface.h"
#include "driverboard.h"

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance (declared in driverboard.h)
// extern DriverBoard rp;

// =============================================================================
// ENCODER INTERRUPT SERVICE ROUTINES
// =============================================================================

/**
 * @brief Encoder interrupt service routines
 * Handle encoder pulse detection and counting
 */
void encoderISR_FL_A(void);
void encoderISR_FL_B(void);
void encoderISR_FR_A(void);
void encoderISR_FR_B(void);
void encoderISR_RL_A(void);
void encoderISR_RL_B(void);
void encoderISR_RR_A(void);
void encoderISR_RR_B(void);

/**
 * @brief Reset all encoder counters to zero
 */
void resetEncoders(void);

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
                           float wheel_velocities[4]);

/**
 * @brief Convert wheel velocities to robot velocities (forward kinematics)
 * 
 * @param wheel_velocities Input array of wheel velocities [FL, FR, RL, RR]
 * @param linear_x Output forward/backward velocity (m/s)
 * @param linear_y Output left/right strafe velocity (m/s)
 * @param angular_z Output rotational velocity (rad/s)
 */
void mecanumDriveOdometry(float wheel_velocities[4], float *linear_x, 
                         float *linear_y, float *angular_z);

#endif // MOTOR_CONTROL_H