/**
 * @file motor_control.h
 * @brief Motor control and peripheral management for linear actuator
 * 
 * This module handles all peripheral management including:
 * - Motor control and PWM output
 * - Encoder reading and processing
 * - Position and velocity calculations
 * - PID velocity control with acceleration limiting
 * - Safety monitoring
 * - Status LED management
 * 
 * @author Alfie Bot Project
 * @date 2025-10-27
 */

#pragma once

#include <Arduino.h>
#include "../../include/config.h"
#include <ros_interface.h>
#include "../../include/driverboard.h"

// =============================================================================
// ENCODER INTERRUPT SERVICE ROUTINES
// =============================================================================

/**
 * @brief Encoder interrupt service routines
 * Handle encoder pulse detection and counting
 */
void encoderISR_A(void);
void encoderISR_B(void);

/**
 * @brief Reset encoder counter to zero
 */
void resetEncoders(void);

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
int16_t applyVelocityPID(float target_velocity, float current_velocity, float dt);

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
                             float max_acceleration, float dt);