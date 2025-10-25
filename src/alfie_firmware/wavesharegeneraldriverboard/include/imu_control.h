#pragma once

#include "config.h"

/**
 * @brief Initializes the IMU sensor hardware
 * 
 * Sets up the IMU (Inertial Measurement Unit) sensor for operation.
 * Configuration details are defined in the IMU library implementation.
 * 
 * @note Should be called once during system initialization
 * @warning Function declaration exists but implementation not found in current codebase
 */
void initIMU();


/**
 * @brief Reads and updates IMU sensor data including orientation, acceleration, gyroscope, and magnetometer
 * 
 * Retrieves current IMU measurements and updates the global DriverBoard structure with:
 * - Roll, pitch, and yaw angles (orientation)
 * - Raw gyroscope data (angular velocity)
 * - Raw accelerometer data (linear acceleration)
 * - Raw magnetometer data (magnetic field in Tesla)
 * - Board temperature from IMU sensor
 * 
 * All magnetic field data is already converted to Tesla units by the underlying
 * imuDataGet() function, so no additional conversion is needed.
 * 
 * @note Called periodically from the main control loop to maintain fresh sensor data
 * @note Updates global DriverBoard b structure members: stAngles, stGyroRawData, stAccelRawData, stMagnRawData, board_temp, IMU_Roll, IMU_Pitch, IMU_Yaw
 * @see imuDataGet() for low-level sensor reading
 * @see QMI8658_readTemp() for temperature reading
 */
void getIMUData();
