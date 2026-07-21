/**
 * @file imu.h
 * @brief BNO085 IMU driver wrapper for RP2040 back drive board
 *
 * Thin wrapper around the SparkFun BNO08x Arduino Library. The BNO085 runs
 * on-chip sensor fusion, so this module simply brings up the I2C0 bus, enables
 * the rotation-vector / gyro / accelerometer reports, and drains sensor events
 * into a plain data struct for the ROS publisher to copy.
 *
 * Bus: I2C0 (SDA=GP12, SCL=GP13), RESET=GP11 (see config.h).
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>

/**
 * @brief IMU telemetry snapshot (SI units, ROS conventions)
 *
 * Orientation is a true unit quaternion from the BNO085 rotation vector.
 * Angular velocity is calibrated gyro in rad/s. Linear acceleration is the
 * raw accelerometer in m/s^2 with gravity included (matches
 * sensor_msgs/Imu.linear_acceleration semantics).
 */
typedef struct {
    float qw, qx, qy, qz;               ///< Orientation quaternion (rotation vector, mag-referenced)
    float game_qw, game_qx, game_qy, game_qz; ///< Compass-free quaternion (game rotation vector)
    float gyro_x, gyro_y, gyro_z;       ///< Angular velocity (rad/s)
    float accel_x, accel_y, accel_z;    ///< Linear acceleration (m/s^2, gravity incl.)
    bool valid;                         ///< True once a rotation-vector report has been received
} ImuData_t;

/**
 * @brief Initialize the I2C0 bus and the BNO085 sensor
 *
 * Configures the I2C0 pins, starts the bus, resets/initializes the BNO085 and
 * enables the rotation-vector, gyro and accelerometer reports.
 *
 * @return true if the sensor initialized successfully, false otherwise
 */
bool imuInit(void);

/**
 * @brief Drain pending BNO085 events into the provided data struct
 *
 * Non-blocking: returns quickly when no new event is available, so it is safe
 * to call from the Core 0 control loop at high frequency.
 *
 * @param out Destination data struct (updated in place)
 * @return true if any field was updated this call
 */
bool imuUpdate(volatile ImuData_t &out);
