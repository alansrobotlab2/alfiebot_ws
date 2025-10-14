#ifndef _IMU_H_
#define _IMU_H_

#include "AK09918.h"
#include "QMI8658.h"
#include <stdio.h>
#include <math.h>

/**
 * @brief Structure to store Euler angle orientation data
 */
typedef struct imu_st_angles_data_tag
{
  float fYaw;    ///< Yaw angle (rotation around Z-axis) in degrees
  float fPitch;  ///< Pitch angle (rotation around Y-axis) in degrees
  float fRoll;   ///< Roll angle (rotation around X-axis) in degrees
}IMU_ST_ANGLES_DATA;

/**
 * @brief Structure to store 3-axis sensor data as floating point values
 */
typedef struct imu_st_sensor_data_float
{
  float X;  ///< X-axis sensor reading
  float Y;  ///< Y-axis sensor reading
  float Z;  ///< Z-axis sensor reading
}IMU_ST_SENSOR_DATA_FLOAT;

/**
 * @brief Initialize the IMU system (QMI8658 + AK09918 magnetometer)
 * @note Initializes I2C communication, configures sensors, and sets up AHRS quaternions
 *       Sets magnetometer to 100Hz continuous mode
 *       I2C pins: SDA=32, SCL=33 at 400kHz
 */
void imuInit();

/**
 * @brief Read and process all IMU sensor data with AHRS fusion
 * @param pstAngles Pointer to EulerAngles structure to store calculated orientation (pitch, roll, yaw) in degrees
 * @param pstGyroRawData Pointer to structure to store gyroscope data [X, Y, Z] in rad/s
 * @param pstAccelRawData Pointer to structure to store accelerometer data [X, Y, Z] in m/s²
 * @param pstMagnRawData Pointer to structure to store magnetometer data [X, Y, Z] in Tesla
 * @note Uses Mahony AHRS algorithm to fuse sensor data into orientation
 *       Magnetometer data is calibrated using offset values
 *       Euler angles are calculated from quaternion representation
 */
void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstMagnRawData); 

/**
 * @brief Update AHRS quaternion using Mahony filter algorithm
 * @param gx Gyroscope X-axis reading in rad/s
 * @param gy Gyroscope Y-axis reading in rad/s
 * @param gz Gyroscope Z-axis reading in rad/s
 * @param ax Accelerometer X-axis reading in m/s²
 * @param ay Accelerometer Y-axis reading in m/s²
 * @param az Accelerometer Z-axis reading in m/s²
 * @param mx Magnetometer X-axis reading in µT
 * @param my Magnetometer Y-axis reading in µT
 * @param mz Magnetometer Z-axis reading in µT
 * @note Fuses 9-DOF sensor data (gyro + accel + mag) to estimate orientation quaternion
 *       Uses proportional-integral feedback for drift correction
 *       Updates global quaternion variables (q0, q1, q2, q3)
 *       Gains: Kp=4.50 (proportional), Ki=1.0 (integral)
 */
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
 * @brief Fast inverse square root calculation for vector normalization
 * @param x Input value
 * @return Approximation of 1/sqrt(x)
 * @note Uses fast approximation algorithm for computational efficiency in AHRS calculations
 */
float invSqrt(float x);

/**
 * @brief Read temperature from QMI8658 sensor
 * @return Temperature reading in degrees Celsius
 * @note Standalone function for reading internal IMU temperature sensor
 */
float QMI8658_readTemp();

#endif
