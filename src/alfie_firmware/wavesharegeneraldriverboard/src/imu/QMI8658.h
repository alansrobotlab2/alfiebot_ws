/*
 * @Description: QMI8658
 * @Author: zjw
 * @Date: 2022-10-24
 * @LastEditTime: 2022-10-24
 * @LastEditors: zjw
 */

#pragma once

#include <Arduino.h>
#include "QMI8658reg.h"

/**
 * @brief Structure to store Euler angle orientation
 */
typedef struct
{
	float roll;   ///< Roll angle (rotation around X-axis)
  float pitch;  ///< Pitch angle (rotation around Y-axis)
  float yaw;    ///< Yaw angle (rotation around Z-axis)
} EulerAngles;

/**
 * @brief Structure to store sensor calibration offset errors
 */
typedef struct
{
  float X_Off_Err;  ///< X-axis offset error correction value
  float Y_Off_Err;  ///< Y-axis offset error correction value
  float Z_Off_Err;  ///< Z-axis offset error correction value
}QMI8658_TypeDef_Off;

class QMI8658
{
  uint8_t last_status; ///< Status of last I2C transmission
  
  /**
   * @brief Read a single byte from sensor register via I2C
   * @param reg Register address to read from
   * @return Byte value read from register
   */
  uint8_t read_reg(uint8_t reg);

  /**
   * @brief Write a single byte to sensor register via I2C
   * @param reg Register address to write to
   * @param value Byte value to write
   */
  void write_reg(uint8_t reg,uint8_t value);

public:
  /**
   * @brief Read a 16-bit word (2 bytes) from sensor registers via I2C
   * @param reg Starting register address (reads reg and reg+1)
   * @return 16-bit word value (little endian: low byte first)
   */
  uint16_t readWord_reg(uint8_t reg);
  
  /**
   * @brief Calculate Euler angles from accelerometer and gyroscope data
   * @param pitch Pointer to store calculated pitch angle
   * @param roll Pointer to store calculated roll angle
   * @param yaw Pointer to store calculated yaw angle
   * @param acc Array to store 3-axis accelerometer data [X, Y, Z]
   * @param gyro Array to store 3-axis gyroscope data [X, Y, Z]
   * @return true on success, false on failure
   * @note Currently reads sensor data but angle calculation is commented out
   */
  bool GetEulerAngles(float *pitch,float *roll, float *yaw,float acc[3],float gyro[3]);

  /**
   * @brief Configure accelerometer settings
   * @param range Measurement range (±2g, ±4g, ±8g, ±16g)
   * @param odr Output data rate (sample frequency in Hz)
   * @param lpfEnable Low-pass filter configuration (enable/disable)
   * @param stEnable Self-test configuration (enable/disable)
   */
  void config_acc(enum qmi8658_AccRange range, enum qmi8658_AccOdr odr,
                      enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable);
  
  /**
   * @brief Configure gyroscope settings
   * @param range Measurement range (±16dps, ±32dps, ±64dps, ±128dps, ±256dps, ±512dps, ±1024dps, ±2048dps)
   * @param odr Output data rate (sample frequency in Hz)
   * @param lpfEnable Low-pass filter configuration (enable/disable)
   * @param stEnable Self-test configuration (enable/disable)
   */
  void config_gyro(enum qmi8658_GyrRange range, enum qmi8658_GyrOdr odr,
                      enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable);

  /**
   * @brief Read both accelerometer and gyroscope data simultaneously
   * @param acc Array to store 3-axis accelerometer data [X, Y, Z] in m/s² (or mg if QMI8658_UINT_MG_DPS defined)
   * @param gyro Array to store 3-axis gyroscope data [X, Y, Z] in rad/s (or dps if QMI8658_UINT_MG_DPS defined)
   * @note Default units are SI: m/s² for acceleration, rad/s for angular velocity
   */
  void read_sensor_data(float acc[3], float gyro[3]);
  
  /**
   * @brief Read accelerometer data only
   * @param acc Array to store 3-axis accelerometer data [X, Y, Z] in m/s² (or mg if QMI8658_UINT_MG_DPS defined)
   */
  void read_acc(float acc[3]);
  
  /**
   * @brief Read gyroscope data only
   * @param gyro Array to store 3-axis gyroscope data [X, Y, Z] in rad/s (or dps if QMI8658_UINT_MG_DPS defined)
   */
  void read_gyro(float gyro[3]);
  
  /**
   * @brief Read both accelerometer and gyroscope data (alias for read_sensor_data)
   * @param acc Array to store 3-axis accelerometer data [X, Y, Z]
   * @param gyro Array to store 3-axis gyroscope data [X, Y, Z]
   */
  void read_xyz(float acc[3], float gyro[3]);
  
  /**
   * @brief Convert sensor axis orientation based on physical mounting layout
   * @param data_a Accelerometer data array [X, Y, Z] to be transformed
   * @param data_g Gyroscope data array [X, Y, Z] to be transformed
   * @param layout Layout code specifying physical orientation (0-7)
   * @note Applies axis remapping and sign changes based on sensor mounting
   */
  void axis_convert(float data_a[3], float data_g[3], int layout);
  
  /**
   * @brief Configure control registers for power management
   * @param low_power Low power mode setting (0=normal, non-zero=low power)
   */
  void config_reg(unsigned char low_power);
  
  /**
   * @brief Enable or disable specific sensors (accelerometer, gyroscope)
   * @param enableFlags Bit flags: bit 0=accelerometer, bit 1=gyroscope
   *                    0x0=all disabled, 0x1=acc only, 0x2=gyro only, 0x3=both enabled
   */
  void enableSensors(unsigned char enableFlags);
  
  /**
   * @brief Read the device ID (WHO_AM_I register)
   * @return Device ID byte (should be 0x05 for QMI8658)
   */
  unsigned char get_id(void);
  
  /**
   * @brief Initialize the QMI8658 IMU sensor with default configuration
   * @return Status byte (0 typically indicates success)
   * @note Sets up accelerometer and gyroscope with default ranges and ODR
   */
  unsigned char begin(void);
  
  /**
   * @brief Dump all sensor registers for debugging purposes
   * @note Prints register addresses and values via Serial output
   */
  void dump_reg(void);
  
  /**
   * @brief Perform on-demand calibration of the sensor
   * @note Calibrates sensor offsets, typically performed when sensor is stationary
   */
  void qmi8658_on_demand_cali(void);
  
  /**
   * @brief Automatically calculate and apply sensor offsets/calibration
   * @note Samples sensor data multiple times to determine offset corrections
   */
  void autoOffsets(void);

public:
  int16_t ax, ay, az, gx, gy, gz;    ///< Raw sensor values (accelerometer and gyroscope)
  float pith, roll, yaw;              ///< Calculated Euler angles
  unsigned long now, lastTime = 0;    ///< Timing variables for calculations
  float dt;                           ///< Differential time (微分时间) - time delta between samples
  float agz = 0;                      ///< Angle variable (角度变量) - accumulated angle
  long gzo = 0;                       ///< Gyroscope offset (陀螺仪偏移量)
  QMI8658_TypeDef_Off TempAcc = {0};  ///< Accelerometer calibration offsets (校准值)
  QMI8658_TypeDef_Off TempGyr = {0};  ///< Gyroscope calibration offsets
};

/*----------------------------------------------------------------------------------------------
  QMI8658C UI Sensor Configuration Settings and Output Data
*/
///<Configuration Registers>
#define QMI8658_ADDR 0X6B  //device address
#define WHO_AM_I 0X00 //Device identifier
#define CTRL1 0x02    //Serial Interface and Sensor Enable
#define CTRL2 0x03    //Accelerometer Settings
#define CTRL3 0x04    //Gyroscope Settings
#define CTRL4 0X05    //Magnetometer Settings
#define CTRL5 0X06    //Sensor Data Processing Settings
#define CTRL7 0x08    //Enable Sensors and Configure Data Reads
#define CTRL8 0X09    //Reserved – Special Settings

///<Sensor Data Output Registers>
#define AccX_L 0x35
#define AccX_H 0x36
#define AccY_L 0x37
#define AccY_H 0x38
#define AccZ_L 0x39
#define AccZ_H 0x3A
#define TEMP_L 0x33

#define GyrX_L 0x3B
#define GyrX_H 0x3C
#define GyrY_L 0x3D
#define GyrY_H 0x3E
#define GyrZ_L 0x3F
#define GyrZ_H 0x40
// int16_t QMI8658C_readBytes(unsigned char tmp);
//extern QMI8658C _QMI8658C;
