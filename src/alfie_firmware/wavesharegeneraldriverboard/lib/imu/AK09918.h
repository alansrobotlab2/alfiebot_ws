/*
    AK09918.h
    A library for Grove - IMU 9DOF(ICM20600 + AK09918)

    Copyright (c) 2018 seeed technology inc.
    Website    : www.seeed.cc
    Author     : Jerry Yip
    Create Time: 2018-06
    Version    : 0.1
    Change Log :

    The MIT License (MIT)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/



#pragma once

#include <Arduino.h>
#include <Wire.h>

/***************************************************************
    AK09918 I2C Register
 ***************************************************************/
#define AK09918_I2C_ADDR    0x0c    // I2C address (Can't be changed)
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

/**
 * @brief AK09918 magnetometer operating modes
 * 
 * The AK09918 supports seven operation modes:
 * (1) Power-down mode: AK09918 doesn't measure, minimal power consumption
 * (2) Single measurement mode (NORMAL): measure when getData() is called
 * (3) Continuous measurement mode 1: 10Hz, measure 10 times per second
 * (4) Continuous measurement mode 2: 20Hz, measure 20 times per second
 * (5) Continuous measurement mode 3: 50Hz, measure 50 times per second
 * (6) Continuous measurement mode 4: 100Hz, measure 100 times per second
 * (7) Self-test mode: internal self-test, use selfTest() function to activate
 */
enum AK09918_mode_type_t {
    AK09918_POWER_DOWN = 0x00,        ///< Power-down mode, sensor inactive
    AK09918_NORMAL = 0x01,            ///< Single measurement mode, trigger on read
    AK09918_CONTINUOUS_10HZ = 0x02,   ///< Continuous mode at 10Hz sampling rate
    AK09918_CONTINUOUS_20HZ = 0x04,   ///< Continuous mode at 20Hz sampling rate
    AK09918_CONTINUOUS_50HZ = 0x06,   ///< Continuous mode at 50Hz sampling rate
    AK09918_CONTINUOUS_100HZ = 0x08,  ///< Continuous mode at 100Hz sampling rate
    AK09918_SELF_TEST = 0x10,         ///< Self-test mode (use selfTest() function, not switchMode())
};

/**
 * @brief AK09918 error codes returned by functions
 */
enum AK09918_err_type_t {
    AK09918_ERR_OK = 0,                 ///< Operation successful, no error
    AK09918_ERR_DOR = 1,                ///< Data overrun - data was skipped/not read in time
    AK09918_ERR_NOT_RDY = 2,            ///< Data not ready yet
    AK09918_ERR_TIMEOUT = 3,            ///< Read/write operation timed out
    AK09918_ERR_SELFTEST_FAILED = 4,    ///< Self-test failed, sensor values out of spec
    AK09918_ERR_OVERFLOW = 5,           ///< Sensor overflow, magnetic field too strong (|x|+|y|+|z| >= 4912µT)
    AK09918_ERR_WRITE_FAILED = 6,       ///< I2C write operation failed
    AK09918_ERR_READ_FAILED = 7,        ///< I2C read operation failed
};

/**
 * @brief Structure to store 3-axis sensor data as 16-bit signed integers
 */
typedef struct imu_st_sensor_data_tag
{
  short int s16X;  ///< X-axis sensor reading (signed 16-bit)
  short int s16Y;  ///< Y-axis sensor reading (signed 16-bit)
  short int s16Z;  ///< Z-axis sensor reading (signed 16-bit)
}IMU_ST_SENSOR_DATA;

class AK09918 {
  public:
    /**
     * @brief Default constructor for AK09918 magnetometer
     */
    AK09918();

    /**
     * @brief Initialize the AK09918 magnetometer with specified mode
     * @param mode Operating mode (default: AK09918_NORMAL for single measurement)
     *             - AK09918_POWER_DOWN: Sensor powered down, no measurements
     *             - AK09918_NORMAL: Single measurement mode, manual trigger
     *             - AK09918_CONTINUOUS_10HZ: Continuous measurements at 10Hz
     *             - AK09918_CONTINUOUS_20HZ: Continuous measurements at 20Hz
     *             - AK09918_CONTINUOUS_50HZ: Continuous measurements at 50Hz
     *             - AK09918_CONTINUOUS_100HZ: Continuous measurements at 100Hz
     * @return AK09918_ERR_OK on success, error code on failure
     * @note AK09918_SELF_TEST mode is not supported by this function, use selfTest() instead
     */
    AK09918_err_type_t initialize(AK09918_mode_type_t mode = AK09918_NORMAL);
    
    /**
     * @brief Check if new magnetometer data is ready to read
     * @return AK09918_ERR_OK if data is ready, AK09918_ERR_NOT_RDY if not ready, 
     *         AK09918_ERR_READ_FAILED if I2C read fails
     * @note Only useful in continuous measurement modes (10Hz/20Hz/50Hz/100Hz)
     */
    AK09918_err_type_t isDataReady();
    
    /**
     * @brief Check if any data measurements were skipped/overrun
     * @return AK09918_ERR_DOR if data was skipped, AK09918_ERR_OK if no skip,
     *         AK09918_ERR_READ_FAILED if I2C read fails
     * @note Occurs when new data arrives before previous data was read
     */
    AK09918_err_type_t isDataSkip();
    
    /**
     * @brief Read calibrated magnetic field data in microTesla (µT)
     * @param axis_x Pointer to store X-axis magnetic field in µT
     * @param axis_y Pointer to store Y-axis magnetic field in µT
     * @param axis_z Pointer to store Z-axis magnetic field in µT
     * @return AK09918_ERR_OK on success, AK09918_ERR_OVERFLOW if sensor saturated,
     *         AK09918_ERR_TIMEOUT in normal mode if measurement times out,
     *         AK09918_ERR_READ_FAILED if I2C read fails
     * @note Values are calibrated (raw * 0.15µT/LSB). In NORMAL mode, triggers a measurement.
     */
    AK09918_err_type_t getData(int16_t* axis_x, int16_t* axis_y, int16_t* axis_z);
    
    /**
     * @brief Read raw magnetic field data directly from sensor (LSB units)
     * @param axis_x Pointer to store raw X-axis value (uncalibrated)
     * @param axis_y Pointer to store raw Y-axis value (uncalibrated)
     * @param axis_z Pointer to store raw Z-axis value (uncalibrated)
     * @return AK09918_ERR_OK on success, AK09918_ERR_OVERFLOW if sensor saturated,
     *         AK09918_ERR_TIMEOUT in normal mode if measurement times out,
     *         AK09918_ERR_READ_FAILED if I2C read fails
     * @note Raw values need to be multiplied by 0.15 to get µT. In NORMAL mode, triggers a measurement.
     */
    AK09918_err_type_t getRawData(int16_t* axis_x, int16_t* axis_y, int16_t* axis_z);


    /**
     * @brief Get the current operating mode of the magnetometer
     * @return Current mode (AK09918_POWER_DOWN, AK09918_NORMAL, AK09918_CONTINUOUS_*)
     */
    AK09918_mode_type_t getMode();
    
    /**
     * @brief Switch the magnetometer to a different operating mode
     * @param mode Target operating mode to switch to
     * @return AK09918_ERR_OK on success, AK09918_ERR_WRITE_FAILED on I2C write failure
     * @note Cannot switch to AK09918_SELF_TEST mode using this function, use selfTest() instead
     */
    AK09918_err_type_t switchMode(AK09918_mode_type_t mode);
    
    /**
     * @brief Perform sensor self-test to verify proper operation
     * @return AK09918_ERR_OK if self-test passes, 
     *         AK09918_ERR_SELFTEST_FAILED if test fails (values out of spec),
     *         AK09918_ERR_WRITE_FAILED or AK09918_ERR_READ_FAILED on I2C errors
     * @note Self-test checks if sensor produces expected magnetic field values.
     *       Expected ranges: X[-200,200], Y[-200,200], Z[-1000,-150] in raw units
     */
    AK09918_err_type_t selfTest();
    
    /**
     * @brief Perform a soft reset of the magnetometer
     * @return AK09918_ERR_OK on success, AK09918_ERR_WRITE_FAILED on I2C write failure
     * @note Resets the sensor to default state, requires re-initialization after reset
     */
    AK09918_err_type_t reset();
    
    /**
     * @brief Convert error code to human-readable string description
     * @param err Error code to describe
     * @return String containing error code name and description
     */
    String strError(AK09918_err_type_t err);
    
    /**
     * @brief Read the magnetometer device ID (company and device identification)
     * @return 16-bit device ID (high byte = WIA1/company ID, low byte = WIA2/device ID)
     */
    uint16_t getDeviceID();



  private:
    /**
     * @brief Read the raw mode register value from sensor
     * @return Raw mode register value
     */
    uint8_t _getRawMode();
    
    /**
     * @brief Write a single byte to sensor register via I2C
     * @param addr I2C device address
     * @param reg Register address to write to
     * @param Value Byte value to write
     * @return true on success, false on failure
     */
    bool writeByte(uint8_t addr,uint8_t reg ,uint8_t Value);
    
    /**
     * @brief Read a single byte from sensor register via I2C
     * @param addr I2C device address
     * @param reg Register address to read from
     * @param buf Pointer to buffer to store read byte
     * @return true on success, false on failure
     */
    bool readByte(uint8_t addr,uint8_t reg,uint8_t *buf);
    
    /**
     * @brief Read multiple bytes from sensor registers via I2C
     * @param addr I2C device address
     * @param reg Starting register address to read from
     * @param num Number of bytes to read
     * @param buf Pointer to buffer to store read bytes
     * @return true on success, false on failure
     */
    bool readBytes(uint8_t addr,uint8_t reg,uint8_t num,uint8_t *buf);
    
    uint8_t _addr;                  ///< I2C address of the device
    AK09918_mode_type_t _mode;      ///< Current operating mode
    uint8_t _buffer[16];            ///< Internal buffer for I2C transactions

};

