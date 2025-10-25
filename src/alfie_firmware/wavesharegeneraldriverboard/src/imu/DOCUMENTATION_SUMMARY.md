# IMU Library Documentation Summary

This document summarizes the comprehensive documentation added to all functions in the imu directory.

## Files Updated

### 1. **AK09918.h** - Magnetometer Sensor Driver
Complete Doxygen-style documentation added for:
- **1 constructor** - AK09918()
- **13 public functions**:
  - `initialize()` - Initialize magnetometer with operating mode
  - `isDataReady()` - Check if new data is available
  - `isDataSkip()` - Check for data overrun
  - `getData()` - Read calibrated magnetic field in µT
  - `getRawData()` - Read raw magnetometer values
  - `getMode()` - Get current operating mode
  - `switchMode()` - Change operating mode
  - `selfTest()` - Perform self-test diagnostics
  - `reset()` - Soft reset the sensor
  - `strError()` - Convert error codes to strings
  - `getDeviceID()` - Read device identification
- **4 private helper functions** - I2C read/write operations
- **2 enumerations**: 
  - `AK09918_mode_type_t` - 7 operating modes (power-down, normal, continuous 10/20/50/100Hz, self-test)
  - `AK09918_err_type_t` - 8 error codes
- **1 struct**: `IMU_ST_SENSOR_DATA` - 3-axis signed 16-bit sensor data

### 2. **QMI8658.h** - 6-Axis IMU Sensor Driver
Complete Doxygen-style documentation added for:
- **21 public functions**:
  - `begin()` - Initialize IMU with default config
  - `get_id()` - Read device WHO_AM_I
  - `config_acc()` - Configure accelerometer range/ODR/filters
  - `config_gyro()` - Configure gyroscope range/ODR/filters
  - `read_sensor_data()` - Read both accel and gyro
  - `read_acc()` - Read accelerometer only
  - `read_gyro()` - Read gyroscope only
  - `read_xyz()` - Alias for read_sensor_data
  - `GetEulerAngles()` - Calculate orientation angles
  - `axis_convert()` - Remap axes for different mounting
  - `config_reg()` - Power management configuration
  - `enableSensors()` - Enable/disable accel and gyro
  - `dump_reg()` - Debug register dump
  - `qmi8658_on_demand_cali()` - On-demand calibration
  - `autoOffsets()` - Auto-calculate offsets
  - `readWord_reg()` - Read 16-bit register value
- **3 private I2C functions** - Low-level read/write operations
- **2 structures**:
  - `EulerAngles` - Roll, pitch, yaw orientation
  - `QMI8658_TypeDef_Off` - Calibration offset errors
- **9 public member variables** - Raw values, angles, timing, calibration data

### 3. **IMU.h** - High-Level IMU Interface
Complete Doxygen-style documentation added for:
- **4 main functions**:
  - `imuInit()` - Initialize entire IMU system (QMI8658 + AK09918)
  - `imuDataGet()` - Read all sensors with AHRS fusion
  - `imuAHRSupdate()` - Mahony AHRS filter algorithm
  - `invSqrt()` - Fast inverse square root
  - `QMI8658_readTemp()` - Read temperature sensor
- **2 data structures**:
  - `IMU_ST_ANGLES_DATA` - Yaw, pitch, roll angles
  - `IMU_ST_SENSOR_DATA_FLOAT` - 3-axis float data
- **AHRS Integration**: Documents 9-DOF sensor fusion with magnetometer compensation

### 4. **QMI8658reg.h** - Register Definitions and Enumerations
Enhanced documentation for:
- **10+ enumerations**:
  - `qmi8658_LpfConfig` - Low-pass filter enable/disable
  - `qmi8658_HpfConfig` - High-pass filter enable/disable
  - `qmi8658_StConfig` - Self-test enable/disable
  - `qmi8658_LpfMode` - Filter modes for accel/gyro
  - `qmi8658_AccRange` - ±2g, ±4g, ±8g, ±16g
  - `qmi8658_AccOdr` - 13 ODR options from 3Hz to 8000Hz
  - `qmi8658_GyrRange` - ±16dps to ±2048dps (8 options)
  - `qmi8658_GyrOdr` - 9 ODR options from 31.25Hz to 8000Hz
  - `qmi8658_AccUnit` - g or m/s²
  - `qmi8658_GyrUnit` - dps or rad/s
- **3 configuration structures**:
  - `qmi8658_cali` - Calibration data with 24 fields
  - `qmi8658_config` - Sensor configuration
  - `qmi8658_state` - Complete sensor state

## Documentation Style

All function documentation follows **Doxygen format** which provides:
- ✅ **IntelliSense/hover tooltips** in VS Code
- ✅ **Parameter descriptions** with types, units, and ranges
- ✅ **Return value explanations** with error codes
- ✅ **Usage notes** and important considerations
- ✅ **Units documentation** (µT, m/s², rad/s, degrees, etc.)
- ✅ **Enum value descriptions** for all configurations

## Key Features Documented

### AK09918 Magnetometer
- **7 operating modes**: Power-down, single measurement, continuous at 10/20/50/100Hz, self-test
- **I2C address**: 0x0C (fixed, cannot be changed)
- **Sensitivity**: 0.15 µT/LSB
- **Self-test ranges**: X[-200,200], Y[-200,200], Z[-1000,-150]
- **Overflow detection**: |x|+|y|+|z| >= 4912µT

### QMI8658 6-Axis IMU
- **Accelerometer ranges**: ±2g, ±4g, ±8g, ±16g
- **Gyroscope ranges**: ±16dps to ±2048dps (8 options)
- **Output data rates**: 3Hz to 8000Hz (many options)
- **Units**: SI units (m/s², rad/s) or traditional (mg, dps)
- **Filters**: Low-pass and high-pass configurable
- **I2C address**: 0x6B
- **Device ID**: 0x05

### IMU Fusion System
- **9-DOF AHRS**: Mahony filter algorithm
- **Sensors**: QMI8658 (accel + gyro) + AK09918 (magnetometer)
- **Gains**: Kp=4.50 (proportional), Ki=1.0 (integral)
- **Sample period**: 24ms (≈41.7Hz)
- **Output**: Quaternion (q0, q1, q2, q3) → Euler angles (roll, pitch, yaw)
- **I2C configuration**: SDA=GPIO32, SCL=GPIO33, 400kHz

## Units Reference

| Sensor | Raw | Calibrated | SI Unit |
|--------|-----|------------|---------|
| Magnetometer | LSB | µT | Tesla (T) = µT × 10⁻⁶ |
| Accelerometer | LSB | mg or m/s² | m/s² |
| Gyroscope | LSB | dps or rad/s | rad/s |
| Temperature | LSB | °C | °C |
| Angles | - | degrees | degrees |

## Configuration Examples

### Magnetometer Initialization
```cpp
AK09918 mag;
mag.initialize(AK09918_CONTINUOUS_100HZ);  // 100Hz continuous mode
```

### IMU Configuration
```cpp
QMI8658 imu;
imu.begin();  // Default configuration
imu.config_acc(Qmi8658AccRange_8g, Qmi8658AccOdr_1000Hz, 
               Qmi8658Lpf_Enable, Qmi8658St_Disable);
imu.config_gyro(Qmi8658GyrRange_512dps, Qmi8658GyrOdr_1000Hz,
                Qmi8658Lpf_Enable, Qmi8658St_Disable);
```

### Complete System
```cpp
imuInit();  // Initialize QMI8658 + AK09918 + AHRS

EulerAngles angles;
IMU_ST_SENSOR_DATA_FLOAT gyro, accel, magn;
imuDataGet(&angles, &gyro, &accel, &magn);
// angles contains fused orientation (roll, pitch, yaw)
```

## Error Handling

All functions return appropriate error codes:
- **AK09918**: Returns `AK09918_err_type_t` enum
- **QMI8658**: Returns status bytes or data directly
- **IMU**: void functions, data returned via pointers

## Benefits

1. **Complete API Understanding**: Every function's purpose and usage is clear
2. **Unit Clarity**: All physical units are documented (µT, m/s², rad/s, °C, etc.)
3. **Configuration Guidance**: Range and ODR options fully explained
4. **Error Handling**: Error codes and their meanings documented
5. **IDE Integration**: Full IntelliSense support in VS Code
6. **Sensor Fusion**: AHRS algorithm parameters and operation documented

---

**Total Items Documented**: 
- 50+ functions across 4 header files
- 15+ enumerations with full value descriptions
- 8+ structures with member documentation
- 100+ register and constant definitions

**Documentation Standard**: Doxygen-compatible  
**Language**: C++  
**Sensor Hardware**: 
- QMI8658 - 6-axis IMU (accel + gyro)
- AK09918 - 3-axis magnetometer
- Combined: 9-DOF IMU system with AHRS

**Date Completed**: October 13, 2025
