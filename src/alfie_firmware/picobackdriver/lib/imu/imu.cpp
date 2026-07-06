/**
 * @file imu.cpp
 * @brief BNO085 IMU driver wrapper implementation (SparkFun BNO08x library)
 *
 * @author Alfie Bot Project
 */

#include "imu.h"
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "../../include/config.h"

// BNO085 sensor instance (owns the SHTP protocol state)
static BNO08x myIMU;

// Latched init state; when false, imuUpdate() is a no-op
static bool imu_ready = false;

/**
 * @brief (Re)enable the desired BNO085 sensor reports
 */
static void enableReports(void) {
    myIMU.enableRotationVector(IMU_REPORT_INTERVAL_MS);  // fused quaternion
    myIMU.enableGyro(IMU_REPORT_INTERVAL_MS);            // calibrated gyro (rad/s)
    myIMU.enableAccelerometer(IMU_REPORT_INTERVAL_MS);   // accel incl. gravity (m/s^2)
}

bool imuInit(void) {
    // Configure I2C0 pins BEFORE Wire.begin() (required by earlephilhower RP2040 core)
    Wire.setSDA(IMU_I2C_SDA_PIN);
    Wire.setSCL(IMU_I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    // begin() owns the reset line (GP11); INT is not wired, so we poll.
    if (!myIMU.begin(IMU_I2C_ADDR, Wire, -1, IMU_RESET_PIN)) {
        imu_ready = false;
        return false;
    }

    enableReports();
    imu_ready = true;
    return true;
}

bool imuUpdate(volatile ImuData_t &out) {
    if (!imu_ready) {
        return false;
    }

    bool updated = false;

    // Drain all events queued since the last call. If the sensor was reset
    // (e.g. brownout), re-enable the reports so telemetry recovers.
    while (myIMU.getSensorEvent()) {
        if (myIMU.wasReset()) {
            enableReports();
        }

        switch (myIMU.getSensorEventID()) {
            case SENSOR_REPORTID_ROTATION_VECTOR:
                out.qw = myIMU.getQuatReal();
                out.qx = myIMU.getQuatI();
                out.qy = myIMU.getQuatJ();
                out.qz = myIMU.getQuatK();
                out.valid = true;
                updated = true;
                break;

            case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
                out.gyro_x = myIMU.getGyroX();
                out.gyro_y = myIMU.getGyroY();
                out.gyro_z = myIMU.getGyroZ();
                updated = true;
                break;

            case SENSOR_REPORTID_ACCELEROMETER:
                out.accel_x = myIMU.getAccelX();
                out.accel_y = myIMU.getAccelY();
                out.accel_z = myIMU.getAccelZ();
                updated = true;
                break;

            default:
                break;
        }
    }

    return updated;
}
