/**
 * @file hiwonder_driver.h
 * @brief I2C driver for the Hiwonder 4-channel encoder motor controller
 *
 * Thin register-access wrapper around the Arduino Wire library. The Hiwonder
 * controller has its own MCU that runs a closed-loop velocity PID per channel,
 * so this driver only writes target speeds (pulses/10ms) and reads back
 * accumulated encoder counts. Motor/wheel index order is [FL, FR, RL, RR];
 * the physical-channel remap and direction signs live here (see .cpp) so all
 * wiring-order calibration is in one place.
 *
 * @author Alfie Bot Project
 */

#ifndef HIWONDER_DRIVER_H
#define HIWONDER_DRIVER_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Driver for the Hiwonder 4-channel encoder motor controller over I2C.
 */
class Hiwonder {
public:
    Hiwonder(uint8_t address = HIWONDER_I2C_ADDR);

    /**
     * @brief Initialise the I2C bus and configure the controller.
     * Sets motor type and encoder polarity, then stops all motors.
     * @return true if the initial configuration writes were acknowledged.
     */
    bool begin(void);

    /**
     * @brief Command closed-loop target speeds for the four wheels.
     * @param speed_pulses Per-wheel speed in pulses/10ms, order [FL, FR, RL, RR].
     *                     Values are clamped to +/-HIWONDER_MAX_SPEED.
     */
    void setSpeeds(const int8_t speed_pulses[4]);

    /**
     * @brief Read the accumulated encoder counts for the four wheels.
     * @param counts Output array, order [FL, FR, RL, RR].
     */
    void readEncoders(int32_t counts[4]);

    /**
     * @brief Stop all four motors (zero closed-loop speed).
     */
    void stop(void);

    /**
     * @brief Read the controller's measured battery voltage.
     * @return Battery voltage in millivolts (0 if the read fails).
     */
    uint16_t readBatteryMv(void);

    /**
     * @brief Status of the last I2C transaction (Wire.endTransmission code).
     * @return 0 on success.
     */
    uint8_t lastStatus(void) const { return last_status_; }

private:
    void writeRegister(uint8_t reg, const uint8_t *data, uint8_t len);
    bool readRegister(uint8_t reg, uint8_t *data, uint8_t len);

    uint8_t address_;
    uint8_t last_status_;
};

#endif // HIWONDER_DRIVER_H
