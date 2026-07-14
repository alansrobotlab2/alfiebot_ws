/**
 * @file hiwonder_driver.cpp
 * @brief I2C driver implementation for the Hiwonder 4-channel encoder motor controller
 *
 * @author Alfie Bot Project
 */

#include "hiwonder_driver.h"
#include <Wire.h>

// =============================================================================
// WIRING / DIRECTION CALIBRATION
// =============================================================================
//
// The firmware works in logical wheel order [FL, FR, RL, RR]. These two tables
// map that logical order onto the physical Hiwonder motor channels and fix each
// wheel's spin direction. Adjust these (and HIWONDER_ENCODER_POLARITY in
// config.h) during bring-up so that:
//   - a positive FL command spins the FL wheel forward, and
//   - the FL encoder count increases when the FL wheel rolls forward.
//
// MOTOR_CHANNEL_MAP[logical] = Hiwonder channel index (0..3) that wheel is wired to.
// MOTOR_DIR_SIGN[logical]    = +1 or -1 applied to both speed command and count.
static const uint8_t MOTOR_CHANNEL_MAP[4] = {1, 3, 0, 2}; // FL, FR, RL, RR
static const int8_t  MOTOR_DIR_SIGN[4]    = {1, 1, -1, 1}; // FL, FR, RL, RR
//
// Calibration determined on hardware 2026-07-13 (open-loop spin test on the
// mecanumtest rig, same hardware):
//   - Channel map: commanding logical FL/FR/RL/RR with an identity map physically
//     spun RL/FL/RR/FR, i.e. Hiwonder ch0->RL, ch1->FL, ch2->RR, ch3->FR, giving
//     FL->ch1, FR->ch3, RL->ch0, RR->ch2.
//   - Direction: FL/FR/RR roll forward on +command; RL rolled backward, so RL=-1.
//   - FR's encoder was reversed relative to its motor (closed-loop PID runaway);
//     FIXED IN HARDWARE by swapping the FR encoder A/B wires. All four now count
//     up on forward, so closed-loop control is stable.

Hiwonder::Hiwonder(uint8_t address)
    : address_(address), last_status_(0) {}

// =============================================================================
// LOW-LEVEL REGISTER ACCESS
// =============================================================================

void Hiwonder::writeRegister(uint8_t reg, const uint8_t *data, uint8_t len) {
    HIWONDER_WIRE.beginTransmission(address_);
    HIWONDER_WIRE.write(reg);
    for (uint8_t i = 0; i < len; i++) {
        HIWONDER_WIRE.write(data[i]);
    }
    last_status_ = HIWONDER_WIRE.endTransmission();
}

bool Hiwonder::readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
    HIWONDER_WIRE.beginTransmission(address_);
    HIWONDER_WIRE.write(reg);
    last_status_ = HIWONDER_WIRE.endTransmission();
    if (last_status_ != 0) {
        return false;
    }

    uint8_t received = HIWONDER_WIRE.requestFrom(address_, len);
    if (received != len) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        data[i] = HIWONDER_WIRE.read();
    }
    return true;
}

// =============================================================================
// PUBLIC API
// =============================================================================

bool Hiwonder::begin(void) {
    HIWONDER_WIRE.setSDA(I2C_SDA_PIN);
    HIWONDER_WIRE.setSCL(I2C_SCL_PIN);
    HIWONDER_WIRE.begin();
    HIWONDER_WIRE.setClock(I2C_FREQ_HZ);

    // Configure motor type, then encoder polarity.
    uint8_t motor_type = HIWONDER_MOTOR_TYPE;
    writeRegister(HIWONDER_REG_MOTOR_TYPE, &motor_type, 1);
    bool ok = (last_status_ == 0);

    uint8_t polarity = HIWONDER_ENCODER_POLARITY;
    writeRegister(HIWONDER_REG_ENCODER_POLARITY, &polarity, 1);
    ok = ok && (last_status_ == 0);

    stop();
    return ok;
}

void Hiwonder::setSpeeds(const int8_t speed_pulses[4]) {
    int8_t out[4];
    for (uint8_t logical = 0; logical < 4; logical++) {
        int16_t v = (int16_t)speed_pulses[logical] * MOTOR_DIR_SIGN[logical];
        if (v > HIWONDER_MAX_SPEED)  v = HIWONDER_MAX_SPEED;
        if (v < -HIWONDER_MAX_SPEED) v = -HIWONDER_MAX_SPEED;
        out[MOTOR_CHANNEL_MAP[logical]] = (int8_t)v;
    }
    writeRegister(HIWONDER_REG_FIXED_SPEED, (const uint8_t *)out, 4);
}

void Hiwonder::readEncoders(int32_t counts[4]) {
    uint8_t buf[16];
    if (!readRegister(HIWONDER_REG_ENCODER_TOTAL, buf, 16)) {
        return; // leave previous values on failure
    }
    for (uint8_t logical = 0; logical < 4; logical++) {
        uint8_t ch = MOTOR_CHANNEL_MAP[logical];
        uint8_t *p = &buf[ch * 4];
        int32_t raw = (int32_t)((uint32_t)p[0] |
                                ((uint32_t)p[1] << 8) |
                                ((uint32_t)p[2] << 16) |
                                ((uint32_t)p[3] << 24));
        counts[logical] = raw * MOTOR_DIR_SIGN[logical];
    }
}

void Hiwonder::stop(void) {
    const uint8_t zeros[4] = {0, 0, 0, 0};
    writeRegister(HIWONDER_REG_FIXED_SPEED, zeros, 4);
}

uint16_t Hiwonder::readBatteryMv(void) {
    uint8_t buf[2];
    if (!readRegister(HIWONDER_REG_ADC_BAT, buf, 2)) {
        return 0;
    }
    return (uint16_t)(buf[0] | (buf[1] << 8));
}
