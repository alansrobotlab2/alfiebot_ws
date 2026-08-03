/**
 * @file servo_control.h
 * @brief Serial-bus-servo control for the head module (Feetech ST/SMS).
 *
 * Ported from wavesharegeneraldriverboard, trimmed to NUM_SERVOS and extended
 * with SI (radian) <-> servo-count conversions. The command/feedback flow uses
 * the shared DriverBoard mirror `b.mBuf[]`:
 *   - updateServoStatus()  : sync-read feedback  -> mBuf
 *   - updateServoActive()  : sync-write pos/speed/accel/torque for enabled servos
 *   - updateServoIdle()    : sync-write torque-off for disabled servos
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include "driverboard.h"

// ---- Bus loop -------------------------------------------------------------

/// Sync-read status (position..current) for all servos into b.mBuf. Pings all
/// servos to resync the bus on any read failure.
void updateServoStatus();

/// Sync-write pos + speed + accel + torque for servos with torqueSwitch == 1.
void updateServoActive();

/// Sync-write torque-off for servos with torqueSwitch == 0.
void updateServoIdle();

/// Ping every servo to interrupt/resync the half-duplex bus after a bad read.
void flushSerialServoLine();

/// Stage torque-off for all servos (applied on next updateServoIdle()).
void disableAllServoTorques();

/// Read EEPROM angle limits into b.min/maxAngleCount and seed each servo's
/// target to its current position (so nothing jumps on enable). Call at init.
void initServoState();

/// Serve a pending register-map read parked by the ROS service on Core 1.
/// Call from the Core 0 servo tick, alongside the sync read/write pair - the
/// bus must only ever be driven from one core.
void serviceMemoryRequest();

// ---- SI (radian) <-> servo-count conversions ------------------------------

/// Radians (-pi..pi about center) -> STS count (0..4095), clamped to the
/// servo's EEPROM angle limits.
int16_t radToCount(uint8_t servoIndex, float rad);

/// STS count (0..4095) -> radians about center.
float   countToRad(int16_t count);

/// Radians/second -> STS speed units (steps/s), magnitude clamped.
uint16_t radPerSecToSpeed(float radPerSec);

/// STS signed speed value (sign-magnitude) -> radians/second.
float    speedToRadPerSec(int16_t rawSpeed);

/// Radians/second^2 -> STS acceleration units (0..254).
uint8_t  radPerSec2ToAccel(float radPerSec2);
