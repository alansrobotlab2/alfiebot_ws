/**
 * @file driverboard.h
 * @brief DriverBoard: all arm-driver state and hardware handles.
 *
 * Owns the serial-bus-servo controller and the onboard status LED. A single
 * global instance `DriverBoard b;` is the source of truth shared between Core 0
 * (servo bus + status LED) and Core 1 (micro-ROS comms). Servo command/feedback
 * flows through `mBuf[]`, mirroring the head/back driver design.
 *
 * The arm firmware is identical on both arms; `arm_side`, `polarity`, and
 * `namespace_str` are filled in at boot by armSelectSide() (lib/arm_control)
 * from the Pico unique board serial.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include "config.h"
#include "memorystruct.h"
#include "SCServo.h"
#include "../lib/ws2812/ws2812.h"

class DriverBoard {
public:
    // ---- Serial bus servos ------------------------------------------------
    SMS_STS st;                              ///< ST/SMS bus controller (bind pSerial = &Serial1)

    MemoryReplyBuf mBuf[NUM_SERVOS] = {};    ///< per-servo register mirror (cmd + feedback)
    MemoryReplyBuf buf = {};                 ///< scratch buffer for sync reads

    uint8_t IDS[NUM_SERVOS];                 ///< bus IDs, 1-based (filled in ctor)
    uint8_t servoCMDIDS[NUM_SERVOS];         ///< scratch ID list for sync writes

    uint8_t torquecommandbuf[NUM_SERVOS] = {};                       ///< torque-off sync-write payload
    uint8_t servocommandbuf[SERVO_CMD_PACKET_SIZE * NUM_SERVOS] = {}; ///< active sync-write payload

    // Per-servo angle limits (counts) read from EEPROM at init; used to clamp targets.
    int16_t minAngleCount[NUM_SERVOS];
    int16_t maxAngleCount[NUM_SERVOS];

    // ---- Board identity (left vs right) -----------------------------------
    volatile uint8_t arm_side = ARM_SIDE_UNKNOWN;   ///< 0=left, 1=right, 255=unknown
    const int8_t *polarity   = nullptr;             ///< per-servo sign (length NUM_SERVOS)
    const char   *namespace_str = ARM_NS_UNKNOWN;   ///< ROS namespace for this side
    volatile uint8_t board_serial[8] = {};          ///< raw Pico unique id (flash RUID)
    char board_serial_str[2 * 8 + 1] = {0};         ///< 16-char hex id + NUL

    // ---- Status LED -------------------------------------------------------
    WS2812 statusLED;

    // ---- Inter-core / watchdog -------------------------------------------
    volatile bool new_arm_command = false;    ///< set by Core1 callback
    volatile uint32_t last_cmd_time = 0;       ///< millis() of last ArmCmd (watchdog)

    DriverBoard()
        : statusLED(WS2812B_PIN, 800000)
    {
        for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
            IDS[i]         = i + 1;
            servoCMDIDS[i] = i + 1;
            minAngleCount[i] = SERVO_MIN_COUNT;
            maxAngleCount[i] = SERVO_MAX_COUNT;
        }
    }

    /// ROS namespace string (set by armSelectSide(); defaults to unknown_arm).
    const char *getNamespace() const { return namespace_str; }
};

// Global instance (defined in main.cpp)
extern DriverBoard b;
