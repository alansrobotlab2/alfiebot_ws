/**
 * @file driverboard.h
 * @brief DriverBoard: all head-module state and hardware handles.
 *
 * Owns the serial-bus-servo controller, the eye-LED state, and the onboard
 * status LED. A single global instance `DriverBoard b;` is the source of
 * truth shared between Core 0 (servo bus + eyes + status LED) and Core 1
 * (micro-ROS comms). Servo command/feedback flows through `mBuf[]`, mirroring
 * the wavesharegeneraldriverboard design.
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

    // ---- Eye LEDs ---------------------------------------------------------
    volatile uint16_t eye_pwm[NUM_EYES]   = {0, 0};  ///< commanded duty 0..4095
    volatile uint16_t eye_state[NUM_EYES] = {0, 0};  ///< last-applied duty 0..4095

    // ---- Status LED -------------------------------------------------------
    WS2812 statusLED;

    // ---- Inter-core / watchdog -------------------------------------------
    volatile bool new_head_command = false;  ///< set by Core1 callback, cleared by Core0
    volatile uint32_t last_cmd_time = 0;      ///< millis() of last HeadCmd (watchdog)

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
};

// Global instance (defined in main.cpp)
extern DriverBoard b;
