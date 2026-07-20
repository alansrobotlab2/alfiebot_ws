/**
 * @file eye_control.h
 * @brief Eye LED driver (TB6612FNG) + onboard status/heartbeat LED behaviors.
 *
 * Each eye LED sits on one TB6612FNG channel: STBY held HIGH, INx fixed to one
 * polarity, brightness set by PWM duty at EYE_PWM_FREQ_HZ (100 Hz). The onboard
 * WS2812 shows the micro-ROS connection state (ported from picobackdriver).
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include "driverboard.h"

/// Configure TB6612 pins, enable the driver, set 100 Hz 12-bit PWM, eyes off.
void initEyes();

/// Set one eye's brightness (idx = EYE_LEFT / EYE_RIGHT, duty 0..4095) and record
/// it in b.eye_state.
void setEye(uint8_t idx, uint16_t duty);

/// Apply b.eye_pwm[] to both eyes (called from the Core 0 loop).
void applyEyes();

/// Turn both eyes off immediately (watchdog / shutdown).
void eyesOff();

/// Update the onboard WS2812 from the micro-ROS agent state (call ~every loop).
void updateStatusLED();
