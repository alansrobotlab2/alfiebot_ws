/**
 * @file status_led.h
 * @brief Onboard WS2812 status/heartbeat LED behavior.
 *
 * The onboard WS2812 shows the micro-ROS connection state (ported from
 * picoheaddriver). If the board could not identify itself as left or right
 * (unknown serial), it shows a distinctive colour and the arm is kept limp.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>
#include "driverboard.h"

/// Update the onboard WS2812 from the micro-ROS agent state (call ~every loop).
void updateStatusLED();
