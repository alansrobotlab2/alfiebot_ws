#pragma once

#include <Arduino.h>

/**
 * @brief Initializes the shoulder limit switch GPIO pin as input
 * 
 * Configures GPIO pin 34 (SHOULDER_LIMIT_SWITCH_PIN) as a digital input
 * for reading the shoulder mechanism limit switch state.
 * 
 * @note Should be called once during system initialization
 * @note Pin 34 is input-only on ESP32 and does not support pull-up/pull-down
 * @see SHOULDER_LIMIT_SWITCH_PIN for pin number definition
 */
void setupShoulderLimitSwitch();


/**
 * @brief Reads the current state of the shoulder limit switch with inverted logic
 * 
 * Reads the digital state of the shoulder limit switch and inverts the logic
 * so that HIGH (not triggered) returns true and LOW (triggered) returns false.
 * This provides more intuitive semantics where true = switch open/not pressed
 * and false = switch closed/pressed.
 * 
 * Previously returned active-low logic (LOW == triggered), but this function
 * reverses the output for clearer code logic.
 * 
 * @return true if limit switch is not triggered (pin reads HIGH)
 * @return false if limit switch is triggered (pin reads LOW)
 * @note Pin must be initialized with setupShoulderLimitSwitch() first
 * @see SHOULDER_LIMIT_SWITCH_PIN for pin number definition
 */
bool readShoulderLimitSwitch();


#define SHOULDER_LIMIT_SWITCH_PIN 34




