/**
 * DriverBoard.h
 * 
 * Class representing the RP2040 driver board state
 * Encapsulates eye PWM values, sensor readings, and device status
 */

#pragma once

#include <Arduino.h>

class DriverBoard {
public:
  // Eye PWM values
  uint8_t leftEyePWM;          // Left eye PWM value (0-255)
  uint8_t rightEyePWM;         // Right eye PWM value (0-255)
  
  // Sensor readings
  bool backSwitchPressed;      // Back switch state (true=pressed)
  uint8_t chipTemperature;     // Temperature in whole degrees C
  
  /**
   * Constructor - initialize with default values
   */
  DriverBoard() 
    : leftEyePWM(0)
    , rightEyePWM(0)
    , backSwitchPressed(false)
    , chipTemperature(25)  // Default 25°C
  {
  }
};
