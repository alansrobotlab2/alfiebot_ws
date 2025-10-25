/**
 * WS2812.h
 * 
 * WS2812B RGB LED driver using RP2040 PIO
 */

#pragma once

#include <Arduino.h>
#include "hardware/pio.h"
#include "ws2812.pio.h"

class WS2812 {
public:
  /**
   * Constructor
   * @param pin GPIO pin number for WS2812 data line
   * @param frequency WS2812 frequency in Hz (default 800kHz)
   */
  WS2812(uint pin, float frequency = 800000);
  
  /**
   * Initialize the WS2812 LED
   * @return true if successful, false otherwise
   */
  bool begin();
  
  /**
   * Set RGB color
   * @param r Red value (0-255)
   * @param g Green value (0-255)
   * @param b Blue value (0-255)
   */
  void setColor(uint8_t r, uint8_t g, uint8_t b);
  
  /**
   * Set color using HSV
   * @param hue Hue value (0-255)
   * @param sat Saturation (0-255)
   * @param val Value/brightness (0-255)
   */
  void setHSV(uint8_t hue, uint8_t sat, uint8_t val);
  
  /**
   * Turn off the LED
   */
  void clear();
  
  /**
   * Rainbow animation cycle
   * @param delayMs Delay between steps in milliseconds
   */
  void rainbowCycle(uint delayMs = 10);

private:
  uint _pin;
  float _frequency;
  PIO _pio;
  uint _sm;
  uint _offset;
  bool _initialized;
  
  /**
   * Convert HSV to RGB
   */
  void hsv2rgb(uint8_t hue, uint8_t sat, uint8_t val, uint8_t &r, uint8_t &g, uint8_t &b);
};
