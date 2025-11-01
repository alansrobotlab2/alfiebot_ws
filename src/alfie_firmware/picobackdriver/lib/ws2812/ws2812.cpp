/**
 * WS2812.cpp
 * 
 * WS2812B RGB LED driver implementation using RP2040 PIO
 */

#include "ws2812.h"

WS2812::WS2812(uint pin, float frequency)
  : _pin(pin), _frequency(frequency), _pio(nullptr), _sm(0), _offset(0), _initialized(false) {
}

bool WS2812::begin() {
  // Try to use PIO0 first
  _pio = pio0;
  
  // Claim an unused state machine
  int sm_result = pio_claim_unused_sm(_pio, false);
  if (sm_result < 0) {
    // Try PIO1 if PIO0 is full
    _pio = pio1;
    sm_result = pio_claim_unused_sm(_pio, false);
    if (sm_result < 0) {
      return false;  // No available state machines
    }
  }
  _sm = (uint)sm_result;
  
  // Add the PIO program
  if (!pio_can_add_program(_pio, &ws2812_program)) {
    pio_sm_unclaim(_pio, _sm);
    return false;
  }
  _offset = pio_add_program(_pio, &ws2812_program);
  
  // Initialize the PIO program
  ws2812_program_init(_pio, _sm, _offset, _pin, _frequency, false);
  
  _initialized = true;
  
  // Turn off LED initially
  clear();
  
  return true;
}

void WS2812::setColor(uint8_t r, uint8_t g, uint8_t b) {
  if (!_initialized) return;
  
  // WS2812 expects GRB format
  uint32_t pixel_grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
  pio_sm_put_blocking(_pio, _sm, pixel_grb << 8u);
}

void WS2812::setHSV(uint8_t hue, uint8_t sat, uint8_t val) {
  uint8_t r, g, b;
  hsv2rgb(hue, sat, val, r, g, b);
  setColor(r, g, b);
}

void WS2812::clear() {
  setColor(0, 0, 0);
}

void WS2812::rainbowCycle(uint delayMs) {
  for (int i = 0; i < 256; i += 5) {
    setHSV(i, 255, 100);
    delay(delayMs);
  }
}

void WS2812::hsv2rgb(uint8_t hue, uint8_t sat, uint8_t val, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (sat == 0) {
    r = g = b = val;
    return;
  }
  
  uint8_t region = hue / 43;
  uint8_t remainder = (hue - (region * 43)) * 6;
  
  uint8_t p = (val * (255 - sat)) >> 8;
  uint8_t q = (val * (255 - ((sat * remainder) >> 8))) >> 8;
  uint8_t t = (val * (255 - ((sat * (255 - remainder)) >> 8))) >> 8;
  
  switch (region) {
    case 0:  r = val; g = t; b = p; break;
    case 1:  r = q; g = val; b = p; break;
    case 2:  r = p; g = val; b = t; break;
    case 3:  r = p; g = q; b = val; break;
    case 4:  r = t; g = p; b = val; break;
    default: r = val; g = p; b = q; break;
  }
}
