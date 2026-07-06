#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// The RP2040-Zero's only onboard LED is a single WS2812 RGB on GPIO16.
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 16
#endif

Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Heartbeat: 1 = on, 0 = off. 100ms per segment -> 800ms loop.
const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
const uint8_t pattern_len = sizeof(heartbeat_pattern) / sizeof(heartbeat_pattern[0]);
const uint16_t segment_ms = 100;

void setup() {
  led.begin();
  led.setBrightness(64);  // WS2812 green at full brightness is very bright
  led.clear();
  led.show();
}

void loop() {
  for (uint8_t i = 0; i < pattern_len; i++) {
    led.setPixelColor(0, heartbeat_pattern[i] ? led.Color(0, 255, 0) : 0);
    led.show();
    delay(segment_ms);
  }
}
