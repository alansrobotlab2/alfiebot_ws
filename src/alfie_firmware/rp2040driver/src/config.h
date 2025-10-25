/**
 * config.h
 * 
 * Configuration definitions for RP2040 I2C Slave Device
 */

#pragma once

// ============================================================================
// I2C Configuration
// ============================================================================
#define I2C_SLAVE_ADDRESS 0x58
#define I2C_CLOCK_SPEED 400000  // Fast Mode: 400 kHz (Standard Mode would be 100 kHz)
#define I2C_WATCHDOG_TIMEOUT_MS 100  // Expected 100Hz communication (10ms period)

// I2C pins (default for RP2040)
// SDA: GPIO 4 (Pin 6)
// SCL: GPIO 5 (Pin 7)

// ============================================================================
// WS2812B RGB LED Configuration
// ============================================================================
#define WS2812_PIN 16

// ============================================================================
// Motor/LED PWM Configuration (Eye LEDs)
// ============================================================================
// Left Eye
#define PWMA_PIN 6    // Left eye PWM
#define AIN1_PIN 7    // Left eye direction A
#define AIN2_PIN 8    // Left eye direction B

// Right Eye
#define PWMB_PIN 9    // Right eye PWM
#define BIN1_PIN 10   // Right eye direction A
#define BIN2_PIN 11   // Right eye direction B

// Back Switch
#define BACK_SWITCH_PIN 12  // Back switch input (with internal pullup)

// Temperature Sensor
#define TEMP_ADC_PIN 4      // ADC4 - RP2040 internal temperature sensor (ADC channel 4)

// PWM Settings
#define PWM_FREQUENCY 8192  // Target: 8192 Hz PWM frequency (framework dependent)
#define PWM_RESOLUTION 8    // 8-bit PWM (0-255)

// ============================================================================
// I2C Command Protocol
// ============================================================================
// Commands (write operations - master sends to slave)
#define CMD_SET_EYES 0x01      // Set eye PWM values: [0x01, LEFT_PWM, RIGHT_PWM]
#define CMD_GET_STATUS 0x02    // Request status (triggers status response on next read)

// Status Response Packet (read operation - slave sends to master)
// Sent in response to requestEvent (I2C read)
// Byte 0: Left eye PWM (0-255)
// Byte 1: Right eye PWM (0-255)
// Byte 2: Back switch state (0=not pressed, 1=pressed)
// Byte 3: Temperature (whole degrees C, 0-255)
#define STATUS_PACKET_SIZE 4

// ============================================================================
// Heartbeat LED Pattern Configuration
// ============================================================================
#define HEARTBEAT_INTERVAL_MS 1000   // 1 second between heartbeat cycles
#define HEARTBEAT_PULSE_MS 125       // 125ms per pulse (8 pulses = 1 second)

// Timeout LED Pattern Configuration
#define TIMEOUT_BLINK_MS 250         // 250ms blink period (4 Hz: on/off 4 times per second)

// Heartbeat pattern: double pulse, then pause
// Pattern array defined in main.cpp
