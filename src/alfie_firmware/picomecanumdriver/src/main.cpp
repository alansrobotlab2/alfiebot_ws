/**
 * @file main.cpp
 * @brief RP2040 Mecanum Drive Robot - Dual Core Implementation
 * 
 * This implementation uses both cores of the RP2040:
 * - Core 0: Peripheral Management (Motors, Encoders, Sensors) - setup() + loop()
 * - Core 1: ROS2 Communications and High-level Control - setup1() + loop1()
 * 
 * @author Alfie Bot Project
 * @date 2025-10-24
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "ros_interface.h"
#include "motor_control.h"
#include "driverboard.h"

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance
DriverBoard rp;

// Onboard WS2812 status LED (single pixel on GPIO16)
static Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

// =============================================================================
// CORE 0: SETUP AND LOOP (Peripheral Management)
// =============================================================================

/**
 * @brief Core 0 setup function - runs once on startup
 * Initializes hardware peripherals for motor control and sensors
 */
void setup() {
    // Initialize serial communication for micro-ROS
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 5000) {
        // Wait for serial connection or timeout
    }

    // Initialize the onboard WS2812 status LED
    statusLed.begin();
    statusLed.setBrightness(LED_BRIGHTNESS);
    statusLed.clear();
    statusLed.show();

    // Initialize hardware peripherals
    rp.initializePeripherals();
}

/**
 * @brief Core 0 main loop - Handles peripheral management
 * Runs motor control, encoder reading, odometry, safety monitoring, and LED status
 */
void loop() {
    static uint32_t last_control_time = 0;
    static uint32_t last_led_time = 0;
    static uint8_t led_step = 0;
    uint32_t current_time = millis();
    
    // Run control loop at specified frequency
    if (current_time - last_control_time >= CONTROL_LOOP_PERIOD_MS) {
        last_control_time = current_time;
        
        // Update all peripherals (motors, encoders, odometry, safety)
        rp.updatePeripherals();
    }
    
    // LED status blink pattern (updates every LED_BLINK_PERIOD_MS = 125ms)
    if (current_time - last_led_time >= LED_BLINK_PERIOD_MS) {
        last_led_time = current_time;
        
        // Define blink patterns
        // Normal pattern: 1,0,1,0,1,0,1,0 (when system loaded but ROS not connected)
        // Heartbeat pattern: 1,0,1,0,0,0,0,0 (when ROS is connected)
        const uint8_t normal_pattern[] = {1, 0, 1, 0, 1, 0, 1, 0};
        const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
        
        // Select pattern based on ROS connection state
        bool led_state;
        if (agent_state == AGENT_CONNECTED) {
            led_state = heartbeat_pattern[led_step];
        } else {
            led_state = normal_pattern[led_step];
        }

        // Green when on, off otherwise
        statusLed.setPixelColor(0, led_state ? statusLed.Color(0, 255, 0) : 0);
        statusLed.show();

        // Advance to next step (8 steps total, wraps around)
        led_step = (led_step + 1) % 8;
    }
    
    // Small delay to prevent overwhelming the CPU
    delay(1);
}

// =============================================================================
// CORE 1: SETUP AND LOOP (ROS Communications)
// =============================================================================

/**
 * @brief Core 1 setup function - runs once on startup
 * Initializes ROS2 communications and micro-ROS
 */
void setup1() {
    // Wait for Core 0 to finish basic initialization
    delay(1000);
    
    // Initialize ROS communications
    initializeRosInterface();
}

/**
 * @brief Core 1 main loop - Handles ROS2 communications with state machine
 * Manages micro-ROS subscriber and publisher using a robust state machine approach
 */
void loop1() {
    // Drift-free 100 Hz tick: EXECUTE_AT_RATE_MS advances the deadline by a fixed
    // ROS_TASK_PERIOD_MS instead of resetting it to "now", so the task's own
    // run-time no longer stretches the true period.
    EXECUTE_AT_RATE_MS(ROS_TASK_PERIOD_MS, rosStateMachineTask());

    // Small delay to prevent overwhelming the CPU
    delay(1);
}