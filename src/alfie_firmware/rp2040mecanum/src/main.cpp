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
#include "config.h"
#include "ros_interface.h"
#include "motor_control.h"
#include "driverboard.h"

// =============================================================================
// GLOBAL DRIVERBOARD INSTANCE
// =============================================================================

// Global DriverBoard instance
DriverBoard rp;

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
    
    // Initialize hardware peripherals
    rp.initializePeripherals();
}

/**
 * @brief Core 0 main loop - Handles peripheral management
 * Runs motor control, encoder reading, odometry, and safety monitoring
 */
void loop() {
    static uint32_t last_control_time = 0;
    uint32_t current_time = millis();
    
    // Run control loop at specified frequency
    if (current_time - last_control_time >= CONTROL_LOOP_PERIOD_MS) {
        last_control_time = current_time;
        
        // Update all peripherals (motors, encoders, odometry, safety)
        rp.updatePeripherals();
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
    static uint32_t last_wake_time = 0;
    const uint32_t frequency_ms = ROS_TASK_PERIOD_MS; // 10ms = 100Hz
    
    // Initialize the last_wake_time variable with the current time
    if (last_wake_time == 0) {
        last_wake_time = millis();
    }
    
    // Wait for the next cycle (precise 100 Hz timing)
    uint32_t current_time = millis();
    uint32_t elapsed = current_time - last_wake_time;
    
    if (elapsed >= frequency_ms) {
        last_wake_time = current_time;
        
        // Execute ROS state machine task
        rosStateMachineTask();
    }
    
    // Small delay to prevent overwhelming the CPU
    delay(1);
}