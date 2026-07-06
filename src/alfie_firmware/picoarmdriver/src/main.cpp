/**
 * @file main.cpp
 * @brief RP2040-Zero arm driver — dual-core.
 *
 *   Core 0 (setup/loop):   serial-bus servos + status LED
 *   Core 1 (setup1/loop1): micro-ROS comms (ArmCmd in / ArmState out)
 *
 * Hardware:
 *   - 7 Feetech ST/SMS bus servos (6 logical joints; shoulder-pitch is a
 *     mirrored 2-servo coupled pair) on a Waveshare Bus Servo Adapter A over
 *     UART0 (GP12=TX / GP13=RX), 1 Mbps.
 *   - onboard WS2812 status/heartbeat LED (GP16).
 *
 * The same firmware runs on both arms; the board reads its unique serial at boot
 * (armSelectSide) to become the left or right arm.
 *
 * @author Alfie Bot Project
 */

#include <Arduino.h>
#include "config.h"
#include "driverboard.h"
#include "servo_control.h"
#include "arm_control.h"
#include "status_led.h"
#include "ros_interface.h"

// Global shared state (declared extern in driverboard.h).
DriverBoard b;

// =============================================================================
// CORE 0 — servo bus, status LED
// =============================================================================
void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Serial bus servos on UART0. Remap off the default GP0/GP1 BEFORE begin().
    Serial1.setTX(SERVO_UART_TX);
    Serial1.setRX(SERVO_UART_RX);
    Serial1.begin(SERVO_BAUD);
    b.st.pSerial = &Serial1;

    // Decide left vs right from the Pico serial before anything uses polarity
    // or the ROS namespace.
    armSelectSide();

    b.statusLED.begin();

    // Give the bus a moment, then sync the mirror to the servos' present state.
    delay(50);
    initServoState();
}

void loop()
{
    static uint32_t last_servo_tick = 0;
    uint32_t now = millis();

    if (now - last_servo_tick >= SERVO_LOOP_PERIOD_MS) {
        last_servo_tick = now;

        updateServoStatus();   // sync-read feedback -> mBuf
        updateServoActive();   // sync-write pos/speed/accel/torque (enabled servos)
        updateServoIdle();     // sync-write torque-off (disabled servos)
    }

    updateStatusLED();         // self-throttled to LED_BLINK_PERIOD_MS
    delay(1);
}

// =============================================================================
// CORE 1 — micro-ROS
// =============================================================================
void setup1()
{
    delay(1000);               // let Core 0 bring hardware up + select side first
    initializeRosInterface();
}

void loop1()
{
    static uint32_t last_ros_tick = 0;
    uint32_t now = millis();

    if (now - last_ros_tick >= ROS_TASK_PERIOD_MS) {
        last_ros_tick = now;
        rosStateMachineTask();
    }

    delay(1);
}
