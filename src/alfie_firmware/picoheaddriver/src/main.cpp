/**
 * @file main.cpp
 * @brief RP2040-Zero head module driver — dual-core.
 *
 *   Core 0 (setup/loop):   serial-bus servos + eye LEDs + status LED
 *   Core 1 (setup1/loop1): micro-ROS comms (HeadCmd in / HeadState out)
 *
 * Hardware:
 *   - 3 Feetech ST/SMS bus servos (pan, tilt, roll) on Waveshare Bus Servo
 *     Adapter A over UART0 (GP12=TX / GP13=RX), 1 Mbps.
 *   - 2 eye LEDs via TB6612FNG, PWM @ 100 Hz.
 *   - onboard WS2812 status/heartbeat LED (GP16).
 *
 * @author Alfie Bot Project
 */

#include <Arduino.h>
#include "config.h"
#include "driverboard.h"
#include "servo_control.h"
#include "eye_control.h"
#include "ros_interface.h"

// Global shared state (declared extern in driverboard.h).
DriverBoard b;

// =============================================================================
// CORE 0 — servo bus, eye LEDs, status LED
// =============================================================================
void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // Serial bus servos on UART0. Remap off the default GP0/GP1 (GP1 is the
    // Eye A PWM pin) BEFORE begin().
    // Grow the RX FIFO before begin(): the multi-servo sync-read reply arrives as
    // one back-to-back burst that overflows the earlephilhower default 32-byte RX
    // buffer and drops feedback. 256 absorbs the whole batch.
    Serial1.setTX(SERVO_UART_TX);
    Serial1.setRX(SERVO_UART_RX);
    Serial1.setFIFOSize(256);
    Serial1.begin(SERVO_BAUD);
    b.st.pSerial = &Serial1;

    b.statusLED.begin();
    initEyes();

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
        applyEyes();           // push b.eye_pwm[] to the TB6612
        serviceMemoryRequest();// one-shot register read parked by the ROS service
    }

    updateStatusLED();         // self-throttled to LED_BLINK_PERIOD_MS
    delay(1);
}

// =============================================================================
// CORE 1 — micro-ROS
// =============================================================================
void setup1()
{
    delay(1000);               // let Core 0 bring hardware up first
    initializeRosInterface();
}

void loop1()
{
    // Drift-free 100 Hz tick: EXECUTE_AT_RATE_MS advances the deadline by a fixed
    // ROS_TASK_PERIOD_MS instead of resetting it to "now", so the task's own
    // run-time no longer stretches the true period.
    EXECUTE_AT_RATE_MS(ROS_TASK_PERIOD_MS, rosStateMachineTask());

    delay(1);
}
