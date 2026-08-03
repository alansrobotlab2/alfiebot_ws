/**
 * @file config.h
 * @brief Configuration for the RP2040-Zero head module driver.
 *
 * Head module hardware:
 *   - 3 Feetech ST/SMS serial bus servos on a Waveshare Bus Servo Adapter A
 *     (half-duplex bus over UART0, GP12=TX / GP13=RX, 1 Mbps). Bus IDs:
 *       ID 1 = pan, ID 2 = tilt, ID 3 = roll
 *   - 2 eye LEDs, each on one channel of a TB6612FNG H-bridge, PWM @ 100 Hz.
 *   - onboard WS2812 status/heartbeat LED (GP16).
 *   - micro-ROS over USB serial.
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>

#define NAMESPACE "alfie/low"

// =============================================================================
// SERIAL / micro-ROS
// =============================================================================
#define SERIAL_BAUD_RATE        1000000  ///< USB serial baud for the micro-ROS agent

// =============================================================================
// SERIAL BUS SERVOS (Feetech ST/SMS via Waveshare Bus Servo Adapter A)
// =============================================================================
#define SERVO_UART_TX           12       ///< UART0 TX -> adapter RXD
#define SERVO_UART_RX           13       ///< UART0 RX <- adapter TXD
#define SERVO_BAUD              1000000   ///< ST/SMS bus baud (1 Mbps)
#define NUM_SERVOS              3         ///< pan(1), tilt(2), roll(3)

// Servo index <-> joint mapping (array index; bus ID = index + 1)
#define SERVO_PAN               0
#define SERVO_TILT              1
#define SERVO_ROLL              2

// Feetech STS position scale: 4096 counts / 360deg, center = 2048.
#define SERVO_COUNTS_PER_REV    4096
#define SERVO_CENTER_COUNT      2048
#define SERVO_MIN_COUNT         0
#define SERVO_MAX_COUNT         4095
// Counts per radian = 4096 / (2*pi)
#define SERVO_COUNTS_PER_RAD    (SERVO_COUNTS_PER_REV / (2.0f * PI))
// STS acceleration register unit is 100 steps/s^2 per LSB.
#define SERVO_ACCEL_STEPS_PER_UNIT  100.0f
// STS present-current register unit ~ 6.5 mA per LSB.
#define SERVO_CURRENT_MA_PER_UNIT   6.5f
// Default torque limit (0..1000, 1000 = 100%) used when a command sends 0.
#define SERVO_DEFAULT_TORQUE    1000

// Serial-bus servo memory-map register addresses (absolute, STS/SMS).
#define SBS_TORQUEENABLE        0x28
#define SBS_ACCELERATION        0x29
#define SBS_TARGETLOCATION      0x2A
#define SBS_CURRENTLOCATION     0x38
// Bytes read back per status poll: 0x38..0x46 (location..current) = 15 bytes.
#define SBS_STATUS_READ_LEN     15
// Bytes written per active-servo command packet: acc + pos + time + speed + torque.
#define SERVO_CMD_PACKET_SIZE   9

// =============================================================================
// EYE LEDs (TB6612FNG H-bridge, PWM @ 100 Hz)
// =============================================================================
#define EYE_STBY_PIN            4         ///< TB6612 STBY (HIGH = enabled)

// Eye A = left eye (channel A)
#define EYE_A_PWM_PIN           1         ///< PWMA
#define EYE_A_IN1_PIN           3         ///< AIN1
#define EYE_A_IN2_PIN           2         ///< AIN2

// Eye B = right eye (channel B)
#define EYE_B_PWM_PIN           7         ///< PWMB
#define EYE_B_IN1_PIN           5         ///< BIN1
#define EYE_B_IN2_PIN           6         ///< BIN2

#define NUM_EYES                2
#define EYE_LEFT                0
#define EYE_RIGHT               1
#define EYE_PWM_FREQ_HZ         100       ///< Eye LED PWM frequency
#define EYE_PWM_MAX             4095      ///< 12-bit duty range

// =============================================================================
// STATUS LED (onboard WS2812)
// =============================================================================
#define WS2812B_PIN             16        ///< onboard WS2812 data pin
#define LED_BLINK_PERIOD_MS     125       ///< status LED tick (8 steps/second)

// =============================================================================
// LOOP TIMING
// =============================================================================
#define SERVO_LOOP_PERIOD_MS    20        ///< Core 0 servo bus tick (50 Hz)
#define ROS_TASK_PERIOD_MS      10        ///< Core 1 micro-ROS tick (100 Hz)
#define STATE_PUBLISH_PERIOD_MS 20        ///< Outbound HeadState publish cadence (50 Hz)

// =============================================================================
// SAFETY
// =============================================================================
// If no HeadCmd arrives within this window, disable servo torque + eyes off.
#define WATCHDOG_TIMEOUT_MS     500

// -----------------------------------------------------------------------------
// Register-map service
// -----------------------------------------------------------------------------
// Operations accepted by the register-map service. Anything else is rejected.
#define SERVO_SERVICE_OP_READ    'r'   ///< read the whole register table
#define SERVO_SERVICE_OP_WRITE   'w'   ///< write `value` to `address`
#define SERVO_SERVICE_OP_UNLOCK  'u'   ///< clear the EEPROM write lock
#define SERVO_SERVICE_OP_LOCK    'l'   ///< set the EEPROM write lock

// Lock flag (0x37). The vendor table is counter-intuitive here: writing 0
// DISABLES the lock, so 0 means EEPROM is writable.
#define SBS_LOCK                 0x37
#define SERVO_EPROM_UNLOCKED     0
#define SERVO_EPROM_LOCKED       1

// First SRAM address. Everything below this is EEPROM: it survives power
// cycles, wears out with writes, and is refused while the servo is locked or
// holding torque.
#define SERVO_EPROM_END          0x28

// Write outcome, reported in ServoMemoryMap.writebyteresult.
#define SERVO_WRITE_NONE         0     ///< no write attempted (plain read)
#define SERVO_WRITE_OK           1
#define SERVO_WRITE_NOT_WRITABLE 2     ///< address not in the writable table
#define SERVO_WRITE_LOCKED       3     ///< EEPROM write while the lock is set
#define SERVO_WRITE_TORQUE_ON    4     ///< EEPROM write while the servo holds torque
#define SERVO_WRITE_BUS_FAILED   5     ///< the servo did not acknowledge
#define SERVO_WRITE_MISMATCH     6     ///< wrote, but the read-back disagrees
#define SERVO_WRITE_BAD_VALUE    7     ///< value does not fit the register width

// How long the Core 1 service callback waits for Core 0 to run the operation on
// its servo tick. One tick is SERVO_LOOP_PERIOD_MS, so this is several ticks of
// slack; exceeding it means Core 0 is wedged and the service reports failure
// rather than blocking the ROS executor indefinitely. A write does up to four
// bus transactions (lock check, write, re-read), so it needs more room than a
// plain read.
#define MEM_REQ_TIMEOUT_MS       250

// =============================================================================
// MATH
// =============================================================================
#ifndef PI
#define PI                      3.14159265358979323846
#endif

// =============================================================================
// ROS STATE MACHINE
// =============================================================================
typedef enum {
    WAITING_AGENT = 0,      ///< Waiting for micro-ROS agent
    AGENT_AVAILABLE,        ///< Agent available, creating entities
    AGENT_CONNECTED,        ///< Connected, entities created
    AGENT_DISCONNECTED      ///< Disconnected, cleanup needed
} RosAgentState_t;

#define AGENT_PING_INTERVAL_MS  100
#define AGENT_HEALTH_CHECK_MS   200
#define AGENT_PING_TIMEOUT_MS   50

// Health check while CONNECTED: rmw_uros_ping_agent BLOCKS the whole ROS loop
// (including the HeadState publisher) for up to timeout*attempts, so keep a single
// short attempt and tolerate consecutive misses instead. Old config (100ms x 10)
// stalled the loop up to 1s per check whenever the agent replied slowly, which
// collapsed the published state rate (observed on the arms: ~15-20 Hz).
#define AGENT_HEALTH_TIMEOUT_MS 25
#define AGENT_PING_ATTEMPTS     1
#define AGENT_HEALTH_ATTEMPTS   1
// Consecutive failed health pings before declaring AGENT_DISCONNECTED
// (10 x 200ms cycle = ~2s of unresponsive agent, same effective latency as before).
#define AGENT_HEALTH_MAX_MISSES 10

// If the agent only shows up after the board has been waiting this long, the
// USB-CDC FIFOs / XRCE framing on both ends are full of hours of stale ping
// traffic and sessions establish degraded (observed: state rate stuck at
// ~15-20 Hz until a power cycle). Reboot instead of connecting: re-enumeration
// recreates the known-good fresh-boot condition, and the agent (respawn=True)
// reconnects to a seconds-old board. Normal bringup never waits this long.
#define AGENT_LONG_WAIT_REBOOT_MS 120000

/**
 * @brief Run @p code no more than once per @p interval_ms milliseconds.
 */
#define EXECUTE_EVERY_N_MS(interval_ms, code) \
    do { \
        static uint32_t last_execution = 0; \
        uint32_t current_time = millis(); \
        if (current_time - last_execution >= (uint32_t)(interval_ms)) { \
            last_execution = current_time; \
            code; \
        } \
    } while (0)

/**
 * @brief Run @p code on a fixed cadence of @p interval_ms (drift-free).
 *
 * Unlike EXECUTE_EVERY_N_MS, which resets its deadline to "now" (so the task's
 * own run-time stretches the true period), this advances the deadline by exactly
 * @p interval_ms, keeping the long-run average rate exact. If the loop falls more
 * than one interval behind (e.g. after an agent stall) the deadline is clamped
 * forward so it never fires a catch-up burst. Signed comparison tolerates the
 * millis() wraparound.
 */
#define EXECUTE_AT_RATE_MS(interval_ms, code) \
    do { \
        static uint32_t next_deadline_ms = 0; \
        uint32_t now_ms = millis(); \
        if ((int32_t)(now_ms - next_deadline_ms) >= 0) { \
            next_deadline_ms += (uint32_t)(interval_ms); \
            if ((int32_t)(now_ms - next_deadline_ms) >= 0) { \
                next_deadline_ms = now_ms + (uint32_t)(interval_ms); \
            } \
            code; \
        } \
    } while (0)
