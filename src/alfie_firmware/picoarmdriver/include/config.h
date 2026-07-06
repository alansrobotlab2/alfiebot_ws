/**
 * @file config.h
 * @brief Configuration for the RP2040-Zero arm driver.
 *
 * Arm hardware (identical left and right; the board self-identifies at boot):
 *   - 7 Feetech ST/SMS serial bus servos on a Waveshare Bus Servo Adapter A
 *     (half-duplex bus over UART0, GP12=TX / GP13=RX, 1 Mbps). Bus IDs = index+1.
 *   - onboard WS2812 status/heartbeat LED (GP16).
 *   - micro-ROS over USB serial.
 *
 * The arm exposes 6 LOGICAL joints; the shoulder-pitch joint is realised as a
 * mirrored 2-servo coupled pair (physical servos 2 and 3), so 6 joints -> 7
 * servos. The 6<->7 expansion (previously host-side in alfie_bringup/master_cmd
 * + master_status) lives in lib/arm_control.
 *
 *   Physical servo idx | bus ID | logical joint
 *   -------------------+--------+---------------------------------
 *          0           |   1    | shoulder_yaw
 *          1           |   2    | shoulder_pitch   (primary of pair)
 *          2           |   3    | shoulder_pitch   (derived - mirror)
 *          3           |   4    | elbow_pitch
 *          4           |   5    | wrist_pitch
 *          5           |   6    | wrist_roll
 *          6           |   7    | gripper
 *
 * @author Alfie Bot Project
 */

#pragma once

#include <Arduino.h>

// =============================================================================
// SERIAL / micro-ROS
// =============================================================================
#define SERIAL_BAUD_RATE        1000000  ///< USB serial baud for the micro-ROS agent

// ROS node name is shared by both arms; the namespace carries the left/right
// distinction (chosen at boot from the board serial, see lib/arm_control).
#define ARM_NODE_NAME           "arm_controller"
#define ARM_NS_LEFT             "alfie/low/left_arm"
#define ARM_NS_RIGHT            "alfie/low/right_arm"
#define ARM_NS_UNKNOWN          "alfie/low/unknown_arm"

// =============================================================================
// BOARD IDENTITY (left vs right)
// =============================================================================
// arm_side values.
#define ARM_SIDE_LEFT           0
#define ARM_SIDE_RIGHT          1
#define ARM_SIDE_UNKNOWN        255

// Pico unique board id (16 uppercase hex chars from pico_get_unique_board_id_string).
// Bootstrapping: flash this firmware, the board comes up under ARM_NS_UNKNOWN;
// `ros2 topic echo <ns>/armstate` -> read board_serial, or check the serial in
// ArmState, then paste the 16-char id below and re-flash both boards.
// NOTE: this id is the QSPI flash RUID; it changes if the flash chip is replaced.
#define ARM_SERIAL_LEFT         "5303284738D1359C"
#define ARM_SERIAL_RIGHT        "530328474102739C"

// =============================================================================
// SERIAL BUS SERVOS (Feetech ST/SMS via Waveshare Bus Servo Adapter A)
// =============================================================================
#define SERVO_UART_TX           12       ///< UART0 TX -> adapter RXD
#define SERVO_UART_RX           13       ///< UART0 RX <- adapter TXD
#define SERVO_BAUD              1000000   ///< ST/SMS bus baud (1 Mbps)

#define NUM_SERVOS              7         ///< physical servos (bus ID = index + 1)
#define NUM_JOINTS             6         ///< logical joints exposed over ROS

// Logical joint indices (order of ArmCmd/ArmState arrays).
#define JOINT_SHOULDER_YAW      0
#define JOINT_SHOULDER_PITCH    1
#define JOINT_ELBOW_PITCH       2
#define JOINT_WRIST_PITCH       3
#define JOINT_WRIST_ROLL        4
#define JOINT_GRIPPER           5

// Coupled-pair layout: physical servo 2 (index) is the derived mirror of the
// shoulder-pitch primary (physical servo 1).
#define SERVO_SHOULDER_PITCH_PRIMARY   1
#define SERVO_SHOULDER_PITCH_DERIVED   2

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
// STATUS LED (onboard WS2812)
// =============================================================================
#define WS2812B_PIN             16        ///< onboard WS2812 data pin
#define LED_BLINK_PERIOD_MS     125       ///< status LED tick (8 steps/second)

// =============================================================================
// LOOP TIMING
// =============================================================================
#define SERVO_LOOP_PERIOD_MS    10        ///< Core 0 servo bus tick (100 Hz)
#define ROS_TASK_PERIOD_MS      10        ///< Core 1 micro-ROS tick (100 Hz)

// =============================================================================
// SAFETY
// =============================================================================
// If no ArmCmd arrives within this window, disable all servo torque.
#define WATCHDOG_TIMEOUT_MS     500

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
#define AGENT_HEALTH_TIMEOUT_MS 100
#define AGENT_PING_ATTEMPTS     1
#define AGENT_HEALTH_ATTEMPTS   10

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
