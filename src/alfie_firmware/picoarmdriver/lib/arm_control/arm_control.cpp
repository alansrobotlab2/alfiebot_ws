#include "arm_control.h"
#include "driverboard.h"
#include "servo_control.h"
#include <string.h>
#include <pico/unique_id.h>

extern DriverBoard b;

// ---------------------------------------------------------------------------
// Static mapping tables (ported from alfie_bringup/servo_config.py SERVO_POLARITY
// and master_cmd.py / master_status.py).
// ---------------------------------------------------------------------------

// Per-servo sign for each side, physical-servo order:
//   [yaw, sh_pitch(primary), sh_pitch(derived), elbow, wrist_pitch, wrist_roll, gripper]
static const int8_t ARM_POLARITY_LEFT[NUM_SERVOS]  = { 1, -1, -1, -1,  1, -1, -1 };
static const int8_t ARM_POLARITY_RIGHT[NUM_SERVOS] = { 1,  1,  1, -1,  1, -1, -1 };

// Physical servo index -> logical joint index it is driven from.
// Servo 2 (derived) reads from the shoulder-pitch joint, same as servo 1.
static const uint8_t SERVO_JOINT_MAP[NUM_SERVOS] = {
    JOINT_SHOULDER_YAW,     // servo 0
    JOINT_SHOULDER_PITCH,   // servo 1  (primary)
    JOINT_SHOULDER_PITCH,   // servo 2  (derived - mirror of servo 1)
    JOINT_ELBOW_PITCH,      // servo 3
    JOINT_WRIST_PITCH,      // servo 4
    JOINT_WRIST_ROLL,       // servo 5
    JOINT_GRIPPER           // servo 6
};

// Logical joint index -> the PRIMARY physical servo that reports its feedback.
// (The derived shoulder-pitch servo, index 2, is never used for feedback.)
static const uint8_t JOINT_PRIMARY_SERVO[NUM_JOINTS] = {
    0,  // shoulder_yaw
    1,  // shoulder_pitch (primary)
    3,  // elbow_pitch
    4,  // wrist_pitch
    5,  // wrist_roll
    6   // gripper
};

// ---------------------------------------------------------------------------
// Board identity: which arm am I?
// ---------------------------------------------------------------------------

void armSelectSide()
{
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    memcpy((void *)b.board_serial, id.id, sizeof(id.id));
    pico_get_unique_board_id_string(b.board_serial_str, sizeof(b.board_serial_str));

    if (strcmp(b.board_serial_str, ARM_SERIAL_LEFT) == 0) {
        b.arm_side      = ARM_SIDE_LEFT;
        b.polarity      = ARM_POLARITY_LEFT;
        b.namespace_str = ARM_NS_LEFT;
    } else if (strcmp(b.board_serial_str, ARM_SERIAL_RIGHT) == 0) {
        b.arm_side      = ARM_SIDE_RIGHT;
        b.polarity      = ARM_POLARITY_RIGHT;
        b.namespace_str = ARM_NS_RIGHT;
    } else {
        // Unknown board: come up under the unknown namespace so the serial can
        // be read off ArmState, keep a valid (non-null) polarity pointer, but
        // applyArmCmd() will force all torque off while side is unknown.
        b.arm_side      = ARM_SIDE_UNKNOWN;
        b.polarity      = ARM_POLARITY_LEFT;
        b.namespace_str = ARM_NS_UNKNOWN;
    }
}

// ---------------------------------------------------------------------------
// Command: 6 joints -> 7 servos
// ---------------------------------------------------------------------------

void applyArmCmd(const alfie_msgs__msg__ArmCmd *msg)
{
    // Never energise servos with an unknown polarity.
    const bool side_known = (b.arm_side != ARM_SIDE_UNKNOWN);

    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        const uint8_t joint = SERVO_JOINT_MAP[i];
        const alfie_msgs__msg__ServoCmd *jc = &msg->joint_cmd[joint];

        // Position: mirror the derived servo, then apply this servo's polarity.
        float theta = jc->target_location;
        if (i == SERVO_SHOULDER_PITCH_DERIVED) {
            theta = -theta;
        }
        theta = (float)b.polarity[i] * theta;

        b.mBuf[i].memory.torqueSwitch   = (side_known && jc->enabled) ? 1 : 0;
        b.mBuf[i].memory.targetLocation = radToCount(i, theta);
        // Speed / acceleration / torque are magnitudes: copied straight through
        // (the derived servo naturally shares the primary's joint command).
        b.mBuf[i].memory.runningSpeed   = radPerSecToSpeed(jc->target_speed);
        b.mBuf[i].memory.acceleration   = radPerSec2ToAccel(jc->target_acceleration);

        float torque = jc->target_torque;
        if (torque < 0.0f)    torque = 0.0f;
        if (torque > 1000.0f) torque = 1000.0f;
        b.mBuf[i].memory.torqueLimit = (uint16_t)torque;
    }
}

// ---------------------------------------------------------------------------
// Feedback: 7 servos -> 6 joints
// ---------------------------------------------------------------------------

static void fillServoState(uint8_t servoIndex, int8_t polarity,
                           alfie_msgs__msg__ServoState *st)
{
    const MemoryStruct &m = b.mBuf[servoIndex].memory;
    const float pol = (float)polarity;

    st->enabled             = (m.torqueSwitch == 1);
    st->target_location     = pol * countToRad(m.targetLocation);
    st->target_speed        = speedToRadPerSec((int16_t)m.runningSpeed);
    st->target_acceleration = (float)m.acceleration * SERVO_ACCEL_STEPS_PER_UNIT
                              / SERVO_COUNTS_PER_RAD;
    st->target_torque       = (float)m.torqueLimit;

    st->current_location    = pol * countToRad(m.currentLocation);
    st->current_speed       = pol * speedToRadPerSec(m.currentSpeed);

    // Present load is sign-magnitude, magnitude 0..1000 == 0..100%.
    int16_t load = (m.currentLoad & 0x8000) ? -(int16_t)(m.currentLoad & 0x7FFF)
                                            : (int16_t)m.currentLoad;
    st->current_load        = (float)load / 10.0f;

    st->current_temperature = m.currentTemperature;
    st->servo_status        = m.servoStatus;
    st->is_moving           = (m.mobileSign != 0);
    st->current_voltage     = (float)m.currentVoltage / 10.0f;
    st->current_current     = (float)m.currentCurrent * SERVO_CURRENT_MA_PER_UNIT;
}

void fillArmState(alfie_msgs__msg__ArmState *st)
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        const uint8_t servo = JOINT_PRIMARY_SERVO[j];
        fillServoState(servo, b.polarity[servo], &st->joint_state[j]);
    }

    for (uint8_t i = 0; i < 8; i++) {
        st->board_serial[i] = b.board_serial[i];
    }
    st->arm_side = b.arm_side;
}
