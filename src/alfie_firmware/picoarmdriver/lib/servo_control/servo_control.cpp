#include "servo_control.h"

extern DriverBoard b;

// ---------------------------------------------------------------------------
// Bus loop
// ---------------------------------------------------------------------------

void updateServoStatus()
{
    // One sync-read TX for all servos, then per-servo RX into the mirror.
    b.st.syncReadPacketTx(b.IDS, NUM_SERVOS, SBS_CURRENTLOCATION, SBS_STATUS_READ_LEN);

    for (int i = 0; i < NUM_SERVOS; i++) {
        int rx = b.st.syncReadPacketRx(i + 1, b.buf.bytes + SBS_CURRENTLOCATION);
        if (rx > 0) {
            memcpy(b.mBuf[i].bytes + SBS_CURRENTLOCATION,
                   b.buf.bytes + SBS_CURRENTLOCATION,
                   SBS_STATUS_READ_LEN);
        } else {
            // Bus out of sync -> ping everyone to recover, retry next cycle.
            flushSerialServoLine();
            return;
        }
    }
}

// One 16-bit value split into low/high bytes.
static inline void host2scs(uint8_t *dataL, uint8_t *dataH, uint16_t data)
{
    *dataL = data & 0xFF;
    *dataH = (data >> 8) & 0xFF;
}

void updateServoActive()
{
    int index = 0;

    // Build a contiguous write starting at the ACCELERATION register (0x29):
    //   [acc][posL][posH][timeL][timeH][speedL][speedH][torqueL][torqueH]
    // which lands in ACC, GOAL_POSITION, GOAL_TIME, GOAL_SPEED, TORQUE_LIMIT.
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (b.mBuf[i].memory.torqueSwitch != 1) {
            continue;
        }

        uint8_t *p = &b.servocommandbuf[SERVO_CMD_PACKET_SIZE * index];

        uint16_t torque = b.mBuf[i].memory.torqueLimit;
        if (torque == 0) {
            torque = SERVO_DEFAULT_TORQUE;  // 0 would disable output; use full torque.
        }

        p[0] = b.mBuf[i].memory.acceleration & 0xFF;
        host2scs(&p[1], &p[2], (uint16_t)b.mBuf[i].memory.targetLocation);  // 0..4095, positive
        host2scs(&p[3], &p[4], 0);                                          // goal time = 0
        host2scs(&p[5], &p[6], b.mBuf[i].memory.runningSpeed);
        host2scs(&p[7], &p[8], torque);

        b.servoCMDIDS[index] = i + 1;
        index++;
    }

    if (index > 0) {
        b.st.syncWrite(b.servoCMDIDS, index, SBS_ACCELERATION,
                       b.servocommandbuf, SERVO_CMD_PACKET_SIZE);
    }
}

void updateServoIdle()
{
    int index = 0;
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (b.mBuf[i].memory.torqueSwitch == 0) {
            b.torquecommandbuf[index] = 0;
            b.servoCMDIDS[index]      = i + 1;
            index++;
        }
    }
    if (index > 0) {
        b.st.syncWrite(b.servoCMDIDS, index, SBS_TORQUEENABLE, b.torquecommandbuf, 1);
    }
}

void flushSerialServoLine()
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        b.st.Ping(i + 1);
    }
}

void disableAllServoTorques()
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        b.mBuf[i].memory.torqueSwitch = 0;
    }
}

void initServoState()
{
    for (int i = 0; i < NUM_SERVOS; i++) {
        uint8_t id = i + 1;

        // Read full register table so the mirror starts consistent.
        int n = b.st.Read(id, 0, b.mBuf[i].bytes, sizeof(MemoryStruct));
        (void)n;

        // Cache EEPROM angle limits (fall back to full range if unset/invalid).
        int16_t lo = b.mBuf[i].memory.minAngleLimit;
        int16_t hi = b.mBuf[i].memory.maxAngleLimit;
        if (hi > lo) {
            b.minAngleCount[i] = lo;
            b.maxAngleCount[i] = hi;
        }

        // Start disabled, holding the present position so nothing jumps on enable.
        b.mBuf[i].memory.targetLocation = b.mBuf[i].memory.currentLocation;
        b.mBuf[i].memory.torqueSwitch   = 0;
        b.st.EnableTorque(id, 0);
    }
}

// ---------------------------------------------------------------------------
// SI <-> count conversions
// ---------------------------------------------------------------------------

int16_t radToCount(uint8_t servoIndex, float rad)
{
    long count = lroundf(rad * SERVO_COUNTS_PER_RAD) + SERVO_CENTER_COUNT;

    int16_t lo = b.minAngleCount[servoIndex];
    int16_t hi = b.maxAngleCount[servoIndex];
    if (count < lo) count = lo;
    if (count > hi) count = hi;
    return (int16_t)count;
}

float countToRad(int16_t count)
{
    return (float)(count - SERVO_CENTER_COUNT) / SERVO_COUNTS_PER_RAD;
}

uint16_t radPerSecToSpeed(float radPerSec)
{
    float steps = fabsf(radPerSec) * SERVO_COUNTS_PER_RAD;
    if (steps > 65535.0f) steps = 65535.0f;
    return (uint16_t)lroundf(steps);
}

float speedToRadPerSec(int16_t rawSpeed)
{
    // STS present speed is sign-magnitude in bit 15.
    int16_t signed_steps = (rawSpeed & 0x8000) ? -(rawSpeed & 0x7FFF) : rawSpeed;
    return (float)signed_steps / SERVO_COUNTS_PER_RAD;
}

uint8_t radPerSec2ToAccel(float radPerSec2)
{
    float units = (fabsf(radPerSec2) * SERVO_COUNTS_PER_RAD) / SERVO_ACCEL_STEPS_PER_UNIT;
    if (units > 254.0f) units = 254.0f;
    return (uint8_t)lroundf(units);
}
