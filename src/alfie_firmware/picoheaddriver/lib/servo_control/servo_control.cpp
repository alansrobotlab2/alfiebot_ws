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
// Register-map service (Core 0 half)
// ---------------------------------------------------------------------------

// Which registers this service will write, and how wide each one is. Getting
// the width wrong corrupts the neighbouring register, so the width comes from
// here rather than from the caller.
//
// Deliberately absent:
//   0x05 servo ID, 0x06 baud rate - changing either makes the servo vanish from
//     the bus, and nothing in this firmware could talk to it again. Use the
//     standalone picosetservoid sketch, one servo at a time.
//   0x28-0x30 the SRAM command block (torque switch, acceleration, target,
//     time, speed, torque limit) - the control loop rewrites all of it every
//     20 ms from mBuf, so a write here would be erased before it was read back.
//     Use the angle control instead.
//   0x37 lock flag - written by the dedicated lock/unlock operations.
typedef struct {
    uint8_t addr;
    uint8_t width;      // bytes
} WritableReg;

static const WritableReg WRITABLE_REGS[] = {
    {0x07, 1},  // return delay
    {0x08, 1},  // response status level
    {0x09, 2},  // min angle limit
    {0x0B, 2},  // max angle limit
    {0x0D, 1},  // max temperature
    {0x0E, 1},  // max input voltage
    {0x0F, 1},  // min input voltage
    {0x10, 2},  // max torque
    {0x12, 1},  // phase
    {0x13, 1},  // unloading condition
    {0x14, 1},  // LED alarm condition
    {0x15, 1},  // position loop P
    {0x16, 1},  // position loop D
    {0x17, 1},  // position loop I
    {0x18, 2},  // minimum startup force
    {0x1A, 1},  // CW dead zone
    {0x1B, 1},  // CCW dead zone
    {0x1C, 2},  // protection current
    {0x1E, 1},  // angular resolution
    {0x1F, 2},  // position correction
    {0x21, 1},  // operation mode
    {0x22, 1},  // protective torque
    {0x23, 1},  // protection time
    {0x24, 1},  // overload torque
    {0x25, 1},  // speed loop P
    {0x26, 1},  // overcurrent protection time
    {0x27, 1},  // speed loop I
};

static bool writableRegister(uint8_t addr, uint8_t *width)
{
    for (size_t i = 0; i < sizeof(WRITABLE_REGS) / sizeof(WRITABLE_REGS[0]); ++i) {
        if (WRITABLE_REGS[i].addr == addr) {
            *width = WRITABLE_REGS[i].width;
            return true;
        }
    }
    return false;
}

/// Read a register straight out of a full-table buffer. The table is read from
/// address 0 into a packed struct, so a register's address IS its byte offset.
static int32_t registerFromBuffer(const MemoryReplyBuf *buf, uint8_t addr, uint8_t width)
{
    if (width == 2) {
        const uint16_t raw = (uint16_t)buf->bytes[addr] |
                             ((uint16_t)buf->bytes[addr + 1] << 8);
        return (int32_t)(int16_t)raw;
    }
    return (int32_t)buf->bytes[addr];
}

/// Perform one guarded register write. Returns a SERVO_WRITE_* code.
static uint8_t performRegisterWrite(uint8_t id, uint8_t addr, int16_t value, uint8_t *width)
{
    if (!writableRegister(addr, width)) {
        return SERVO_WRITE_NOT_WRITABLE;
    }
    if (*width == 1 && (value < 0 || value > 255)) {
        return SERVO_WRITE_BAD_VALUE;
    }

    if (addr < SERVO_EPROM_END) {
        // Torque first: changing position correction or the angle limits under
        // load makes the servo re-reference and lurch against whatever it is
        // holding.
        if (b.mBuf[id - 1].memory.torqueSwitch != 0) {
            return SERVO_WRITE_TORQUE_ON;
        }
        // Read the lock fresh rather than trusting the mirror - the mirror is
        // only as new as the last full read, and a silently-refused EEPROM
        // write is exactly the failure this is here to prevent.
        const int lock = b.st.readByte(id, SBS_LOCK);
        if (lock < 0) {
            return SERVO_WRITE_BUS_FAILED;
        }
        if (lock != SERVO_EPROM_UNLOCKED) {
            return SERVO_WRITE_LOCKED;
        }
    }

    const int rc = (*width == 2)
        ? b.st.writeWord(id, addr, (uint16_t)value)
        : b.st.writeByte(id, addr, (uint8_t)value);

    return (rc != -1) ? SERVO_WRITE_OK : SERVO_WRITE_BUS_FAILED;
}

void serviceMemoryRequest()
{
    if (!b.mem_req_pending) {
        return;
    }

    const uint8_t id  = b.mem_req_id;
    const uint8_t op  = b.mem_req_op;
    uint8_t write_result = SERVO_WRITE_NONE;
    uint8_t width        = 0;
    bool    read_ok      = false;

    if (id >= 1 && id <= NUM_SERVOS) {
        if (op == SERVO_SERVICE_OP_WRITE) {
            write_result = performRegisterWrite(id, b.mem_req_addr, b.mem_req_value, &width);
        } else if (op == SERVO_SERVICE_OP_UNLOCK || op == SERVO_SERVICE_OP_LOCK) {
            const uint8_t lock = (op == SERVO_SERVICE_OP_UNLOCK)
                ? SERVO_EPROM_UNLOCKED : SERVO_EPROM_LOCKED;
            write_result = (b.st.writeByte(id, SBS_LOCK, lock) != -1)
                ? SERVO_WRITE_OK : SERVO_WRITE_BUS_FAILED;
            width = 1;
        }

        // Every operation ends with a fresh read, so the caller always gets the
        // servo's actual state rather than what we hoped we wrote.
        const int n = b.st.Read(id, 0, b.mem_req_buf.bytes, sizeof(MemoryStruct));
        read_ok = (n == (int)sizeof(MemoryStruct));

        if (write_result == SERVO_WRITE_OK) {
            if (!read_ok) {
                write_result = SERVO_WRITE_MISMATCH;   // cannot prove it landed
            } else {
                const uint8_t vaddr = (op == SERVO_SERVICE_OP_WRITE) ? b.mem_req_addr : SBS_LOCK;
                const int16_t want  = (op == SERVO_SERVICE_OP_WRITE)
                    ? b.mem_req_value
                    : (int16_t)((op == SERVO_SERVICE_OP_UNLOCK)
                                ? SERVO_EPROM_UNLOCKED : SERVO_EPROM_LOCKED);
                if (registerFromBuffer(&b.mem_req_buf, vaddr, width) != (int32_t)want) {
                    write_result = SERVO_WRITE_MISMATCH;
                }
            }
        }

        // The angle limits are cached at boot and used to clamp every commanded
        // position; without this a limit change would not take effect until the
        // board was reset.
        if (write_result == SERVO_WRITE_OK && read_ok &&
            (b.mem_req_addr == 0x09 || b.mem_req_addr == 0x0B)) {
            const int16_t lo = b.mem_req_buf.memory.minAngleLimit;
            const int16_t hi = b.mem_req_buf.memory.maxAngleLimit;
            if (hi > lo) {
                b.minAngleCount[id - 1] = lo;
                b.maxAngleCount[id - 1] = hi;
            }
        }
    }

    b.mem_req_ok           = read_ok;
    b.mem_req_write_result = write_result;
    b.mem_req_width        = width;
    // Publish the buffer and the results before releasing Core 1, or it can
    // observe the handshake ahead of the data it is waiting for.
    __sync_synchronize();
    b.mem_req_pending = false;
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
