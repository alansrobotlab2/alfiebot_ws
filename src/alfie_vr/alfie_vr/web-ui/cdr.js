// CDR Decoding Module
// Minimal little-endian ROS 2 CDR reader plus decoders for the messages we subscribe to.
// Alignment is relative to the start of the payload (after the 4-byte encapsulation header).

const textDecoder = new TextDecoder();

export class CdrReader {
    /**
     * @param {Uint8Array} bytes - Full CDR message including the 4-byte encapsulation header
     */
    constructor(bytes) {
        this.bytes = bytes;
        this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
        this.offset = 4;  // Skip CDR encapsulation header (00 01 00 00 = little-endian)
    }

    align(n) {
        const rel = this.offset - 4;
        const pad = (n - (rel % n)) % n;
        this.offset += pad;
    }

    uint8() {
        const v = this.view.getUint8(this.offset);
        this.offset += 1;
        return v;
    }

    uint32() {
        this.align(4);
        const v = this.view.getUint32(this.offset, true);
        this.offset += 4;
        return v;
    }

    int32() {
        this.align(4);
        const v = this.view.getInt32(this.offset, true);
        this.offset += 4;
        return v;
    }

    float64() {
        this.align(8);
        const v = this.view.getFloat64(this.offset, true);
        this.offset += 8;
        return v;
    }

    // string: uint32 length (includes null terminator) + UTF-8 bytes
    string() {
        const len = this.uint32();
        const strBytes = this.bytes.subarray(this.offset, this.offset + (len > 0 ? len - 1 : 0));
        this.offset += len;
        return textDecoder.decode(strBytes);
    }

    // uint8[]: uint32 length + raw bytes (returned as a view, no copy)
    byteSequence() {
        const len = this.uint32();
        const data = this.bytes.subarray(this.offset, this.offset + len);
        this.offset += len;
        return data;
    }

    // builtin_interfaces/Time
    time() {
        const sec = this.int32();
        const nanosec = this.uint32();
        return { sec, nanosec };
    }
}

// ========================================
// Message Decoders
// ========================================

// std_msgs/msg/String
export function parseStdString(bytes) {
    return new CdrReader(bytes).string();
}

// sensor_msgs/msg/CompressedImage -> { frameId, format, data }
export function parseCompressedImage(bytes) {
    const r = new CdrReader(bytes);
    r.time();                    // header.stamp
    const frameId = r.string();  // header.frame_id
    const format = r.string();
    const data = r.byteSequence();
    return { frameId, format, data };
}

// tf2_msgs/msg/TFMessage -> [{ frameId, childFrameId, position, quaternion }]
export function parseTFMessage(bytes) {
    const r = new CdrReader(bytes);
    const count = r.uint32();
    const transforms = new Array(count);
    for (let i = 0; i < count; i++) {
        r.time();                         // header.stamp
        const frameId = r.string();       // header.frame_id
        const childFrameId = r.string();
        // Transform: translation (Vector3 float64), rotation (Quaternion float64)
        const position = { x: r.float64(), y: r.float64(), z: r.float64() };
        const quaternion = { x: r.float64(), y: r.float64(), z: r.float64(), w: r.float64() };
        transforms[i] = { frameId, childFrameId, position, quaternion };
    }
    return transforms;
}

// rcl_interfaces/msg/Log
// Log level constants: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50
export const LOG_LEVELS = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL'
};

export function parseRosoutLog(bytes) {
    const r = new CdrReader(bytes);
    const stamp = r.time();
    const level = r.uint8();
    const name = r.string();   // logger name
    const msg = r.string();
    const file = r.string();
    const func = r.string();
    const line = r.uint32();
    return {
        timestamp: stamp.sec + stamp.nanosec / 1e9,
        level: level,
        levelName: LOG_LEVELS[level] || `L${level}`,
        name: name,
        msg: msg,
        file: file,
        function: func,
        line: line
    };
}
