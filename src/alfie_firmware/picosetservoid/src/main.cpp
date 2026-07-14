#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// --- Servo to initialize -----------------------------------------------------
// Connect ONE servo to the bus and set its current ID here. This sketch writes
// a known-good starting configuration to that servo:
//   offset    = 0
//   min angle = 0
//   max angle = 4095
//   position  = 2048   (center)
// then reads all four back and confirms them before reporting success.
#define SERVO_ID 4

// --- Status LED -------------------------------------------------------------
// The RP2040-Zero's only onboard LED is a single WS2812 RGB on GPIO16.
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 16
#endif
static Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Two status modes:
//   SOLID  - steady green: during setup/init, and on success (all 4 confirmed).
//   BLINK  - 100ms on / 100ms off green: error or a value that didn't read back.
enum LedMode { LED_SOLID, LED_BLINK };
static LedMode ledMode = LED_SOLID;
static const uint16_t blink_ms = 100;  // half-period of the error blink

static void setLedMode(LedMode mode) { ledMode = mode; }

// Non-blocking LED driver: call frequently. Holds the pixel solid in SOLID mode
// and toggles it every blink_ms in BLINK mode.
static void updateLed() {
  static uint32_t lastToggle = 0;
  static bool blinkOn = false;
  static int8_t shown = -1;  // last value written (-1 = none, forces first write)

  bool on;
  if (ledMode == LED_BLINK) {
    if (millis() - lastToggle >= blink_ms) {
      lastToggle = millis();
      blinkOn = !blinkOn;
    }
    on = blinkOn;
  } else {
    on = true;  // SOLID
  }

  if (on != shown) {
    shown = on;
    led.setPixelColor(0, on ? led.Color(0, 255, 0) : 0);
    led.show();
  }
}

// Waveshare Serial Bus Servo Driver board / Bus Servo Adapter (A) on UART0.
// Waveshare documents a STRAIGHT-THROUGH connection (RX-RX, TX-TX by label):
//   Pico GPIO0 (TX) -> driver board TXD
//   Pico GPIO1 (RX) -> driver board RXD
//   common GND, and the board's mode jumper in the "A" (UART) position.
// The adapter is a transparent passthrough, so the host runs at the servos'
// native baud. Servos speak the Feetech STS/SCS protocol (ST3215 et al.),
// 1 Mbaud / 8N1 default.
static const uint8_t BUS_TX_PIN = 0;
static const uint8_t BUS_RX_PIN = 1;
static const uint32_t BUS_BAUD = 1000000;  // 1 Mbaud

// STS/SCS protocol constants
static const uint8_t PKT_HEADER = 0xFF;
static const uint8_t INST_PING = 0x01;
static const uint8_t INST_READ = 0x02;
static const uint8_t INST_WRITE = 0x03;
static const uint32_t REPLY_TIMEOUT_US = 4000;  // per-ID wait for a status reply

// STS/SCS register addresses. Offset and the angle limits live in EEPROM (must
// unlock to write); goal position is in RAM. STS words are little-endian.
static const uint8_t REG_MIN_ANGLE = 0x09;  // min angle limit, 2 bytes
static const uint8_t REG_MAX_ANGLE = 0x0B;  // max angle limit, 2 bytes
static const uint8_t REG_OFFSET = 0x1F;     // position correction / offset, 2 bytes
static const uint8_t REG_TORQUE_ENABLE = 0x28;  // torque on/off (RAM), 1 byte
static const uint8_t REG_GOAL_POS = 0x2A;   // goal position, 2 bytes
static const uint8_t REG_LOCK = 0x37;       // EEPROM lock: 0 = unlocked, 1 = locked

// Read a 6-byte status reply (FF FF ID LEN ERR CHK) and validate it.
static bool readStatusAck(uint8_t id) {
  uint8_t resp[6];
  uint8_t got = 0;
  uint32_t start = micros();
  while (got < sizeof(resp)) {
    if (Serial1.available()) {
      resp[got++] = Serial1.read();
      start = micros();
    } else if (micros() - start > REPLY_TIMEOUT_US) {
      break;
    }
  }
  if (got < 6) return false;
  if (resp[0] != PKT_HEADER || resp[1] != PKT_HEADER) return false;
  if (resp[2] != id) return false;
  uint8_t rchk = ~(resp[2] + resp[3] + resp[4]) & 0xFF;
  return rchk == resp[5];
}

// Send a PING and return true if the addressed servo replies with its own ID.
static bool pingServo(uint8_t id) {
  while (Serial1.available()) Serial1.read();  // drain stale bytes

  // Packet: FF FF ID LEN(0x02) INST(PING) CHK
  uint8_t chk = ~(id + 0x02 + INST_PING) & 0xFF;
  uint8_t pkt[6] = {PKT_HEADER, PKT_HEADER, id, 0x02, INST_PING, chk};
  Serial1.write(pkt, sizeof(pkt));
  Serial1.flush();  // block until the packet has left the TX FIFO

  return readStatusAck(id);
}

// Write a single byte to a servo register and wait for the status ack.
// Packet: FF FF ID LEN(0x04) INST(WRITE) ADDR DATA CHK
static bool writeReg(uint8_t id, uint8_t addr, uint8_t value) {
  while (Serial1.available()) Serial1.read();  // drain stale bytes

  uint8_t len = 0x04;  // INST + ADDR + DATA + (LEN counts params + 2)
  uint8_t chk = ~(id + len + INST_WRITE + addr + value) & 0xFF;
  uint8_t full[8] = {PKT_HEADER, PKT_HEADER, id, len, INST_WRITE, addr, value, chk};
  Serial1.write(full, sizeof(full));
  Serial1.flush();

  return readStatusAck(id);
}

// Write a 16-bit word (little-endian, STS order) to a register and wait for ack.
// Packet: FF FF ID LEN(0x05) INST(WRITE) ADDR DATA_L DATA_H CHK
static bool writeReg16(uint8_t id, uint8_t addr, uint16_t value) {
  while (Serial1.available()) Serial1.read();  // drain stale bytes

  uint8_t lo = value & 0xFF;
  uint8_t hi = (value >> 8) & 0xFF;
  uint8_t len = 0x05;  // INST + ADDR + 2 DATA + (LEN counts params + 2)
  uint8_t chk = ~(id + len + INST_WRITE + addr + lo + hi) & 0xFF;
  uint8_t full[9] = {PKT_HEADER, PKT_HEADER, id, len, INST_WRITE, addr, lo, hi, chk};
  Serial1.write(full, sizeof(full));
  Serial1.flush();

  return readStatusAck(id);
}

// Read a 16-bit word from a register. Returns true and fills *out on success.
// Request:  FF FF ID LEN(0x04) INST(READ) ADDR READLEN(0x02) CHK
// Response: FF FF ID LEN(0x04) ERR DATA_L DATA_H CHK
static bool readReg16(uint8_t id, uint8_t addr, uint16_t *out) {
  while (Serial1.available()) Serial1.read();  // drain stale bytes

  uint8_t len = 0x04;
  uint8_t readLen = 0x02;
  uint8_t chk = ~(id + len + INST_READ + addr + readLen) & 0xFF;
  uint8_t pkt[8] = {PKT_HEADER, PKT_HEADER, id, len, INST_READ, addr, readLen, chk};
  Serial1.write(pkt, sizeof(pkt));
  Serial1.flush();

  uint8_t resp[8];
  uint8_t got = 0;
  uint32_t start = micros();
  while (got < sizeof(resp)) {
    if (Serial1.available()) {
      resp[got++] = Serial1.read();
      start = micros();
    } else if (micros() - start > REPLY_TIMEOUT_US) {
      break;
    }
  }
  if (got < 8) return false;
  if (resp[0] != PKT_HEADER || resp[1] != PKT_HEADER) return false;
  if (resp[2] != id) return false;
  uint8_t rchk = ~(resp[2] + resp[3] + resp[4] + resp[5] + resp[6]) & 0xFF;
  if (rchk != resp[7]) return false;

  *out = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  return true;
}

// The four register values to write, then read back and confirm.
struct RegInit {
  uint8_t addr;
  uint16_t value;
  const char *name;
};
static const RegInit INIT_REGS[4] = {
    {REG_OFFSET, 0, "offset"},
    {REG_MIN_ANGLE, 0, "min angle"},
    {REG_MAX_ANGLE, 4095, "max angle"},
    {REG_GOAL_POS, 2048, "position"},
};

// Write the starting configuration to the servo, then read all four values back
// and confirm each matches. Returns true only if all four are confirmed.
static bool initializeServo(uint8_t id) {
  Serial.println();
  Serial.printf("Initializing servo ID %u\n", id);

  if (!pingServo(id)) {
    Serial.printf("  ERROR: no servo replied at ID %u. Check wiring/power.\n", id);
    return false;
  }
  Serial.printf("  Found servo at ID %u.\n", id);

  // Unlock EEPROM (offset + angle limits live there; goal position is RAM).
  if (!writeReg(id, REG_LOCK, 0)) {
    Serial.println(F("  ERROR: failed to unlock EEPROM."));
    return false;
  }

  for (const RegInit &r : INIT_REGS) {
    if (!writeReg16(id, r.addr, r.value)) {
      Serial.printf("  ERROR: failed to write %s.\n", r.name);
      writeReg(id, REG_LOCK, 1);  // attempt to re-lock before bailing
      return false;
    }
  }

  // Re-lock EEPROM.
  if (!writeReg(id, REG_LOCK, 1)) {
    Serial.println(F("  WARNING: values written but failed to re-lock EEPROM."));
  }

  // Enable torque so the servo actually drives to the goal position. The goal
  // register holds 2048 either way, but without torque the servo never moves.
  if (!writeReg(id, REG_TORQUE_ENABLE, 1)) {
    Serial.println(F("  ERROR: failed to enable torque."));
    return false;
  }

  // Read back and confirm all four values.
  for (const RegInit &r : INIT_REGS) {
    uint16_t v = 0;
    if (!readReg16(id, r.addr, &v)) {
      Serial.printf("  ERROR: failed to read back %s.\n", r.name);
      return false;
    }
    if (v != r.value) {
      Serial.printf("  ERROR: %s read back %u, expected %u.\n", r.name, v, r.value);
      return false;
    }
    Serial.printf("  %s = %u confirmed.\n", r.name, v);
  }

  Serial.printf("  SUCCESS: all 4 values confirmed on servo %u.\n", id);
  return true;
}

void setup() {
  led.begin();
  led.setBrightness(64);  // WS2812 green at full brightness is very bright
  led.clear();
  led.show();
  setLedMode(LED_SOLID);  // solid green through setup and init

  Serial.begin(115200);  // USB serial for debug / monitor
  while (!Serial && millis() < 3000) {
    // wait briefly for USB CDC so we don't miss the first output
  }

  Serial1.setTX(BUS_TX_PIN);
  Serial1.setRX(BUS_RX_PIN);
  Serial1.begin(BUS_BAUD);

  Serial.println(F("Waveshare bus servo initializer ready."));
}

void loop() {
  static bool done = false;
  static uint32_t startMs = 0;
  if (startMs == 0) startMs = millis();

  updateLed();

  // Run the one-shot init sequence shortly after boot, then idle.
  if (!done && millis() - startMs >= 1000) {
    done = true;
    bool ok = initializeServo(SERVO_ID);
    Serial.println(ok ? F("Done. You may disconnect the servo.")
                      : F("Failed. Fix the issue and re-flash to retry."));
    // Solid = success (all 4 read back and confirmed); blink = error.
    setLedMode(ok ? LED_SOLID : LED_BLINK);
  }
}
