#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// --- Heartbeat LED (from zero_heartbeat) ------------------------------------
// The RP2040-Zero's only onboard LED is a single WS2812 RGB on GPIO16.
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL 16
#endif
static Adafruit_NeoPixel led(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Heartbeat: 1 = on, 0 = off. 100ms per segment -> 800ms loop (double blink).
static const uint8_t heartbeat_pattern[] = {1, 0, 1, 0, 0, 0, 0, 0};
static const uint8_t pattern_len = sizeof(heartbeat_pattern) / sizeof(heartbeat_pattern[0]);
static const uint16_t segment_ms = 100;

// Non-blocking heartbeat: advances one pattern segment every segment_ms so the
// LED keeps beating while the bus scanner runs.
static void updateHeartbeat() {
  static uint32_t lastStep = 0;
  static uint8_t step = 0;
  static int8_t shown = -1;  // last segment value written to the LED (-1 = none)
  if (millis() - lastStep >= segment_ms) {
    lastStep = millis();
    step = (step + 1) % pattern_len;
  }
  if (heartbeat_pattern[step] != shown) {
    shown = heartbeat_pattern[step];
    led.setPixelColor(0, shown ? led.Color(0, 255, 0) : 0);
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
static const uint8_t BUS_TX_PIN = 12;
static const uint8_t BUS_RX_PIN = 13;
static const uint32_t BUS_BAUD = 1000000;  // 1 Mbaud

// STS/SCS protocol constants
static const uint8_t PKT_HEADER = 0xFF;
static const uint8_t INST_PING = 0x01;
static const uint8_t MAX_SERVO_ID = 253;        // 254 (0xFE) is broadcast
static const uint32_t REPLY_TIMEOUT_US = 4000;  // per-ID wait for a status reply

// Send a PING and return true if the addressed servo replies with its own ID.
static bool pingServo(uint8_t id) {
  // Drain any stale bytes before issuing the ping.
  while (Serial1.available()) Serial1.read();

  // Packet: FF FF ID LEN(0x02) INST(PING) CHK
  uint8_t chk = ~(id + 0x02 + INST_PING) & 0xFF;
  uint8_t pkt[6] = {PKT_HEADER, PKT_HEADER, id, 0x02, INST_PING, chk};
  Serial1.write(pkt, sizeof(pkt));
  Serial1.flush();  // block until the packet has left the TX FIFO

  // Expected reply: FF FF ID LEN ERR CHK (6 bytes)
  uint8_t resp[6];
  uint8_t got = 0;
  uint32_t start = micros();
  while (got < sizeof(resp)) {
    if (Serial1.available()) {
      resp[got++] = Serial1.read();
      start = micros();  // refresh timeout as bytes trickle in
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

static void scanBus() {
  Serial.println();
  Serial.println(F("Scanning bus for servos (IDs 0-253)..."));

  bool found[MAX_SERVO_ID + 1] = {false};
  uint8_t count = 0;
  for (uint8_t id = 0; id <= MAX_SERVO_ID; id++) {
    updateHeartbeat();  // keep the LED beating during the sweep
    if (pingServo(id)) {
      found[id] = true;
      count++;
    }
  }

  // Address map: 16-column grid, "." = no reply, "##" = servo present.
  Serial.println(F("Address map (rows of 16):"));
  Serial.print(F("      "));
  for (uint8_t c = 0; c < 16; c++) {
    Serial.printf("%02X ", c);
  }
  Serial.println();
  for (uint16_t base = 0; base <= MAX_SERVO_ID; base += 16) {
    Serial.printf("%3u:  ", base);
    for (uint8_t c = 0; c < 16; c++) {
      uint16_t id = base + c;
      if (id > MAX_SERVO_ID) break;
      Serial.print(found[id] ? "## " : " . ");
    }
    Serial.println();
  }

  Serial.printf("Done. %u servo(s) found.\n", count);
}

void setup() {
  led.begin();
  led.setBrightness(64);  // WS2812 green at full brightness is very bright
  led.clear();
  led.show();

  Serial.begin(115200);  // USB serial for debug / monitor
  while (!Serial && millis() < 3000) {
    // wait briefly for USB CDC so we don't miss the first output
  }

  Serial1.setTX(BUS_TX_PIN);
  Serial1.setRX(BUS_RX_PIN);
  Serial1.begin(BUS_BAUD);

  Serial.println(F("Waveshare bus servo scanner ready."));
}

void loop() {
  static uint32_t lastScan = 0;
  const uint32_t scan_interval_ms = 5000;  // re-scan every 5s

  updateHeartbeat();

  if (millis() - lastScan >= scan_interval_ms) {
    lastScan = millis();
    scanBus();
  }
}
  
  
