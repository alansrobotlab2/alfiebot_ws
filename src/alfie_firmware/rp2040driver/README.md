# RP2040 I2C Slave Driver

This firmware configures a Waveshare RP2040-Zero as an I2C slave device for controlling eye LEDs and monitoring sensors on a robot.

## Hardware Configuration

### Board
- **Waveshare RP2040-Zero** (RP2040 derivative)
- Connected as I2C slave at address **0x58**
- I2C Bus: SDA=GPIO4, SCL=GPIO5

### Peripherals

| Peripheral | Pins | Description |
|------------|------|-------------|
| WS2812B RGB LED | GPIO 16 | Status indicator (heartbeat) |
| Left Eye PWM | GPIO 6 (PWMA) | PWM output for left eye LED |
| Left Eye Direction | GPIO 7 (AIN1, HIGH)<br>GPIO 8 (AIN2, LOW) | Motor driver direction pins |
| Right Eye PWM | GPIO 9 (PWMB) | PWM output for right eye LED |
| Right Eye Direction | GPIO 10 (BIN1, LOW)<br>GPIO 11 (BIN2, HIGH) | Motor driver direction pins |
| Back Switch | GPIO 12 | Input with pullup (LOW=pressed) |
| Temperature Sensor | ADC4 (internal) | RP2040 internal temperature sensor |

## I2C Communication Protocol

### I2C Address
- **0x58** (88 decimal)

### Expected Communication Rate
- **100 Hz** (10ms period)
- Watchdog timeout if no I2C activity for >100ms

### Status LED Behavior
- **Green heartbeat** (double pulse): I2C communication healthy
- **Red heartbeat** (double pulse): I2C timeout or no communication

---

## Command Messages (I2C Write)

### Command 0x01: Set Eye PWM Values

Sets the PWM duty cycle for both eye LEDs.

**Format:**
```
Byte 0: 0x01 (command)
Byte 1: Left eye PWM (0-255)
Byte 2: Right eye PWM (0-255)
```

**Example:**
```bash
# Set left eye to 50% (128), right eye to 78% (200)
i2cset -y 1 0x58 0x01 128 200 i
```

**PWM Output:**
- 0 = Off (0% duty cycle)
- 255 = Full brightness (100% duty cycle)
- 8-bit resolution (0-255)

---

### Command 0x02: Request Status

Requests a status packet. The status packet will be returned on the next I2C read operation.

**Format:**
```
Byte 0: 0x02 (command)
```

**Example:**
```bash
# Request status
i2cset -y 1 0x58 0x02
```

---

## Status Response (I2C Read)

### Status Packet Format

After sending command 0x02, perform an I2C read to retrieve the 5-byte status packet.

**Format:**
```
Byte 0: Left eye PWM value (0-255)
Byte 1: Right eye PWM value (0-255)
Byte 2: Back switch state (0=not pressed, 1=pressed)
Byte 3: Temperature high byte (signed 16-bit, °C × 100)
Byte 4: Temperature low byte (signed 16-bit, °C × 100)
```

**Example:**
```bash
# Request and read status
i2cset -y 1 0x58 0x02           # Request status
i2cget -y 1 0x58                # Read first byte
# Continue reading remaining 4 bytes
```

**Temperature Decoding:**

The temperature is a signed 16-bit integer representing degrees Celsius × 100.

```python
# Example: Reading temperature from status packet
temp_high = status_packet[3]
temp_low = status_packet[4]
temp_raw = (temp_high << 8) | temp_low
temp_celsius = temp_raw / 100.0

# Example values:
# 2550 (0x09F6) = 25.50°C
# 3042 (0x0BE2) = 30.42°C
# -500 (0xFE0C) = -5.00°C
```

---

## Development Setup

### Prerequisites

Make sure you have the following installed:
```bash
sudo apt-get install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

### PlatformIO Configuration

Edit `~/.platformio/platforms/raspberrypi/platform.json`:
```json
"toolchain-gccarmnoneeabi": {
  "type": "toolchain",
  "owner": "platformio",
  "version": "~1.90301.0"
}
```

### Building

```bash
cd /path/to/rp2040driver
pio run                    # Build
pio run --target upload    # Upload to board
pio device monitor         # View serial output
```

---

## References

- [Waveshare RP2040-Zero Wiki](https://www.waveshare.com/wiki/RP2040-Zero)
- [PlatformIO RP2040 Documentation](https://docs.platformio.org/en/latest/boards/raspberrypi/pico.html)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)


