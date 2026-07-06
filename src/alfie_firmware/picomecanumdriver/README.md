# picomeccanumdriver

micro-ROS firmware for the Alfiebot mecanum drive base.

## Hardware

- **MCU board:** Waveshare RP2040-Zero (`board = waveshare_rp2040_zero`)
- **Motor driver:** Hiwonder 4-channel encoder motor driver over I2C
  - I2C0: **GP12 = SDA**, **GP13 = SCL**, 400 kHz, address `0x34`
  - The controller runs its own per-channel closed-loop velocity PID; the RP2040
    writes target speeds (pulses/10 ms, reg `51`) and reads accumulated encoder
    counts (reg `60`).
  - Motor type: JGB37-520 (`MOTOR_TYPE = 3`)
- **Status LED:** single onboard WS2812 RGB on GP16 (Adafruit NeoPixel)

## Software architecture

Dual-core (earlephilhower Arduino-Pico core):

- **Core 0** (`setup()`/`loop()`): peripheral management — reads encoders, runs
  inverse mecanum kinematics, pushes closed-loop speed targets to the Hiwonder
  controller over I2C, integrates odometry, and drives the WS2812 status LED.
- **Core 1** (`setup1()`/`loop1()`): micro-ROS communications and the
  agent-reconnect state machine.

The I2C bus to the Hiwonder controller is owned exclusively by Core 0; Core 1
only exchanges data through the shared (`volatile`) velocity command / odometry.

## ROS 2 interface

- **Node:** `mecanum_drive_controller`
- **Subscribes:** `/mecanumdrive` (`geometry_msgs/msg/Twist`) — body velocity
  command (`linear.x`, `linear.y` in m/s; `angular.z` in rad/s)
- **Publishes:** `/odom` (`nav_msgs/msg/Odometry`)
- **Transport:** micro-ROS serial. The agent runs on `/dev/ttyAlfieD` at
  `1500000` baud (see `alfie_bringup`).
- Wheel order everywhere is **[FL, FR, RL, RR]**.
- **Watchdog:** if no command arrives for `WATCHDOG_TIMEOUT_MS` (500 ms) the
  velocity command is zeroed and the wheels stop.

## Calibration (must be done on hardware)

- **`COUNTS_PER_WHEEL_REV`** (`include/config.h`): encoder counts reported by
  reg `60` per full wheel-output revolution. Ties commanded speed and odometry
  together and sets top-speed headroom against the ±100 pulses/10 ms register
  limit. Start from the estimate, then measure (spin a wheel N turns, read the
  count delta).
- **Channel map & direction** (`src/hiwonder_driver.cpp`): `MOTOR_CHANNEL_MAP`
  and `MOTOR_DIR_SIGN` map logical [FL, FR, RL, RR] onto the physical Hiwonder
  channels and fix each wheel's spin/count direction. `HIWONDER_ENCODER_POLARITY`
  in `config.h` flips the controller's global encoder sign.

## Build / flash

```
pio run -e waveshare_rp2040_zero        # build
pio run -e waveshare_rp2040_zero -t upload   # flash (BOOTSEL)
```

`fix_atomic.py` (pre-build) strips micro-ROS's conflicting 64-bit atomic object;
`reboot_after_upload.py` (post-upload) jumps the board out of BOOTSEL into the app.

> Serial is the micro-ROS transport — **no `Serial.print` debugging**.

## References

- https://arduino-pico.readthedocs.io/en/latest/platformio.html
- https://docs.platformio.org/en/latest/core/installation/udev-rules.html
