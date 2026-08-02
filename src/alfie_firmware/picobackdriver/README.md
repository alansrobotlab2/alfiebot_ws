# Pico Back Driver Board

Firmware for the Alfie robot's back lift — a belt-driven linear actuator on a
Waveshare RP2040 Zero, driven by a DRV8876 H-bridge and controlled over
micro-ROS.

Gen2 hardware. The gen1 TB6612FNG is gone; so is most of what the previous
revision of this document said about it.

---

## Hardware

| | |
|---|---|
| **MCU** | Waveshare RP2040 Zero (serial `5303284740EC7D9C` → `/dev/ttyAlfieB`) |
| **Driver** | DRV8876 (3.5 A, 44.3 °C/W, integrated current sense + fault reporting) |
| **Motor** | JGB37-520B, **90:1**, 110 rpm @ 12 V, 11 PPR quadrature encoder |
| **Transmission** | 20T 2GT pulley (2 mm pitch, 40 mm circumference), GT2 belt |
| **IMU** | BNO085 on I2C0 |
| **Status LED** | WS2812B on GP16 |
| **Comms** | micro-ROS over USB CDC @ 1,000,000 baud |

### Pin map

| GPIO | Signal | Notes |
|---|---|---|
| GP28 | `EN` | PWM, 24 kHz |
| GP27 | `PH` | direction — `MOTOR_PH_UP = HIGH` |
| GP26 | `PMODE` | driven **LOW** for PH/EN mode. Never leave floating |
| GP15 | `nSLEEP` | HIGH = active |
| GP7 | `nFAULT` | open-drain input, internal pull-up |
| GP3 / GP2 | `ENC_A` / `ENC_B` | quadrature, both on CHANGE |
| GP1 | `LIMIT_SW` | bottom limit, **normally closed → HIGH = triggered** |
| GP12 / GP13 | I2C0 SDA / SCL | BNO085 |
| GP11 | IMU reset | |
| GP16 | WS2812B | |

Free: GP0, GP4, GP5, GP6, GP8, GP9, GP10, GP14, GP17–22.

### DRV8876 configuration

| Pin | State | Why |
|---|---|---|
| `PMODE` | driven LOW from GP26 | PH/EN mode. Floating = independent half-bridge, which **inverts the down duty** on a gravity-loaded axis |
| `IMODE` | 20 kΩ to GND (level 2) | Only the overcurrent response still matters: **auto-retry**, not latch-off |
| `VREF` | tied to 3.3 V | Only needs to be above GND now |
| `IPROPI` / `CS` | **strapped to GND** | Disables internal current regulation, datasheet §7.3.3.2 |

**Current regulation is deliberately disabled.** The ITRIP chopper was clamping
below the current required to lift, so the actuator could not rise at any duty.
Overcurrent protection (3.5–5.5 A, 2 ms auto-retry) and thermal shutdown are
separate silicon and remain active — the part is still self-protecting.

With the chopper gone, **`VELOCITY_PID_OUTPUT_LIMIT` is the only thing bounding
motor current.** See `config.h`, "WHY THERE IS NO CURRENT TRIP".

---

## Calibrated constants

Measured on hardware, not derived. Everything downstream depends on these.

```
GEAR_RATIO              90.0        90:1, confirmed against 34685 counts = 350.00 mm
ENCODER_COUNTS_PER_METER 99,000     = 11 PPR × 4 × 90 / 0.040 m
counts per mm           99.0
ACTUATOR_MAX_POSITION   0.350 m     real stroke from the limit switch
```

Two independent checks agree: the count calibration lands within 0.1% of the
geometric model, and the measured voltage-speed slope (5.82 mm/s per volt) is
within 5% of the theoretical 6.11 for 110 rpm through a 40 mm pulley.

### Measured performance

| duty | speed (up) | 350 mm | stall current |
|---|---|---|---|
| 160 | 27.7 mm/s | 12.6 s | 2.28 A |
| 200 | 46.1 mm/s | 7.7 s | 2.85 A |
| **220** ← ceiling | **51.5 mm/s** | **6.9 s** | 3.14 A |
| 245 | 51.9 mm/s | 6.7 s | 3.50 A ← OCP floor |

Descent is gravity-assisted and ~35% faster: 69.7 mm/s at duty 220.

Running current is **~0.45 A** (from the voltage-speed intercept) — well under
the motor's 1.0 A continuous rating, and ~0.14 W in the driver. The drive is
lightly loaded.

The duty ceiling of 220 is set by **OCP margin**, not thermals: a dead stall
draws 3.14 A, 10% under the DRV8876's 3.5 A trip point, so a jam is answered by
the firmware stall detector rather than by driver retry chatter. 245 is the hard
ceiling on the ceiling.

---

## Control architecture

Dual-core, cascaded control.

- **Core 0** — peripherals: motor, encoder ISRs, IMU, limit switch, LED. Control
  loop at **500 Hz** (`CONTROL_LOOP_PERIOD_MS = 2`).
- **Core 1** — micro-ROS state machine at a drift-free **50 Hz**.

```
BackCmd.position ──► position error ──► × POSITION_KP (5.0) ──► desired velocity
                                                                      │
        BackCmd.velocity (a LIMIT, not a command) ──► clamp ◄──────────┘
                                                     │
                            MAX_ACTUATOR_ACCELERATION ▼ ramp
                                                     │
                     encoder velocity ──► applyVelocityPID ──► PWM ──► DRV8876
```

**`BackCmd.velocity` is a maximum, not a setpoint.** Position is the command;
velocity and acceleration are constraints on how it gets there.

`POSITION_KP` is currently hardcoded in `motor_control.cpp`, not exposed in
`config.h`.

### Gains and limits

```
VELOCITY_PID_KP             2500.0    PWM counts per m/s of error
VELOCITY_PID_KI             3000.0    supplies the gravity-holding bias
VELOCITY_PID_KD             0.0       disabled
VELOCITY_PID_OUTPUT_LIMIT   220       duty ceiling = current ceiling
VELOCITY_PID_INTEGRAL_LIMIT 0.045
MAX_ACTUATOR_VELOCITY       0.046 m/s
MAX_ACTUATOR_ACCELERATION   0.35 m/s²
MIN_PWM_UPWARD              100       floor on any non-zero output
MIN_PWM_DOWNWARD            20        gravity assists
POSITION_TOLERANCE_M        0.002
```

⚠️ `MIN_PWM_UPWARD = 100` already produces ~18.7 mm/s. Against a 46 mm/s cap
that leaves a **2.5:1 controllable range** — commanded velocities below the
floor make the loop chatter between zero and 18.7 mm/s.

---

## Safety

| Detector | Trips on | Recovery |
|---|---|---|
| **Stall** | duty above `STALL_PWM_THRESHOLD` with no motion for 250 ms | Classified: `MOTOR_BLOCKED` (was moving, then stopped) hard-latches immediately; `LOAD_EXCEEDED` (never broke away) retries 3× at 3 s intervals then latches |
| **Runaway** | over 0.150 m/s, or motion against the command for 150 ms | Latches until reset — miswiring cannot be retried away |
| **nFAULT** | debounced low | Real device fault only (OCP/TSD/UVLO); chopping is disabled. Re-arms via nSLEEP cycle, rate-limited to 1/s |
| **Calibration** | see below | Own guards, because the usual two cannot fire on that path |

Runaway hard-latches because it means the hardware is not what the firmware
believes it to be — and on a gravity-loaded axis a sign inversion becomes
positive feedback.

### Error codes (`BackState.error_code`)

| | |
|---|---|
| `0x00` | none |
| `0x02` | motor driver fault |
| `0x08` | `DRIVER_FAULT` — nFAULT asserted |
| `0x09` | `MOTOR_BLOCKED` — obstruction, hard latched |
| `0x0A` | `LOAD_EXCEEDED` — never broke away, retrying |
| `0x0B` | `MOTOR_RUNAWAY` — overspeed or wrong direction, latched |

`stall_position` and `stall_count` are published alongside. A real obstruction
stalls at the **same** `stall_position` across retries; an overload stalls
wherever it happened to start.

---

## Status LED (WS2812B)

Faults take priority over everything else.

| Colour | State |
|---|---|
| **Fast red strobe** (80 ms) | Runaway — most severe, needs a reset |
| **Solid red** | Obstruction hard-latched, or driver fault |
| **Blinking red** (250 ms) | Overload, retrying on its own |
| **Purple** (pulsing) | Calibration in progress |
| **Dim blue** | Waiting for micro-ROS agent |
| **Yellow** | Agent connected, creating entities |
| **Cyan** | Moving |
| **Green heartbeat** | Operational and idle |
| **Orange** | Agent disconnected |
| **Dim white** | Unknown state |

Solid vs blinking red is the distinction that matters when you are looking at
the robot rather than at a topic: solid needs a human, blinking will retry.

---

## ROS 2 interface

Node `back_drive_controller` in namespace **`/alfie/low`**.

### Topics

| Topic | Type | Dir | QoS | Rate |
|---|---|---|---|---|
| `/alfie/low/backcmd` | `alfie_msgs/BackCmd` | sub | Best Effort | on demand |
| `/alfie/low/backstate` | `alfie_msgs/BackState` | pub | Best Effort | 50 Hz |
| `/alfie/low/neck_power` | `std_msgs/Empty` | sub | Best Effort | heartbeat |

**Best Effort throughout** — for real-time motor control, stale data is worse
than a dropped message. A `RELIABLE` subscriber on the host side will simply
never match and will sit silent.

`BackState` carries position, velocity, acceleration, encoder pulses, PWM
output, limit switch state, board temperature, calibration status, the fault
fields above, and the BNO085 IMU data.

### Service

| Service | Type | QoS |
|---|---|---|
| `/alfie/low/calibrate_back` | `alfie_msgs/srv/BackRequestCalibration` | Reliable |

---

## Calibration (homing)

Homes to the bottom limit switch and zeroes position. **Required before any
position command is meaningful** — without it, every position is measured
against an arbitrary origin.

```bash
ros2 service call /alfie/low/calibrate_back alfie_msgs/srv/BackRequestCalibration
```

**Sequence:**

1. Rejects the call if calibration is already running.
2. Blocks incoming `BackCmd` for the duration.
3. Reads the limit switch — **HIGH = triggered** (normally-closed).
4. If not already at the bottom, drives down at `VELOCITY_PID_OUTPUT_LIMIT`
   while polling the switch and the encoder.
5. On assertion: immediate brake, encoders reset, PID state cleared,
   `is_calibrated` set.

**This path carries its own guards**, because the usual ones cannot fire on it:
it drives the bridge directly with normal motor control suspended, so
`motor.pwm_output` is stale — runaway detection is skipped explicitly, and stall
detection silently never triggers.

| Guard | Threshold | Result |
|---|---|---|
| Stall | no 20 counts of progress within 400 ms | abort, `success: false` |
| Reversal | 200 counts *upward* while commanded down | latches `runaway_detected` — this is a wiring fault and must not be retried |
| Timeout | 20 s | abort, `success: false` |

Before these existed, a failed homing run drove into the bottom stop for the
full 20 s at the duty ceiling with no current limit behind it.

---

## Building and flashing

```bash
~/.platformio/penv/bin/pio run -e back              # build
~/.platformio/penv/bin/pio run -e back -t upload    # flash
```

After changing a `.msg`, clean the micro-ROS build first:

```bash
~/.platformio/penv/bin/pio run -e back -t clean_microros
```

Install the udev rule so the board appears as `/dev/ttyAlfieB`:

```bash
sudo cp ../99-alfie-pico.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

⚠️ **`pio run | tail` reports `tail`'s exit code.** A build can fail while the
shell reports success. Check the output, not the status.

---

## Bring-up tools

In [`bringup/`](bringup/) — not compiled, since PlatformIO only builds `src/`
and `lib/`. See [`bringup/README.md`](bringup/README.md).

- **`position_test.cpp`** — open-loop position moves and `GEAR_RATIO`
  calibration. This is what produced the constants above.
- **`pid_tune.py`** — velocity-PID step-response harness. Reports steady-state
  error, rise time, overshoot, settling, ripple, and time spent pinned at the
  duty ceiling or the PWM floor.

---

## Code structure

```
├── include/
│   ├── config.h              # All constants, heavily annotated with WHY
│   └── driverboard.h         # DriverBoard class and state structs
├── lib/
│   ├── motor_control/        # Motor, encoder, safety detectors, LED
│   ├── ros_interface/        # micro-ROS entities, calibration service
│   └── ws2812/               # PIO-based RGB LED driver
├── src/
│   └── main.cpp              # Dual-core entry points
├── bringup/                  # Hardware bring-up tools (not compiled)
└── extra_packages/
    └── alfie_msgs/           # Custom ROS 2 messages
```

`config.h` is the primary reference. It records the reasoning behind each
constant, including dead ends, so they are not re-explored.

---

## Known limitations

- **`ACTUATOR_MAX_POSITION` = 0.350 has almost no margin.** The measured stroke
  is 350 mm and **there is no top limit switch** — only the bottom is switched.
  Nothing stops an overrun except the stall detector, after it has been pushing
  into the stop. 0.340 would be cheap insurance.
- **`MIN_PWM_UPWARD` was measured with the broken encoder decoder** (see below)
  and may be conservative. Re-measuring could widen the 2.5:1 controllable speed
  range considerably.
- **The velocity PID is untuned** against the corrected scale. Gains predate the
  calibration.
- **`POSITION_KP` is hardcoded** in `motor_control.cpp` rather than in `config.h`.
- **No current measurement anywhere in the system** — IPROPI is grounded. Load is
  inferred from the voltage-speed relationship, not measured.

### Bugs fixed during gen2 bring-up

Recorded because each was live in the shipping firmware and none announced
itself:

- **Encoder decoder** counted ±1 on every interrupt from current pin levels with
  no state memory, losing ~1 count in 3 and drifting cumulatively (20 mm after
  one round trip, 32 mm after two). It corrupted *every* calibration this
  project ever took, including gen1's, and is the origin of the long-standing
  "unexplained 2.33× gap". Replaced with a Gray-code transition table.
- **Limit switch polarity** inverted at all three read sites, which made
  calibration skip its descent and zero wherever the actuator happened to be — a
  silent false datum with no error and no timeout.
- **`RUNAWAY_VELOCITY_MPS`** derived as `MAX_ACTUATOR_VELOCITY × 2`, which sat
  below real gravity-assisted descent speeds. Every normal descent would have
  latched an unrecoverable fault. Now an absolute constant.

---

## Coding standards

1. Descriptive names; comments explain *why*, not *what*.
2. Constants in `config.h`, with the reasoning beside them.
3. **No serial prints** — the port is micro-ROS's.
4. Test on hardware. Every constant above that says "measured" was.
5. Keep this README current with the code.

## Resources

- [DRV8876 datasheet](https://www.ti.com/lit/ds/symlink/drv8876.pdf)
- [Arduino-Pico / PlatformIO](https://arduino-pico.readthedocs.io/en/latest/platformio.html)

## Authors

Alfie Bot Project
