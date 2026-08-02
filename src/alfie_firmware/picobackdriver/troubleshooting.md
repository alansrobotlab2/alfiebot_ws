# Back Driver Gen2 Bring-Up — Troubleshooting State

Handoff document. Written mid-bring-up; read **Open Problem** first, it is the
only thing blocking.

---

## TL;DR

**RESOLVED — the actuator lifts.** Current regulation was the limiter. IPROPI is
now strapped to GND (datasheet §7.3.3.2), which disables the ITRIP chopper, and
`nFAULT` went from 13/60 to **0/60** on every burst. Commanded UP now produces
net upward motion for the first time in this bring-up.

**Next action: find true break-away duty**, then set `MIN_PWM_UPWARD` from it.
Currently 50, which measurement shows is too low. See [Next Steps](#next-steps).

The subtlety worth carrying forward: ITRIP measured *nominal* at 1.33 A, and at
duty 60 the motor can only demand 0.86 A — so by the arithmetic it should never
have been chopping at all. It was. See
[Why ITRIP chopped below its own trip point](#why-itrip-chopped-below-its-own-trip-point).

---

## Resolution

### Result after strapping IPROPI to GND

Jog test, duty 60, 24 kHz, `nFAULT 0/60` on **every** burst:

| burst | moved |
|---|---|
| DOWN | −474 |
| UP | +74, +32, −86, −93, +3, −58, +171, +183 |

Net across the eight UP bursts: **+224 counts** (position −475 → −251). Duty 60
sits right on break-away — individual bursts go both ways, but the trend is up.

The velocity column discriminates them: bursts that gained peaked at
0.014–0.017 m/s, bursts that lost peaked at 0.031–0.045 m/s. The fast ones are
gravity winning, not the motor working harder.

Compare the same duty before the strap: −121 counts, `nFAULT` 13/60.

### Why ITRIP chopped below its own trip point

This is the part that misled the whole bring-up, and it is worth understanding
before anyone re-enables current sensing.

All three terms are nominal — `R_IPROPI` 2.486 kΩ measured, `V_VREF` 3.3 V,
`A_IPROPI` 1000 µA/A from the datasheet — giving `ITRIP` = 1.33 A. At duty 60
the motor can draw at most `60/255 × 12 / 3.3` = **0.86 A**. By the arithmetic,
chopping at duty 60 is impossible. It was chopping anyway, 22% of samples.

The resolution is that **ITRIP is not an average-current limit.** The comparator
compares true instantaneous current with only `tBLK` = 1.1 µs of blanking and
`tDEG` = 0.6 µs of deglitch. Brush commutation spikes and switching transients
breach 1.33 A while the average sits at half that. The datasheet effectively
concedes this: §7.3.3.2 recommends a 10 nF capacitor on IPROPI specifically to
stop transients "prematurely triggering" the regulation.

**So the trip point was never the number it computed to**, and no amount of
measuring VREF or R_IPROPI was ever going to reveal that. Anyone re-enabling
current sensing needs the 10 nF cap and should not trust the computed ITRIP
without bench-confirming a duty that should *not* chop indeed does not.

---

## Historical — the problem as it stood before the fix

### Evidence

Powered jog test, hands off, 24 kHz PWM, commanding **UP**:

| duty | moved | nFAULT | |
|---|---|---|---|
| 60 | −121 counts | 13/60 (22%) | sagging |
| 100 | −93 counts | 29/60 (48%) | sagging |
| 160 | −71 counts | 40/60 (67%) | sagging |

Duty **2.7×**, chopping **3×**, motion essentially unchanged and still negative
(sagging) at every duty. This reads as a hard current clamp: force set by ITRIP
rather than by duty, so more duty only chops harder.

Same test commanding **DOWN** (gravity assisting) gives 250–350 counts of travel
with `nFAULT` at only 1–3/60 — low current, no chopping.

### Why the ITRIP explanation is dead

All three terms measured or sourced:

| Term | Value | How established |
|---|---|---|
| `R_IPROPI` | 2.486 kΩ | **Measured** at the CS pad. Nominal 2.49 kΩ, within 0.2% |
| `V_VREF` | 3.3 V | Continuity confirmed to the Pico 3.3 V rail. VREF is high-impedance, so continuity is sufficient |
| `A_IPROPI` | 1000 µA/A | **DRV8876 datasheet**, Electrical Characteristics + worked example §7.3.3.2 |

```
ITRIP = V_VREF / (R_IPROPI × A_IPROPI) = 3.3 / (2486 × 1000e-6) = 1.33 A
```

That is exactly the design intent. And at 1.33 A the belt force should be
`10.81 × (1.33 − 0.12)` = **13.1 kgf against an inferred 4.5 kgf load** — a 2.9×
margin. It sagged anyway.

**So the arithmetic never actually closed.** The force balance said
`I_trip < 0.54 A`; the hardware says 1.33 A. Those cannot both be right, which
means one of the *other* inputs to the force balance is wrong: `GEAR_RATIO`
(derived, never measured), `Kt`, or the 4.5 kgf load figure (inferred, never
measured). Every one of those is unvalidated.

### Ruled out

| Hypothesis | Eliminated by |
|---|---|
| nFAULT noise pickup | Idle check `n` → 0/2000 low with bridge braked |
| Encoder phasing | Verified by hand, driver asleep |
| Motor direction / PH polarity | DOWN now confirmed physically down |
| PMODE / half-bridge inversion | No duty inversion observed |
| Mechanical seizure | Descends freely with gravity |
| PWM ripple (as the whole cause) | Real, and fixed — but chopping persists at 24 kHz |
| Motor torque / gearing | 90:1 has ~3× margin over the load — *but see the caveat above; this rests on unmeasured numbers* |
| R_IPROPI wrong value | **Measured 2.486 kΩ at the CS pad** — nominal, within 0.2% |
| VREF wrong | Continuity confirmed from Pico 3.3 V to VREF; the pin is high-impedance |
| A_IPROPI assumed wrong | Datasheet confirms 1000 µA/A — the value already in use |

---

## Next Steps

### 1. ~~Strap CS/IPROPI to GND~~ — **DONE**

One wire from the CS header pad to GND. VREF stays at 3.3 V — §7.3.3.2 requires
IPROPI-to-GND *and* VREF above GND for a clean disable. Confirmed working:
`nFAULT` 13/60 → 0/60.

Overcurrent (3.5–5.5 A, 2 ms auto-retry) and thermal shutdown are separate
silicon and remain active. This is not the TB6612 situation.

### 2. Find true break-away duty — **next action**

Step `+` up from 60, three or four UP bursts at each duty, until *every* burst
gains ground. Duty 60 is currently marginal (net positive, individual bursts
split both ways).

Then set **`MIN_PWM_UPWARD`** from that measurement with margin. It is currently
**50** — below a duty already shown to be marginal, so it is certainly too low.

Watch the `nFAULT` column stays 0/60 as duty rises. Anything sustained now is a
genuine fault, not chopping.

### 3. Verify the limit switch before restoring the real firmware

Every jog line so far reports `limit raw=LOW`, and per the pin map **LOW =
triggered**. Plausible if parked at the bottom, but jog well clear and confirm
it reads HIGH. The calibration routine trusts this signal, and calibration is
the unprotected path noted below.

### 4. Then, still outstanding

- **Calibrate `GEAR_RATIO`** (currently 209.7, derived not measured). Park low,
  zero, drive to a known high point, measure travel with a rule,
  `GEAR_RATIO_new = 209.7 × indicated / actual`. Then recheck
  `ACTUATOR_MAX_POSITION` — 350 mm was chosen against an uncalibrated scale.
- **Measure the real belt load** with a spring gauge. The 4.5 kgf figure is
  inferred; every margin in config depends on it.
- **Fit a 10 kΩ nFAULT pull-up to 3.3 V** (never 5 V). The internal ~50–80 kΩ
  works but is noise-prone.
- **Close the calibration hole — do this before trusting the real firmware.**
  Calibration drives into the bottom stop for up to 20 s at duty 160 with *no*
  firmware protection: it bypasses normal motor control, so `motor.pwm_output`
  is stale and both the stall and runaway detectors silently never fire. ITRIP
  used to bound that path; nothing does now. Either lower `CALIBRATION_PWM` or
  add an encoder-motion check to the calibration loop. Documented in
  `ros_interface.cpp` at the `CALIBRATION_PWM` definition.
- **Restore the real firmware**: `git mv src/main.cpp.orig src/main.cpp`

---

## Hardware Configuration

### Pin map (Waveshare RP2040 Zero)

| GPIO | Signal | Notes |
|---|---|---|
| GP28 | EN | PWM |
| GP27 | PH | direction |
| GP15 | nSLEEP | HIGH = active |
| GP26 | PMODE | driven LOW for PH/EN mode |
| GP7 | nFAULT | open-drain in, internal pull-up |
| GP3 | ENC_A | |
| GP2 | ENC_B | |
| GP1 | LIMIT_SW | LOW = triggered |
| GP12/13 | I2C0 SDA/SCL | BNO085 |
| GP11 | IMU reset | |
| GP16 | WS2812B | |

Free: GP0, GP4, GP5, GP6, GP8, GP9, GP10, GP14, GP17–22.

**CS (IPROPI) is strapped to GND** to disable current regulation. To use it as a
current *measurement* later needs a free ADC pin — all three (GP26/27/28)
currently carry digital motor signals, so PMODE would have to move off GP26 —
and note that any resistor other than a short re-enables chopping.

### DRV8876 straps

| Pin | State | Required |
|---|---|---|
| PMODE | driven LOW from GP26 | LOW = PH/EN mode. Floating = independent half-bridge, which inverts DOWN duty |
| IMODE | 20 kΩ to GND (as shipped) | level 2. Only the overcurrent response still matters: **auto-retry**, not latch-off |
| VREF | tied to 3.3 V | just has to be > GND now; sets nothing |
| IPROPI | **strapped to GND** (shipped 2.486 kΩ, measured) | disables current regulation, §7.3.3.2 |

### Motor — JGB37-520B, 90:1, 110 RPM @ 12 V

| | |
|---|---|
| No-load | 110 RPM, 120 mA |
| Rated | 10 kg·cm, 85 RPM, 1.0 A |
| Stall | 15 kg·cm, 2.3 A |
| Kt | 6.88 kg·cm/A |
| Winding | **3.3–5.9 Ω measured** (use 3.3 min for protection sizing) |

A **56:1 / 178 RPM** unit is on order and is the intended end state.

### Key formulas

```
Belt: 20T GT2 → 40 mm circumference → 6.37 mm pitch radius
  force(kgf)  = torque(kg·cm) × 1.571
  force(kgf)  = 10.81 × (I − 0.12)          [90:1]  ** UNVALIDATED **
  stall current at duty D = D/255 × 12/3.3 = D/255 × 3.64 A
  PWM ripple  = V × D × (1−D) / (L × f)
```

The force equation is the one that failed to predict reality — it says duty 160
at 1.33 A gives 13.1 kgf against a 4.5 kgf load, and the actuator sagged. Treat
it as suspect until `GEAR_RATIO` and the belt load are measured.

`I_trip = V_VREF / (R_IPROPI × A_IPROPI)` is retired — there is no trip point.

---

## Verified On Hardware

- Encoder phasing (A=GP3, B=GP2) — up = increasing count
- Motor direction (`MOTOR_PH_UP = HIGH`)
- PMODE / PH-EN mode working (no duty inversion)
- Limit switch polarity — **LOW = triggered**, firmware fixed to match
- nFAULT integrity — 0/2000 low when braked
- PWM frequency 24 kHz — 3.2× more motion than 4 kHz at same duty. *Measured
  while chopping was active; the mechanism it was attributed to (ripple against
  a peak-sensing trip point) no longer exists. 24 kHz kept anyway.*
- ~~ITRIP does engage and protect~~ — true at the time, now deliberately disabled

---

## Wrong Turns — Do Not Repeat

**1. "The gearing is too low."** Drove two motor swaps (56:1 → 30:1 → 90:1).
The 30:1 was rejected for failing to lift — but ITRIP was almost certainly
already low at the time, so it never got a fair test. It may well have been
adequate. The 90:1/56:1 give real margin regardless, so nothing is wasted, but
do not conclude anything about gearing until ITRIP is fixed.

**2. "3.3 V is on CS instead of VREF."** Plausible and would have explained
everything. It was a misread pin label; 3.3 V is on VREF.

**3. "PWM ripple is the whole story."** Ripple was real and worth fixing — 3.2×
improvement — but chopping persists at 24 kHz where ripple is negligible, so
there is a second, larger effect underneath.

**4. "ITRIP is misconfigured — find the wrong component."** Days went into
hunting a wrong value in `ITRIP = V_VREF / (R_IPROPI × A_IPROPI)`. Each term was
suspected in turn and each measured nominal: VREF 3.3 V, `R_IPROPI` 2.486 kΩ,
`A_IPROPI` 1000 µA/A. The search was doomed because **no term was wrong** — the
*equation* does not describe what the comparator actually does. It trips on
instantaneous current, and transients breach it while the average sits at half
the trip point. Right symptom, right subsystem, unfalsifiable framing.

**5. "Therefore ITRIP is not the problem at all."** The over-correction, and
briefly the conclusion here after all three terms came back clean: since the
arithmetic said chopping was impossible at duty 60, the cause had to be
mechanical — `GEAR_RATIO`, `Kt`, or the belt load. Wrong. Disabling the chopper
fixed it. Clean arithmetic on an unvalidated *model* is not evidence.

**Common thread:** all five came from chaining inference off indirect evidence
instead of measuring. The nFAULT counts in particular were over-interpreted for
several rounds. **Measure before theorising** — and when a model needs its third
consecutive rescue, suspect the model itself, not the next term in it. The
decisive move here was not another measurement of a term; it was removing the
whole mechanism and seeing what changed.

---

## Gotchas Worth Knowing

- **Encoder phasing is a property of the motor, not the harness.** It flipped
  when the motor was swapped. Re-verify by hand, driver asleep, on every change.
- **nFAULT pulses per chop event**, it does not hold low. At 4 kHz a ~30–50 µs
  pulse aliases to 12–22% of samples at a 200 Hz sample rate. If chopping every
  cycle, the low-fraction scales *with* frequency.
- **ITRIP senses peak current**, not average, with only 1.1 µs blanking. Ripple
  can breach it while the average sits well below.
- **`tOFF` (25 µs) only applies to fixed off-time mode** (IMODE 1/4). This board
  runs cycle-by-cycle (IMODE 2), which waits for the next input edge instead —
  so high PWM frequency is fine and desirable. If IMODE ever moves to 1 or 4,
  revisit `PWM_FREQUENCY`.
- **CS (pin 6) and VREF (pin 5) are adjacent.** Easy to confuse.
- **The floor is applied after the ceiling clamp** in `applyVelocityPID`, so
  `MIN_PWM_UPWARD > VELOCITY_PID_OUTPUT_LIMIT` silently defeats the ceiling.
- **`fix_atomic.py`** hooks `AddPreAction` on the ELF. It was previously a post
  action on `libFrameworkArduino.a`, which never fired after
  `pio run -t clean_microros` — the exact workflow used after a `.msg` change.
- **Background bash + pipe**: `pio run | tail` reports `tail`'s exit code. A
  build can fail while the task reports success. Check the output, not the code.

---

## Firmware State

### Files

- `src/main.cpp` — **temporary powered-jog bring-up test** (not the application)
- `src/main.cpp.orig` — the real firmware, parked. Restore with
  `git mv src/main.cpp.orig src/main.cpp`

### Jog test commands

| Key | Action |
|---|---|
| `u` / `d` | jog up / down, one 300 ms burst, auto-brakes |
| `+` / `-` | duty ±10 (30–160) |
| `f` | cycle PWM 4k → 8k → 16k → 24k Hz |
| `n` | nFAULT idle check (chopping vs noise) |
| `0` | zero encoder count |
| `x` | brake |
| `?` | help |

Open loop by design — no PID, so an inversion cannot become positive feedback.
Bursts abort on overspeed or motion against the command. It has **no stall
abort**, so it will happily push into a hard stop; watch position.

⚠️ **That last point got sharper.** With ITRIP disabled, a burst into a hard stop
at duty 160 draws 2.28 A rather than the 1.33 A the chopper used to allow —
~17 W in the winding instead of ~5.8 W. Bursts are only 300 ms, so this is not
dangerous, but do not sit on `u` against the top stop. Start low and step up.

### Safety features implemented in the real firmware

- **Stall detection** — duty above threshold + no encoder motion for 250 ms.
  Classifies `ERROR_MOTOR_BLOCKED` (was moving, then stopped) vs
  `ERROR_LOAD_EXCEEDED` (never broke away). Blocked latches hard immediately;
  overload retries 3× at 3 s intervals then latches.
- **Runaway guard** — overspeed (>2× max velocity) or motion against command.
  Latches until reset. Catches every sign-inversion fault.
- **nFAULT handling** — distinguishes current chopping from device fault by
  whether the bridge is commanded to drive. Driver faults re-arm via nSLEEP
  cycle, rate-limited to 1/s.
- **LED**: solid red = blocked/driver fault, blinking red = overload retrying,
  fast red strobe = runaway, amber = current limiting.
- **BackState** publishes `error_code`, `current_limiting`, `stall_position`,
  `stall_count`, `fault_latched`. Constants `ERROR_MOTOR_BLOCKED=9`,
  `ERROR_LOAD_EXCEEDED=10`, `ERROR_MOTOR_RUNAWAY=11`.

### Current config values

```
GEAR_RATIO                  209.7   (DERIVED — needs hardware calibration)
MOTOR_FREE_SPEED_RPM        110
MOTOR_KT_KGCM_PER_A         6.88
PWM_FREQUENCY               24000
MAX_ACTUATOR_VELOCITY       0.035
VELOCITY_PID_KP             2500.0
VELOCITY_PID_KI             3000.0
VELOCITY_PID_OUTPUT_LIMIT   160
VELOCITY_PID_INTEGRAL_LIMIT 0.045
MIN_PWM_UPWARD              50
MIN_PWM_DOWNWARD            20
ACTUATOR_MAX_POSITION       0.350   (chosen against an uncalibrated scale)
```

---

## Background

Gen1 used a TB6612FNG with a 56:1 / 178 RPM motor. During the gen2 rebuild the
motor was changed to 30:1 / 333 RPM and **the TB6612 popped and smoked**.

Root cause: the TB6612 is rated 1.2 A continuous into a 0.78 W package, with
**no overcurrent protection at all**. The motor stalls at ~3.64 A (measured
winding), which is ~6.6 W — roughly 8× the package rating. The lower reduction
raised current demand, but the part was marginal regardless.

Replaced with a **DRV8876** (3.5 A, 44 °C/W vs the TB6612's 160 °C/W, plus
adjustable current regulation and fault reporting). The thermal path matters more
than the current rating — per-amp `RDS(on)` is actually slightly *worse*.

### Board

Generic Amazon DRV8876 carrier (2-pack, "3.5A Single Brushed DC Motor Driver").
Not Pololu. Breaks out VM, GND, EN, PH, nSLEEP, nFAULT, CS, VREF. PMODE shipped
**open** (floating = independent half-bridge = no current regulation and inverted
DOWN duty) and is now driven from GP26.

### Serial device

Waveshare RP2040 Zero, serial `5303284740EC7D9C`. Rule added to
`../99-alfie-pico.rules` mapping it to `/dev/ttyAlfieB`. Install with:

```bash
sudo cp ../99-alfie-pico.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Build: `~/.platformio/penv/bin/pio run -e back`
