# Bring-up tools

Not compiled. PlatformIO only builds `src/` and `lib/`, so these sit here safely
alongside the real firmware instead of displacing it.

To run one, swap it in temporarily and put the real firmware back afterwards:

```bash
cp src/main.cpp /tmp/main.cpp.real
cp bringup/position_test.cpp src/main.cpp
~/.platformio/penv/bin/pio run -e back -t upload
# ... test ...
cp /tmp/main.cpp.real src/main.cpp
~/.platformio/penv/bin/pio run -e back -t upload
```

The `#include "../include/config.h"` path works from either location, so no edit
is needed when swapping.

---

## position_test.cpp

Open-loop position moves and `GEAR_RATIO` calibration. Line-based commands —
type and press Enter.

```
g <mm>   go to absolute height, indicated mm from datum
m <mm>   move relative, signed
c <mm>   calibrate: enter the REAL measured travel since datum
0        set the datum here          p <n>  set duty
u / d    one 300 ms burst up/down    s      status
L        flip assumed limit polarity w      hand-move monitor
n        nFAULT idle check           x      brake / abort
```

**Calibration procedure** — this is what produced the current `GEAR_RATIO`:

```
m -400        home down until it aborts with LIMIT
0             datum at the switch
g 300         drive up, clear of both stops
              ... measure the REAL displacement with a rule ...
c 300.0       enter the measurement
```

Then put the printed `GEAR_RATIO` in `config.h` and re-derive the constants it
lists. **Re-run this after any encoder or decoder change** — the calibration is
only as good as the counting underneath it, which is exactly how the original
209.7 survived for so long (see the `GEAR_RATIO` note in `config.h`).

### Why it is safe to run

Fixed duty, no PID. Direction is decided once per move from the sign of the
position error, so a sign fault gives a one-directional runaway that the
reversal check catches rather than an oscillation. Moves abort on stall,
overspeed, reversal, bottom limit, a distance-scaled timeout, or `x`.

**There is no top limit switch.** Only the bottom is switched. On an upward move
nothing stops the actuator at the top except the stall detector. The per-move cap
is 500 mm; step up in increments if the scale is ever in doubt again.

## What bring-up established

Recorded because each cost real time, and all three were live bugs in the
**real** firmware, not in the test rigs:

- **Encoder decoder** counted ±1 on every interrupt from current pin levels with
  no state memory, losing ~1 count in 3. It corrupted every calibration this
  project ever took, including gen1's, and is the origin of the "unexplained
  2.33× gap" that produced `GEAR_RATIO = 209.7`. Fixed with a Gray-code
  transition table in `motor_control.cpp`.
- **Limit switch polarity** inverted at all three read sites. The switch is
  normally-closed, so HIGH means "at the bottom". Reading it active-low made
  `calibrateActuator()` skip its descent and zero wherever the actuator happened
  to be — a silent false datum with no error.
- **`RUNAWAY_VELOCITY_MPS`** was derived as `MAX_ACTUATOR_VELOCITY × 2`, which
  sat below real gravity-assisted descent speeds. Every normal descent would have
  latched an unrecoverable fault. It is now an absolute constant.

Also: the DRV8876's ITRIP chopper is deliberately disabled (IPROPI strapped to
GND). It was clamping below the current needed to lift. See `config.h`,
"WHY THERE IS NO CURRENT TRIP".

---

## pid_tune.py

Velocity-PID step-response harness. Needs the real firmware flashed, the
micro-ROS agent running, and the actuator homed.

```bash
python3 bringup/pid_tune.py --velocities 0.020 0.030 0.040 0.046 --csv /tmp/steps.csv
python3 bringup/pid_tune.py --down          # also measure descending steps
```

Reports per step: steady-state error, rise time, overshoot, settling, ripple,
and the fraction of samples pinned at the duty ceiling or at MIN_PWM_UPWARD.

**`BackCmd.velocity` is a velocity LIMIT, not a command.** The firmware runs a
cascade — position error x POSITION_KP (5.0, hardcoded in `motor_control.cpp`,
not exposed in config.h) -> clamped to the commanded velocity -> acceleration
ramp -> velocity PID. A step is produced by commanding a distant position with a
velocity cap so the outer proportional term saturates.

Two limits on what it can tell you:

- Rise times under ~150 ms are the **acceleration ramp** (0.35 m/s^2), not the
  loop. Judge the gains by overshoot, settling and steady-state error.
- `MIN_PWM_UPWARD` (100) is a floor on any non-zero output, and duty 100 already
  gives ~18.7 mm/s. Against a 46 mm/s cap that leaves a **2.5:1** controllable
  range, and commands below the floor make the loop chatter. Steps below
  `--min-trackable` are skipped rather than producing nonsense.

**Re-measure the break-away duty before tuning.** MIN_PWM_UPWARD = 100 was set
from a duty sweep taken with the broken encoder decoder, so the erratic motion
at duty 60-90 may have been counting noise rather than stiction. If the true
floor is lower, the controllable range widens considerably.
