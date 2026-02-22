# Per-Joint Deadband Filter for Inference Jitter Suppression

## Context

GR00T N1.6 flow matching produces ~0.025 rad of inference noise on position joints even when the ground truth action is perfectly constant. Analysis of episode 0 training data confirmed: right arm joints have **zero** std for the first 209 frames (13.9s) during hold-still phases, yet model output jitters visibly. The noise worsens when the base/head are moving, likely due to DiT cross-attention leakage across the shared 128D latent space.

The current pipeline has no deadband or threshold filtering anywhere. The rate limiter smooths large jumps but faithfully chases noisy micro-oscillations (e.g., shoulder pitch bouncing between 0.105 and 0.110 rad).

**Goal**: Add a per-body-part deadband filter that suppresses target changes smaller than the jitter floor, so the rate limiter stops chasing noise. Backward-compatible (defaults to 0.0 = disabled).

## Ground Truth Evidence

Episode 0 training data — right arm action labels during hold-still phase (frames 0–208):

| Joint | Held Value (rad) | Std | First Movement |
|---|---|---|---|
| right_shoulder_yaw | 0.00000 | 0.00000 | Frame 210 (14.0s) |
| right_shoulder_pitch | 0.10738 | 0.00000 | Frame 209 (13.9s) |
| right_elbow_pitch | -1.48489 | 0.00000 | Frame 210 (14.0s) |
| right_wrist_pitch | -0.14419 | 0.00000 | Frame 209 (13.9s) |
| right_wrist_roll | 0.00000 | 0.00000 | Frame 210 (14.0s) |

The training labels are **perfectly clean** — any jitter in model output is 100% inference noise.

## Model Inference Noise Floor (from RTC Analysis)

Mean prediction spread (std across overlapping chunks) at 4 denoising steps:

| Joint Group | Representative Joint | Spread (rad) |
|---|---|---|
| Right arm | r_elbow_pitch | 0.027 |
| Right arm | r_shoulder_pitch | 0.017 |
| Head | head_yaw | 0.041 (0.028 with RTC) |
| Head | head_pitch | 0.022 (0.016 with RTC) |
| Base | cmd_vel_az | 0.026 (0.016 with RTC) |

## Design

The deadband lives in `RateLimitedInterpolator.set_target()`, applied **before** EMA smoothing and rate limiting.

**Position joints (6:22)**: If `|new_target - current_accepted_target| < deadband`, keep the old target for that joint. The comparison is against the last *accepted* target, not the current interpolated position.

**Base velocity (0:6)**: Snap to zero if `|vel| < threshold`. This is an absolute deadband (not relative) because velocity commands should be zero when the robot should be still.

### Why this insertion point?

1. **Prevents the rate limiter from chasing noise** — the root cause of visible jitter. Without deadband, the rate limiter faithfully tracks each 0.002 rad oscillation.
2. **Operates on 15 FPS target changes**, not 100 Hz output — threshold math is straightforward.
3. **Reuses existing `BODY_PART_INDICES` pattern** — per-body-part configuration, same as rate limits.
4. **Deadband → EMA → Rate Limit ordering** ensures EMA never sees rejected noise, and rate limiting never chases it.

## Files to Modify

### 1. `src/alfie_gr00t/alfie_gr00t/core/rate_limited_interpolator.py`

**Add defaults dict** (after `DEFAULT_MAX_SPEEDS`):
```python
DEFAULT_DEADBANDS = {
    'back': 0.0, 'left_arm': 0.0, 'left_gripper': 0.0,
    'right_arm': 0.0, 'right_gripper': 0.0, 'head': 0.0,
}
```

**Extend `__init__`** with 3 new params: `deadbands: Optional[dict]`, `base_deadband_linear: float = 0.0`, `base_deadband_angular: float = 0.0`. Build a per-index `self._deadband` array (same pattern as `_max_delta`). Store base deadband thresholds. Add `self._deadband_count = 0` diagnostic counter.

**Add deadband logic to `set_target()`** — before the existing EMA block:
```python
if self._target is not None:
    # Position joints: keep old target if within deadband
    for idx in POSITION_INDICES:
        if self._deadband[idx] > 0.0:
            if abs(target[idx] - self._target[idx]) < self._deadband[idx]:
                target[idx] = self._target[idx]
    # Base linear (0:3): snap to zero if below threshold
    if self._base_deadband_linear > 0.0:
        for idx in range(0, 3):
            if abs(target[idx]) < self._base_deadband_linear:
                target[idx] = 0.0
    # Base angular (3:6): snap to zero if below threshold
    if self._base_deadband_angular > 0.0:
        for idx in range(3, 6):
            if abs(target[idx]) < self._base_deadband_angular:
                target[idx] = 0.0
```

**Add `deadband_active` to `get_stats()`** return dict.

### 2. `src/alfie_gr00t/config/groot_client.yaml`

Add 8 parameters after the rate-limit section:
```yaml
deadband_back: 0.0
deadband_left_arm: 0.0
deadband_left_gripper: 0.0
deadband_right_arm: 0.0
deadband_right_gripper: 0.0
deadband_head: 0.0
deadband_base_linear: 0.0
deadband_base_angular: 0.0
```

### 3. `src/alfie_gr00t/alfie_gr00t/nodes/groot_client.py`

**`_declare_parameters()`** (~line 631): Declare 8 deadband parameters.

**`__init__`** (~line 221–233): Build `deadbands` dict from params, pass to `RateLimitedInterpolator(deadbands=..., base_deadband_linear=..., base_deadband_angular=...)`.

**`_log_parameters()`**: Log deadband values when any are non-zero.

## Recommended Tuning Values

Based on the characterization data (not set as defaults — defaults stay 0.0):

```yaml
deadband_left_arm: 0.025      # matches ~0.025 rad jitter floor
deadband_right_arm: 0.025     # matches ~0.025 rad jitter floor
deadband_head: 0.03           # head has ~0.027 rad noise
deadband_left_gripper: 0.02
deadband_right_gripper: 0.02
deadband_back: 0.005          # 5mm position threshold
deadband_base_linear: 0.01    # m/s — suppress base creep
deadband_base_angular: 0.05   # rad/s — suppress rotational drift
```

### Threshold Sizing Rationale

- **Arms (0.025 rad)**: Matches the observed ~0.025 rad per-joint prediction spread. The model's smallest intentional movement steps are typically >0.05 rad/step at 15 FPS (based on training data), so the deadband won't delay real movements.
- **Head (0.03 rad)**: Slightly larger to match the ~0.027–0.041 rad head noise. Head movements are smooth and large when intentional.
- **Grippers (0.02 rad)**: Conservative — gripper close commands are >0.1 rad/step, so 0.02 won't interfere.
- **Base linear (0.01 m/s)**: Suppresses velocity noise without delaying approach (typical approach speed is 0.05–0.15 m/s).
- **Base angular (0.05 rad/s)**: Suppresses rotational drift (typical intentional rotation is >0.15 rad/s).

## Interaction with Other Smoothing

| Stage | Effect on Jitter | Deadband Interaction |
|---|---|---|
| **Deadband** (new) | Rejects noise at source | First in chain — prevents downstream stages from seeing noise |
| **EMA smoothing** | Dampens accepted target transitions | Applied after deadband — smooths transitions between accepted targets |
| **Rate limiter** | Caps velocity of position joints | Applied after EMA — won't chase noise that deadband already rejected |
| **Base velocity limiter** | Magnitude + accel caps on base | Independent — handles large velocity commands, deadband handles small ones |
| **RTC freeze+inpaint** | Reduces noise at denoising level | Complementary — RTC reduces noise amplitude, deadband filters what remains |

## Verification

1. **Syntax check**: `python3 -c "from alfie_gr00t.core.rate_limited_interpolator import RateLimitedInterpolator"`
2. **Regression**: Run with all deadbands at 0.0 — behavior should be identical to current
3. **Live test**: Set recommended values in yaml, run on robot, compare CSV logs (`/tmp/groot_client_debug.csv`) with and without deadband to verify jitter suppression without delayed intentional movements
4. **Check `get_stats()`**: `deadband_active` count should be high during hold-still phases, near zero during active movement
