# Alfie GR00T Smoothing Migration: G1/BEHAVIOR/RoboCasa → Alfie

Gameplan for incorporating NVIDIA's 3-layer smoothing pipeline into Alfie's
GR00T N1.6 client and server, based on analysis of all three reference
deployments (G1_CONTROL.md, SMOOTH_GROOT.md).

---

## 1. Gap Analysis: NVIDIA Reference vs Alfie Current

| Parameter | G1 WBC | BEHAVIOR R1Pro | RoboCasa GR1 | **Alfie (Current)** | **Alfie (Target)** |
|-----------|--------|----------------|--------------|---------------------|---------------------|
| Dataset FPS | 50 Hz | ~20 Hz | 20 Hz | **15 Hz** | **20 Hz** |
| Action horizon | 30 | 32 | 16 | **16** | **50** |
| n_action_steps | 20 | 8 | 8 | **12** | **8** |
| Chunk overlap | 33% | **75%** | 50% | **~0%** ¹ | **84%** |
| Execution window | 0.40 s | 0.40 s | 0.40 s | **0.80 s** | **0.40 s** |
| Re-plan rate | 2.50 Hz | 2.50 Hz | 2.50 Hz | **~1.0 Hz** | **2.50 Hz** |
| Latency skip | — | — | — | **4 steps** | **6 steps** |
| Rate-limited interp | Yes (arms) | No (PD control) | No | **No** | **Yes** |
| Base velocity limits | RL locomotion | ±[0.75, 0.75, 1.0] | N/A | **decay=0.15** | **velocity capping** |
| Per-joint servo config | N/A (sim) | Kp=150 per joint | N/A (sim) | **uniform 1.5 rad/s** | **per-joint** |
| Temporal ensembling | No | No | No | **No** | **Optional** |

¹ With latency_skip=4, n_action_steps=12, Alfie executes actions[4:16] — the entire
remaining horizon. No actions are discarded, so there's no implicit overlap in the
G1 sense. The model re-plans from scratch each cycle.

### Key Insight: The 0.4s / 2.5 Hz Sweet Spot

All three NVIDIA examples converge on the same execution window (0.4s) and re-plan
rate (~2.5 Hz), despite very different FPS and horizons. This is a deliberate design
choice. Alfie's current 0.8s window and ~1.0 Hz re-plan rate is 2x too slow.

### Closest Analog: BEHAVIOR R1Pro

BEHAVIOR is the best reference for Alfie because:
- **Holonomic base** (like Alfie's mecanum) with velocity limits
- **Absolute position targets** for arms (like Alfie's ABSOLUTE action representation)
- **75% overlap** with n_action_steps=8 at ~20 Hz
- No rate-limited interpolation — relies on PD control (Kp=150) for damping

---

## 2. Two-Phase Target Architecture

### Phase A: Current Model (16-step, 15 FPS)

Immediate improvements with no retraining required.

```
Server (GPU)                          Client (Jetson)
┌─────────────┐                       ┌──────────────────────────────┐
│ Gr00tPolicy  │  ──16 actions──►     │ Action Chunk                 │
│ (16-step)    │  (ZMQ, ~280ms)       │   ↓                          │
└─────────────┘                       │ Rate-Limited Interpolator    │
                                      │   ├─ Arms: max 2.0 rad/s    │
                                      │   ├─ Head: max 1.0 rad/s    │
                                      │   ├─ Grippers: max 3.0 rad/s│
                                      │   └─ Back: max 0.5 m/s      │
                                      │   ↓                          │
                                      │ Base Velocity Limiter        │
                                      │   ├─ max |lx|: 0.15 m/s     │
                                      │   ├─ max |ly|: 0.15 m/s     │
                                      │   └─ max |az|: 0.8 rad/s    │
                                      │   ↓                          │
                                      │ Per-Joint Servo Config       │
                                      │   ↓                          │
                                      │ RobotLowCmd @ 100 Hz        │
                                      └──────────────────────────────┘
```

**What this buys:** Velocity-capped transitions at chunk boundaries, no more jerk
from position discontinuities. Per-joint servo tuning for different body parts.

### Phase B: Retrained Model (50-step, 20 Hz)

Full NVIDIA-style overlap after retraining.

```
Server (GPU)                          Client (Jetson)
┌─────────────┐                       ┌──────────────────────────────┐
│ Gr00tPolicy  │  ──50 actions──►     │ Execute first 8 of 50       │
│ (50-step)    │  (ZMQ, ~280ms)       │   actions[6:14] @ 20 Hz     │
│ @ 20 Hz      │                      │   (84% overlap, 0.4s window) │
└─────────────┘                       │   ↓                          │
                                      │ Rate-Limited Interpolator    │
                                      │   (same as Phase A)          │
                                      │   ↓                          │
                                      │ Base Velocity Limiter        │
                                      │   (same as Phase A)          │
                                      │   ↓                          │
                                      │ Per-Joint Servo Config       │
                                      │   ↓                          │
                                      │ RobotLowCmd @ 100 Hz        │
                                      └──────────────────────────────┘
```

**What this buys:** 84% chunk overlap eliminates boundary discontinuities almost
entirely. The rate-limited interpolator becomes a safety layer rather than the
primary smoothing mechanism. 2.5 Hz re-planning matches the NVIDIA sweet spot.

---

## 3. Layer 1: Chunk Overlap

### Current State (Phase A)

- action_horizon=16, n_action_steps=12, latency_skip=4
- Execution window: actions[4:16] = 12 steps over 804ms
- **0% effective overlap**: all non-skipped actions are executed
- Re-plan rate: ~1.0 Hz (too slow)

**Phase A compromise:** Cannot increase overlap without losing the forward velocity
phase (actions 12-15). Proven: mean |lx| at idx 4-11 is 0.0016 m/s vs 0.024 m/s
at idx 12+ (14.8x gap). The model's trajectory structure requires the full [4:16]
window.

**No changes to chunk overlap in Phase A.** Rate-limited interpolation (Layer 2)
handles smoothing instead.

### Target State (Phase B)

- action_horizon=50, n_action_steps=8, latency_skip=6
- Execution window: actions[6:14] = 8 steps over 400ms
- **84% overlap**: 42 of 50 predicted actions discarded
- Re-plan rate: 2.50 Hz (matches NVIDIA sweet spot)

**Why 50-step at 20 Hz:**
```
execution_window = 8 / 20 Hz = 0.40 s     ← matches all 3 NVIDIA examples
re_plan_rate     = 20 / 8   = 2.50 Hz     ← matches all 3 NVIDIA examples
look_ahead       = 50 / 20  = 2.50 s      ← generous planning horizon
latency_skip     = 280ms / 50ms = ~6      ← compensates for ZMQ RTT
```

### Server Changes for Phase B

`scripts/groot_inference_server.py`:
- `action_horizon` parameter: 16 → 50
- Server already returns `response['actions']` as list of lists; just returns more
- The model's `delta_indices` in the embodiment config must be set to `list(range(50))`
  during training

`config/groot_server.yaml`:
```yaml
action_horizon: 50   # was 16
```

### Training Config for Phase B

`alfiebot_config.py` (or equivalent):
```python
# Key changes for Phase B retraining
dataset_fps = 20              # was 15
action_horizon = 50           # was 16
delta_indices = list(range(50))
```

Data collection must also switch to 20 Hz recording rate.

---

## 4. Layer 2: Rate-Limited Interpolation

This is the **single most impactful change** for Phase A. The G1's
`InterpolationPolicy` prevents joints from exceeding a maximum angular velocity,
converting discrete action targets into continuous, velocity-capped trajectories.

### Design: `RateLimitedInterpolator`

**New file:** `core/rate_limited_interpolator.py`

```
┌───────────────────────────────────────────────┐
│           RateLimitedInterpolator              │
│                                                │
│  set_target(timestamp, target_22d)             │
│     ↓                                          │
│  Per-joint velocity planning:                  │
│     duration = max(|target - current| / max_v) │
│     if duration > step_period:                 │
│        stretch transition                      │
│     else:                                      │
│        arrive by next target                   │
│     ↓                                          │
│  get_action(timestamp) → interpolated 22D      │
│     ↓                                          │
│  Returns: velocity-capped position for joints  │
│           pass-through for base velocity       │
└───────────────────────────────────────────────┘
```

**Key principles (from G1 InterpolationPolicy):**
1. Track from **current physical position** (from observation), not from previous target
2. Per-joint max velocity ensures no jerk at chunk boundaries
3. If the model requests a large position jump, the interpolator stretches the
   transition over multiple control ticks
4. Base velocity (indices 0-5) passes through — it's already a velocity command,
   not a position target. Base smoothing is handled separately (Layer 3).

**Per-body-part max velocities (initial values, to be tuned):**

| Body Part | Indices | Max Speed | Units | Rationale |
|-----------|---------|-----------|-------|-----------|
| Back | [6] | 0.3 | m/s | Slow, heavy linear actuator |
| Left arm | [7:12] | 2.0 | rad/s | Fast for manipulation |
| Left gripper | [12] | 3.0 | rad/s | Quick open/close |
| Right arm | [13:18] | 2.0 | rad/s | Fast for manipulation |
| Right gripper | [18] | 3.0 | rad/s | Quick open/close |
| Head | [19:22] | 1.0 | rad/s | Slow, smooth tracking |

**Implementation notes:**
- Uses `time.monotonic()` for wall-clock trajectory queries (like G1)
- Linear interpolation between current and target (like G1's `scipy.interp1d`)
- Garbage collection of old waypoints (like G1's `interpolation_garbage_collection_time`)
- Thread-safe: targets set from inference thread, queries from 100 Hz timer

### Integration into `groot_client.py`

Replace the current ActionInterpolator usage in `_classic_command_callback`:

```python
# CURRENT (action_interpolator lerps between chunk actions):
if self.interpolate_actions and abs_idx < len(chunk) - 1:
    self._action_interpolator.update_waypoint(wi, chunk[wi])
    action = self._action_interpolator.evaluate(frac_t)

# NEW (rate_limited_interpolator tracks from current position to target):
target = chunk[abs_idx]  # raw model target
obs = self.observation_bridge.get_latest_observation()
action = self._rate_limiter.get_action(
    timestamp=now,
    target=target,
    current_state=obs.state,
)
```

The rate limiter handles:
- Position joints (6-21): velocity-capped interpolation from current to target
- Base velocity (0-5): passed through to base velocity limiter (Layer 3)

### Interaction with Hardware Servo Speed

Alfie's servos already have their own velocity limiting (`target_speed=1.5 rad/s`).
The software rate limiter should be set **at or below** the hardware limit:
- Software limit prevents the *command stream* from having discontinuities
- Hardware limit provides a physical safety backstop
- Combined: double-layered protection

Recommended: set software `max_speed` slightly below hardware `target_speed` so
the software is the active constraint. This makes behavior predictable and debuggable.

---

## 5. Layer 3: Per-Body-Part Control

### Base Velocity: BEHAVIOR-Inspired Velocity Capping

BEHAVIOR R1Pro uses `HolonomicBaseJointController` with velocity limits
±[0.75, 0.75, 1.0] m/s. Alfie currently uses a flat decay (v *= 1 - 0.15).

**Replace base_velocity_decay with explicit velocity capping + acceleration limiting:**

| Parameter | Value | Units | Rationale |
|-----------|-------|-------|-----------|
| max_linear_x | 0.15 | m/s | Safe indoor navigation speed |
| max_linear_y | 0.15 | m/s | Mecanum strafe speed |
| max_angular_z | 0.8 | rad/s | Rotation rate |
| max_linear_accel | 0.3 | m/s² | Limits jerk on velocity changes |
| max_angular_accel | 1.5 | rad/s² | Limits rotational jerk |

**Why acceleration limiting instead of EMA:**
- EMA adds phase lag proportional to the smoothing strength
- Acceleration limiting is physics-based: limits dv/dt without lag at steady state
- BEHAVIOR's PD controller achieves similar effect through Kp damping

**Implementation in `action_publisher.py`:**
```python
def _limit_base_velocity(self, target_vel, last_vel, dt):
    """Clamp base velocity magnitude and acceleration."""
    # Clamp magnitude
    target_vel[0] = np.clip(target_vel[0], -max_lx, max_lx)
    target_vel[1] = np.clip(target_vel[1], -max_ly, max_ly)
    target_vel[5] = np.clip(target_vel[5], -max_az, max_az)

    # Clamp acceleration (dv/dt)
    if last_vel is not None:
        dv = target_vel - last_vel
        max_dv_linear = max_linear_accel * dt
        max_dv_angular = max_angular_accel * dt
        dv[0:3] = np.clip(dv[0:3], -max_dv_linear, max_dv_linear)
        dv[3:6] = np.clip(dv[3:6], -max_dv_angular, max_dv_angular)
        target_vel = last_vel + dv

    return target_vel
```

### Per-Joint Servo Configuration

Replace uniform `default_servo_speed=1.5` with per-group config:

| Body Part | Servo Speed (rad/s) | Acceleration (rad/s²) | Torque |
|-----------|--------------------|-----------------------|--------|
| Left arm (servos 0-4) | 2.0 | 5.0 | 0.5 |
| Left gripper (servo 5) | 3.0 | 8.0 | 0.4 |
| Right arm (servos 6-10) | 2.0 | 5.0 | 0.5 |
| Right gripper (servo 11) | 3.0 | 8.0 | 0.4 |
| Head (servos 12-14) | 1.0 | 3.0 | 0.3 |

**Implementation in `action_publisher.py`:**
```python
SERVO_CONFIG = {
    # (speed, acceleration, torque) per servo index
    range(0, 5):   (2.0, 5.0, 0.5),   # left arm
    5:             (3.0, 8.0, 0.4),    # left gripper
    range(6, 11):  (2.0, 5.0, 0.5),   # right arm
    11:            (3.0, 8.0, 0.4),    # right gripper
    range(12, 15): (1.0, 3.0, 0.3),   # head
}
```

---

## 6. What NOT to Implement (Absent from NVIDIA's Code)

An exhaustive search of the NVIDIA codebase confirmed these techniques are **not used**
in any of the three reference deployments:

| Technique | Status | Why Skip It |
|-----------|--------|-------------|
| Temporal ensembling | Absent | Flow matching produces coherent chunks; overlap is sufficient |
| EMA on actions | Absent | Adds lag without solving the root cause (boundary discontinuities) |
| Action blending at boundaries | Absent | Fighting model corrections during manipulation (proven on Alfie) |
| Kalman filtering | Absent | Over-engineering for this use case |
| Control frequency decimation | Absent | 100 Hz is fine with interpolation |

GR00T relies on:
1. **Flow matching's inherent temporal coherence** (all timesteps refined simultaneously)
2. **Chunk overlap** (predict more than you execute)
3. **Embodiment-specific post-processing** (rate-limited interpolation, PD control, RL loco)

We should follow this philosophy: overlap + rate-limited interpolation + per-joint config.
No additional filtering layers unless empirically proven necessary.

---

## 7. Implementation Plan

### Phase A-1: Rate-Limited Interpolator (Highest Impact)

**Files:**
- **CREATE** `alfie_gr00t/core/rate_limited_interpolator.py`
- **MODIFY** `alfie_gr00t/nodes/groot_client.py` — wire in rate limiter
- **MODIFY** `alfie_gr00t/config/groot_client.yaml` — max speed params

**Scope:**
- New `RateLimitedInterpolator` class with per-joint max velocity config
- Replaces `ActionInterpolator` for position-commanded joints (6-21)
- Base velocity (0-5) passes through unchanged
- Configurable via YAML: `max_joint_speeds` dict per body-part group

**Verification:**
- CSV log comparison: before/after rate limiter on same chunk sequence
- Verify no joint velocity exceeds configured max in the CSV
- Visual: smooth head tracking, no arm jerking at chunk boundaries
- Open-loop eval: MSE should not increase (rate limiter only slows down, never overshoots)

### Phase A-2: Base Velocity Capping + Acceleration Limiting

**Files:**
- **MODIFY** `alfie_gr00t/core/action_publisher.py` — add `_limit_base_velocity()`
- **MODIFY** `alfie_gr00t/config/groot_client.yaml` — velocity/accel limit params

**Scope:**
- Replace `base_velocity_decay` with magnitude capping + acceleration limiting
- Configurable limits: max_lx, max_ly, max_az, max_linear_accel, max_angular_accel
- Remove EMA smoothing for base (alpha params become deprecated)

**Verification:**
- CSV log: verify base velocity magnitude never exceeds caps
- CSV log: verify base acceleration (dv/dt) never exceeds limits
- Live test: robot should approach can smoothly without jerky direction changes
- Compare approach time vs old decay=0.15 (should be similar or faster)

### Phase A-3: Per-Joint Servo Configuration

**Files:**
- **MODIFY** `alfie_gr00t/core/action_publisher.py` — per-servo speed/accel/torque
- **MODIFY** `alfie_gr00t/config/groot_client.yaml` — servo config params

**Scope:**
- Replace uniform `default_servo_speed=1.5` with per-group config table
- Different speed/accel/torque for arms, grippers, head
- Back motor already has separate config (velocity=0.2, accel=0.1 during init)

**Verification:**
- Visual: head should track smoothly (1.0 rad/s), arms should be responsive (2.0 rad/s)
- Gripper open/close should be snappy (3.0 rad/s)
- No servo faults or overcurrent warnings

### Phase B-1: Retraining at 20 Hz / 50-Step Horizon

**Files:**
- **MODIFY** `alfiebot_config.py` — fps=20, action_horizon=50, delta_indices
- **MODIFY** `scripts/rosbag_to_groot.py` — 20 Hz resampling
- **MODIFY** `config/groot_server.yaml` — action_horizon=50

**Scope:**
- Recollect data at 20 Hz (or resample existing 15 Hz data)
- Retrain with action_horizon=50 and delta_indices=list(range(50))
- Server returns 50 actions per inference call

**Verification:**
- Open-loop eval: MSE should be comparable to 16-step model
- Trajectory plots: verify model predicts 2.5s into the future coherently

### Phase B-2: Client Timing Update for 0.4s Window

**Files:**
- **MODIFY** `alfie_gr00t/nodes/groot_client.py` — timing constants, n_action_steps
- **MODIFY** `alfie_gr00t/config/groot_client.yaml` — new timing params

**Scope:**
- `TRAINING_FPS = 20` (was 15)
- `ACTION_STEP_PERIOD = 0.050` (was 0.067)
- `n_action_steps = 8` (was 12)
- `latency_skip = 6` (was 4, recalculated: 280ms / 50ms ≈ 6)
- `action_chunk_size = 50` (was 16)
- `inference_trigger_step = 8` (fire at exhaust, sequential)
- Execution window: actions[6:14] = 8 steps over 400ms
- Re-plan rate: 2.50 Hz

**Verification:**
- Timing logs: verify 400ms chunk execution, 2.5 Hz re-planning
- CSV log: verify full trajectory structure at 20 Hz
- Live test: approach behavior should be noticeably smoother due to 84% overlap
- Compare with Phase A results to isolate the benefit of overlap alone

---

## 8. Config Parameter Reference (Target State)

```yaml
/alfie/groot_client:
  ros__parameters:
    # --- Timing (Phase B values, Phase A in parentheses) ---
    action_chunk_size: 50        # (16) must match training horizon
    n_action_steps: 8            # (12) → 0.4s window, 2.5 Hz re-plan
    latency_skip: 6              # (4) → 280ms / 50ms = 6 steps
    inference_trigger_step: 8    # (12) fire at exhaust

    # --- Rate-Limited Interpolation (Phase A-1) ---
    rate_limit_enabled: true
    max_joint_speeds:
      back: 0.3          # m/s (linear actuator)
      left_arm: 2.0      # rad/s
      left_gripper: 3.0  # rad/s
      right_arm: 2.0     # rad/s
      right_gripper: 3.0 # rad/s
      head: 1.0          # rad/s

    # --- Base Velocity Limits (Phase A-2) ---
    max_base_linear_x: 0.15   # m/s
    max_base_linear_y: 0.15   # m/s
    max_base_angular_z: 0.8   # rad/s
    max_base_linear_accel: 0.3   # m/s²
    max_base_angular_accel: 1.5  # rad/s²

    # --- Per-Joint Servo Config (Phase A-3) ---
    servo_speed_arms: 2.0        # rad/s
    servo_speed_grippers: 3.0    # rad/s
    servo_speed_head: 1.0        # rad/s
    servo_accel_arms: 5.0        # rad/s²
    servo_accel_grippers: 8.0    # rad/s²
    servo_accel_head: 3.0        # rad/s²
    servo_torque_arms: 0.5
    servo_torque_grippers: 0.4
    servo_torque_head: 0.3

    # --- Deprecated (remove after Phase A verified) ---
    # base_smoothing_alpha: 1.0   → replaced by acceleration limiting
    # joint_smoothing_alpha: 1.0  → replaced by rate-limited interpolation
    # base_velocity_decay: 0.15   → replaced by velocity capping
    # interpolate_actions: true   → replaced by rate-limited interpolation
```

---

## 9. Risk Assessment

| Risk | Mitigation |
|------|------------|
| Rate limiter adds latency to joint tracking | Set max_speed at or above training-data velocities; limiter only activates on discontinuities |
| Base velocity caps too restrictive | Start generous, tune down based on live testing |
| 50-step model quality worse than 16-step | Train both, compare open-loop eval metrics before deploying |
| 20 Hz data collection changes demo quality | Record at native camera FPS, downsample to 20 Hz in post-processing |
| Per-joint servo config causes oscillation | Start with current uniform values, tune one group at a time |

---

## 10. Success Criteria

1. **Phase A complete:** No visible jerk at chunk boundaries. Head tracks smoothly.
   Base approach is steady. CSV logs confirm velocity limits respected.

2. **Phase B complete:** 0.4s execution window, 2.5 Hz re-plan rate. Robot behavior
   qualitatively matches NVIDIA reference videos. Can-pick-and-place task succeeds
   at ≥50% rate in varied conditions.

3. **Quantitative:** Open-loop eval MSE ≤ 0.002 (comparable to G1's 0.00135 on
   pretrained model). Live action CSV shows <5% of joint commands clipped by
   rate limiter (indicating model already produces smooth trajectories, and the
   limiter is only a safety net).
