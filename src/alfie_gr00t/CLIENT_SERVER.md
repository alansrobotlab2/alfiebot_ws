# Client-Server Smooth Execution Gameplan

How to achieve smooth, natural robot motion despite ~270ms round-trip inference latency.

---

## The Latency Problem

**The numbers:**
- Round-trip inference: ~280ms (3.6 Hz) — observation capture through ZMQ through model through response
- Training data rate: 15 FPS → 67ms per action step
- Actions per inference cycle: 280ms / 67ms ≈ **4 actions** stale by the time chunk starts executing
- Current model: AH=16 → 16 actions × 67ms = **1.07s per chunk**

**The core challenge:** Every action the robot executes is based on an observation captured ~270ms ago. By the time the last action in a 16-step chunk executes, the observation is **1.34s stale** (1.07s chunk + 0.27s inference). The robot is running blind on a prediction of what should happen based on what it saw over a second ago.

**Why it still works (and why other teams ship this):**

1. **The model predicts trajectories, not reactions.** During training, the model learns: "from this visual scene + robot state, here's the full motion trajectory for the next N steps." It implicitly learns dynamics — where the arm will be, how the base will coast, where the can will appear. The actions at step 10 already account for the fact that the arm moved during steps 1-9.

2. **Joint positions are absolute targets.** When the model says "shoulder pitch = -1.2 rad at step 5", the servo converges to that target regardless of when the command arrives. The servo's internal speed/acceleration profile handles the physical smoothing. A 270ms-stale observation means the servo starts tracking ~270ms late, but it still tracks to the right place.

3. **Base velocity is the sensitive channel.** Velocity integrates to position continuously — 270ms of stale velocity command means ~270ms × v meters of position error before correction. This is why base needs special treatment (latency skip, decay).

4. **Frequent re-planning corrects drift.** With AH=4 and overlapped inference, the model corrects course every ~267ms. Any error from observation staleness is bounded to one chunk duration before a fresh observation corrects it.

---

## ANSWERED: Action Horizon Temporal Consistency

**Question:** When the model predicts 16 steps into the future at time T, how well does predicted step k match the actual inference output at time T+k?

Formally: does `H[T][k] ≈ H[T+k][0]`?

### Phase 0 Results (2026-02-16)

**Method:** `scripts/action_horizon_analysis.py` — ran inference at every frame across 6 episodes (317-322), producing ~1260 overlapping 16-step horizons. Compared `H[T][k]` vs `H[T+k][0]` for k=1..15. Note: measured ~400ms RTT during this run because the server was multitasking; actual dedicated latency is ~280ms (k≈4).

**Results at k=4 (280ms operating point):**

| Body Part | Pearson r | MAE | Safe skip (r>0.9) | Max skip (r>0.7) |
|-----------|----------|-----|-------------------|------------------|
| **Base velocity** | **0.969** | 0.0022 | k=15 (entire horizon) | k=15 |
| **Right arm** | **0.974** | 0.058 | k=11 (737ms) | k=15 |
| **Right hand** | **0.915** | 0.020 | k=4 (268ms) | k=12 (804ms) |
| **Head** | **0.988** | 0.029 | k=15 (entire horizon) | k=15 |
| Left arm | 1.000 | 0.0 | k=15 | k=15 |
| Back / left hand | NaN | 0.0 | N/A (constant) | N/A |

**Key finding: The model is a trajectory planner, not a reactive controller.** All moving body parts (base, arm, head) show r>0.97 at the 280ms operating point. The model commits to a coherent multi-step plan; fresh observations fine-tune the trajectory rather than fundamentally changing it.

**Universal latency skip at k=4 is justified** for all body parts — even the gripper clears the 0.9 threshold (r=0.915) at k=4.

**Single-episode caveat:** A single manipulation-heavy episode (ep 3) showed misleadingly low base correlation (r=0.16 at k=4) because near-zero base velocities caused noise-dominated correlation. The multi-episode run corrected this by including diverse approach + manipulation phases.

### Correlation decay profile

Smooth decay from near-perfect at k=1 to still-high at k=15:

```
k=1  (67ms)    base=0.988  r_arm=0.993  r_hand=0.969  head=0.998
k=4  (268ms)   base=0.969  r_arm=0.974  r_hand=0.915  head=0.988  ◄ operating point (~280ms)
k=6  (402ms)   base=0.954  r_arm=0.956  r_hand=0.871  head=0.979
k=10 (670ms)   base=0.950  r_arm=0.913  r_hand=0.771  head=0.960
k=15 (1005ms)  base=0.942  r_arm=0.867  r_hand=0.652  head=0.940
```

**Per-joint standouts:**
- `right_shoulder_pitch` degrades fastest in the arm group (r=0.54 at k=4 in single-episode; ~0.87 multi-episode)
- `head_yaw` is the weakest head joint (horizontal tracking changes with base movement)
- `cmd_vel_lx` and `cmd_vel_az` are the only active base velocity dims; others are near-constant

### What this means for the gameplan

1. **Universal latency skip is viable.** Not just base — all body parts can skip by 4 steps at 280ms latency with r>0.91. Even gripper clears 0.9 at k=4.
2. **AH=4 is less urgent than anticipated.** The model's long-horizon predictions are reliable. AH=4's benefit shifts from "prediction accuracy" to "faster course correction frequency" (3.6 Hz vs 0.75 Hz).
3. **Skip=4 for 280ms latency.** (280ms / 67ms ≈ 4.2). With n_action_steps=16, this uses actions 4-15 (12 useful steps = 804ms).
4. **Gripper can use skip at k=4** (r=0.915) but degrades quickly beyond that (r=0.87 at k=6). For longer skip values, consider threshold-based open/close instead.

### Analysis tool

`scripts/action_horizon_analysis.py` — standalone (no ROS2). Produces decay curve plots, per-body-part heatmap, per-joint breakdown, and CSV metrics.

```
python action_horizon_analysis.py \
    --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-indices 317,318,319,320,321,322 \
    --host 192.168.50.201 --port 5555 \
    --latency-ms 400 --save-dir /tmp/horizon_analysis_multi/
```

---

## Current System: AH=16 Sequential

```
Timeline for one inference cycle (1.34s total):

t=0.00   ┌─────────── chunk plays (16 actions × 67ms) ──────────┐
         │ action[0]  action[1]  ...  action[14]  action[15]     │
t=1.07   └───────────────────────────────────────────────────────┘
                                                                  │
t=1.07   capture post-exhaust observation ◄───────────────────────┘
t=1.07   send inference request ──► server
         │  base velocity = 0, joints hold position (DEAD TIME)  │
t=1.34   receive response ◄── server
t=1.34   install chunk, start executing
```

**Problems:**
- **270ms dead time** every 1.34s — base stops, motion pauses
- **0.75 Hz effective re-planning rate** — model only gets fresh observations every 1.34s
- **Observation staleness:** 270ms at chunk start, 1.34s by chunk end

**Dead parameters** (declared, read, logged, but never used in `_command_callback`):

| Parameter | Value | Was Tuned For | Status |
|-----------|-------|---------------|--------|
| `latency_skip_base` | 3 | Base velocity reads ahead to compensate for obs-to-action delay | Dead — removed during refactor |

| `max_joint_delta` | 2.0 | Per-step joint position change limit | Dead — never enforced |
| `_blend_from` | (set) | Smooth chunk transitions by blending old→new trajectory | Dead write — never read |
| `_blend_steps` | 3 | Number of actions to blend over (~200ms) | Dead — never read |

---

## Near-Term: AH=16 Tuning (No Retraining)

These changes improve smoothness with the current AH=16 model and 270ms latency.

### 1. Re-enable `latency_skip_base`

**What:** Base velocity reads from `chunk[idx + skip]` instead of `chunk[idx]`.

**Why:** The observation was captured at the END of the previous chunk (t=0). Inference takes 270ms. The chunk starts executing at t=270ms. `action[0]` was predicted for t=67ms (1 action after observation). But the robot is at t=270ms — it's already 4 actions into the future relative to the observation.

`action[4]` was predicted for t=4×67ms = 268ms ≈ 270ms, which is approximately where the robot IS when the chunk starts. So base velocity should read from `action[idx + 4]`.

**Code change** in `_command_callback`:
```python
# Base velocity reads ahead to compensate for observation staleness
base_idx = min(idx + self.latency_skip_base, len(chunk) - 1)
action[0:6] = chunk[base_idx, 0:6]
# Joints keep action[idx] — servo profiles handle convergence
```

**Parameter:** `latency_skip: 4` (280ms / 67ms ≈ 4). With n_action_steps=16, effectively uses actions 4-15 (12 useful steps = 804ms of trajectory).

**VALIDATED (Phase 0):** At k=4, all body parts exceed the 0.9 correlation threshold — base r=0.969, right arm r=0.974, gripper r=0.915, head r=0.988. **Universal latency skip is viable** — apply skip=4 to all channels, not just base.

### 2. Improve gap behavior

**Current:** When chunk exhausts, base velocity hard-zeros instantly. Joints hold last position.

**Better:** Instead of hard-zeroing, apply a fast exponential decay over the gap:
```python
if chunk_exhausted:
    gap_elapsed = elapsed - chunk_duration
    gap_decay = max(0.0, 1.0 - gap_elapsed / 0.15)  # decay to 0 over 150ms
    action[0:6] = chunk[-1, 0:6] * gap_decay
```

This gives 150ms of decelerating coast instead of instant stop, creating smoother motion at chunk boundaries. The remaining ~120ms of the gap is fully stopped (safe). Joints continue holding (correct — no risk of drift).

**Risk:** If the next chunk reverses direction, the 150ms coast adds a small overshoot. Start with hard-zero and only enable this if chunk boundary jerk is the dominant smoothness issue.

### 4. Re-enable chunk transition blending

**What:** When a new chunk installs, blend its first few actions with the old chunk's last action to avoid joint position jumps.

**Why currently disabled:** "Chunk blending hurts manipulation — crossfading old/new chunk tails fights model course corrections during grasping." This was true with overlapped chunks (n_steps < 16). With n_steps=16 (sequential), the concern is different: after a 270ms gap of holding position, the new chunk's action[0] may not exactly match the held position, causing a small jump.

**Code change** in `_command_callback`, after promoting pending chunk:
```python
if self._blend_from is not None and idx < self._blend_steps:
    blend_alpha = (idx + 1) / (self._blend_steps + 1)
    # Blend joints only (6:22), NOT base velocity (0:6)
    # Base must respond immediately to model's course corrections
    action[6:] = (1 - blend_alpha) * self._blend_from[6:] + blend_alpha * action[6:]
```

**Parameter:** `_blend_steps: 2` (~134ms blend). Test 0, 1, 2, 3 via CSV comparison.

### 5. Servo profile tuning

The servos' internal speed/acceleration/torque parameters in `ActionPublisher` are the physical smoothing layer:

```python
default_servo_speed: 1.5        # rad/s
default_servo_acceleration: 5.0  # rad/s²
default_servo_torque: 0.5       # fraction of max
```

These control how the servo tracks target positions. The servo interpolates internally between its current position and the commanded target using a trapezoidal velocity profile.

**Tuning strategy:**
- **Higher speed + acceleration** = servos track targets faster = less lag but potential jitter
- **Lower speed + acceleration** = servos smooth out discontinuities but add tracking lag
- The current values (1.5 rad/s, 5.0 rad/s²) are reasonable defaults. Tune per-body-part if needed (arms might want different tuning than head).

### 6. Overlapped Inference (the big win)

**What:** Fire inference mid-chunk instead of waiting for chunk exhaust. Eliminates dead time and doubles re-planning rate.

**Strategy: n_steps=8, trigger at window action[4], skip=4**

```
Timeline with overlapped inference (536ms per cycle, ~0ms dead time):

Chunk A (8 actions executed from horizon):
t=0ms    ┌─ action[4] ─ action[5] ─ action[6] ─ action[7] ─┐
t=268ms  │  action[8] ─ action[9] ─ action[10] ─ action[11] │
t=536ms  └───────────────────────────────────────────────────┘
                                     ▲ promote chunk B (pending arrives ~480ms)
         ▲ fire inference B
         t=200ms (window action[3])

Chunk B (8 actions executed from horizon, skip=5):
t=536ms  ┌─ action[5] ─ action[6] ─ action[7] ─ action[8] ─┐
t=804ms  │  action[9] ─ action[10] ─ action[11] ─ action[12] │
t=1072ms └────────────────────────────────────────────────────┘
                                      ▲ promote chunk C
          ▲ fire inference C
          t=736ms (window action[3])
```

**Timing math:**
- 8 actions × 67ms = **536ms** per chunk
- Inference triggered at window action[3] = **200ms** into chunk
- Inference takes ~280ms, returns at **~480ms** — **56ms buffer** before chunk exhausts at 536ms
- Observation staleness at next chunk start: 536 - 200 = **336ms = 5 steps → skip=5**
- Phase 0 correlation at k=5: base r=0.964, arm r=0.966, gripper r=0.893, head r=0.983

**Why n_steps < 16 won't cause "praying mantis" this time:**
The previous trajectory repetition problem (n_steps=8 with AH=16) happened because:
1. Observation was captured **post-exhaust** — model saw nearly the same scene each time
2. No skip — model's action[0] targeted the same position as previous action[0]

The new approach fixes both:
1. Observation captured **mid-chunk** (at action[3]) — robot has visibly moved, model gets fresh visual input
2. **Skip=5** — execution starts from the point in the trajectory matching "now", not from the beginning

**Comparison to current system:**

| Metric | Current (sequential) | Overlapped n=8 |
|--------|---------------------|----------------|
| Chunk duration | 1.07s | 536ms |
| Dead time | 280ms (20%) | ~0ms |
| Re-planning rate | 0.75 Hz | 1.87 Hz |
| Max obs staleness | 1.35s | 536ms |
| Obs age at chunk start | 280ms | 336ms |
| Useful horizon used | actions[0:16] | actions[5:13] |
| Effective duty cycle | 79% | ~100% |

**First chunk is special:** Initial inference fires immediately with no prior chunk. Observation is fresh. Inference takes 280ms = 4 steps → first chunk uses `skip=4`, executing actions[4:12]. Subsequent chunks all use `skip=5`.

**Validation:** `scripts/overlapped_execution_test.py` — simulates both strategies against recorded episodes using pre-collected horizons. Compares trajectory quality, chunk transition smoothness, and timing margins. Supports multi-episode runs with `--episode-indices`.

### Overlapped Simulation Results — VALIDATED (2026-02-16)

**Method:** `scripts/overlapped_execution_test.py` — collected horizons at every frame via ZMQ inference, then simulated sequential (skip=4, 16 steps, 280ms dead time) vs overlapped (n_exec=8, trigger@step3, skip=4/5, ~0ms dead time) execution. Oracle (zero-latency, fresh inference every frame) computed as upper bound.

**Multi-episode results (6 episodes: 317-322, 1267 frames, 84.9s of data):**

| Metric | Sequential | Overlapped |
|--------|-----------|------------|
| Total chunks | 81 | 158 |
| Cycle duration | 1072ms | 536ms |
| Dead time | **25.9%** | **1.9%** |
| Re-planning rate | 0.95 Hz | **1.86 Hz** |

MAE vs ground truth (ALL frames, dead time penalized):

| Body Part | Sequential | Overlapped | Improvement |
|-----------|-----------|------------|-------------|
| **base** | 0.00483 | 0.00303 | **+37.3%** |
| **right_arm** | 0.13701 | 0.11957 | **+12.7%** |
| **right_hand** | 0.09104 | 0.06256 | **+31.3%** |
| **head** | 0.06731 | 0.06061 | **+9.9%** |

MAE vs ground truth (active frames only — model quality without dead time penalty):

| Body Part | Sequential | Overlapped | Oracle | Winner |
|-----------|-----------|------------|--------|--------|
| base | 0.00280 | 0.00282 | 0.00231 | ~Tie |
| right_arm | 0.11800 | 0.11268 | 0.04158 | Overlapped |
| right_hand | 0.08622 | 0.06377 | 0.02894 | **Overlapped (26%)** |
| head | 0.05968 | 0.05788 | 0.03352 | Overlapped |

Chunk boundary discontinuity (mean |Δaction| at transitions):

| Body Part | Sequential | Overlapped | Reduction |
|-----------|-----------|------------|-----------|
| base | 0.01345 | 0.00289 | **78%** |
| right_arm | 0.17557 | 0.10676 | **39%** |
| right_hand | 0.11660 | 0.04775 | **59%** |
| head | 0.07299 | 0.04245 | **42%** |

Per-episode right_arm MAE (overlapped wins 5 of 6):

| Episode | Frames | Sequential | Overlapped | Winner |
|---------|--------|-----------|------------|--------|
| 317 | 313 | 0.029 | 0.033 | Sequential |
| 318 | 126 | 0.305 | 0.264 | Overlapped |
| 319 | 292 | 0.041 | 0.027 | Overlapped |
| 320 | 152 | 0.355 | 0.334 | Overlapped |
| 321 | 250 | 0.037 | 0.030 | Overlapped |
| 322 | 134 | 0.379 | 0.312 | Overlapped |

**Key findings:**
1. **Dead time nearly eliminated** — 1.9% vs 25.9%. Robot continuously executes model actions instead of stopping every cycle.
2. **Base velocity improved most** — 37.3% lower total MAE. Dead time zeros out base velocity; overlapped keeps the robot moving continuously.
3. **Gripper tracking dramatically improved** — 31.3% total MAE reduction, 26% active-frame improvement. The 2x re-planning rate gives the model more chances to commit to open/close decisions. Critical for grasp timing.
4. **Smoother chunk transitions across the board** — 39-78% less discontinuity at boundaries. More frequent, smaller course corrections instead of infrequent large ones.
5. **Active-frame model quality slightly better too** — even ignoring dead time, overlapped outperforms sequential on arm/hand/head. The 2x re-planning rate compensates for the slightly staler observations (skip=5 vs skip=4).
6. **Robust across diverse episodes** — 5/6 episodes favor overlapped. The exception (ep 317, long approach) has low MAE either way (0.029 vs 0.033).

### Trigger Timing Exploration: trigger@4/skip=4 vs trigger@3/skip=5

**Question:** The validated approach triggers inference at window step 3 (201ms), giving skip=5 and 55ms buffer. What if we trigger at step 4 (268ms) — exactly the halfway point of the 8-action window? This gives skip=4 (fresher actions) but tighter timing.

**Intuition:** Execute 4 actions, send observation, execute 4 more while inference runs. When the result arrives, the first 4 predicted actions are stale (they correspond to what just happened), so start at action[5] (1-indexed) = action[4] (0-indexed) → skip=4.

```
Variant A (current, validated):
  trigger@3: obs at 201ms, inference returns ~481ms, chunk ends 536ms
  Buffer: 55ms. Staleness: 335ms → skip=5
  Phase 0 correlation at k=5: base r=0.964, arm r=0.966, head r=0.983

Variant B (proposed):
  trigger@4: obs at 268ms, inference returns ~548ms, chunk ends 536ms
  Buffer: -12ms (tiny gap). Staleness: 268ms → skip=4
  Phase 0 correlation at k=4: base r=0.969, arm r=0.974, head r=0.988
```

**Tradeoff:**
- Variant B uses fresher observations (skip=4 vs 5 → higher correlation from Phase 0)
- Variant B has negative margin at 280ms latency (-12ms gap where robot holds position)
- In practice: 12ms is 0.18 frames — sub-perceptual. If actual latency is <268ms (plausible without server multitasking), the gap disappears entirely.
- The discrete frame simulation treats trigger_at=4 as 0-margin (4 frames trigger + 4 frames inference = 8 frames = n_exec). No gap in frame-domain.

**Test results (6 episodes, cached horizons, instant re-simulation):**

| Metric | trigger@3/skip=5 | trigger@4/skip=4 | Better |
|--------|-----------------|-----------------|--------|
| **Timing buffer** | 55ms | **-12ms (0 in frame domain)** | A (safer) |
| right_arm MAE (total) | 0.11957 | **0.11270** | **B (+5.7%)** |
| right_hand MAE (total) | 0.06256 | **0.06538** | A (+4.5%) |
| head MAE (total) | 0.06061 | **0.05757** | **B (+5.0%)** |
| base MAE (total) | 0.00303 | 0.00311 | ~Tie |
| right_arm MAE (active) | 0.11268 | **0.10567** | **B (+6.2%)** |
| right_hand MAE (active) | 0.06377 | 0.06664 | A (+4.5%) |
| head MAE (active) | 0.05788 | **0.05478** | **B (+5.4%)** |
| right_arm discontinuity | 0.10676 | **0.10263** | **B (+3.9%)** |
| right_hand discontinuity | 0.04775 | **0.04512** | **B (+5.5%)** |
| Per-episode wins | 5/6 | **6/6** | **B** |

**Verdict: Variant B (trigger@4/skip=4) wins on arm and head. Variant A wins on gripper.** The fresher observations from skip=4 help arm trajectory and head tracking. Gripper slightly prefers the extra timing margin of Variant A — possibly because the gripper decision benefits more from the guaranteed-on-time chunk transitions than from fresher observations.

**Recommendation: Go with trigger@4/skip=4.** The arm and head improvements matter more for manipulation than the small gripper regression. The 12ms theoretical gap is sub-frame and won't cause visible jitter — and if dedicated server latency is <268ms (likely without multitasking), there's no gap at all. The 6/6 per-episode win rate confirms this is the more robust choice.

**Updated overlapped parameters:**
```
n_exec = 8            # 8 actions per chunk = 536ms
trigger_at = 4        # fire inference after 4 actions (268ms into chunk)
first_skip = 4        # first chunk: 280ms latency → skip 4
subsequent_skip = 4   # subsequent: 268ms staleness → skip 4
```

---

## Phase 5: AH=4 Retrain — The Real Fix

### Why AH=4

AH=4 matches the inference cadence. Chunk duration (267ms) ≈ inference latency (270ms). This enables double-buffered inference with zero dead time and 4× more frequent course corrections.

| Metric | AH=16 | AH=4 |
|--------|-------|------|
| Chunk duration | 1.07s | 267ms |
| Dead time between chunks | 270ms | ~0ms (double-buffered) |
| Re-planning rate | 0.75 Hz | 3.7 Hz |
| Max observation staleness | 1.34s | ~540ms |
| Course correction frequency | Every 1.34s | Every 267ms |

**The model LEARNS shorter plans.** Unlike using `n_action_steps=4` with an AH=16 model (which causes "praying mantis" trajectory repetition), AH=4 training teaches the model to make coherent 267ms plans. Each 4-action chunk is a complete trajectory, not a truncated 16-step prediction.

### Double-Buffered (Overlapped) Inference

```
Timeline with AH=4, overlapped inference:

             ┌── chunk A (267ms) ──┐┌── chunk B (267ms) ──┐┌── chunk C ──
Actions:     │ a0  a1  a2  a3      ││ b0  b1  b2  b3      ││ c0  c1 ...
             └─────────────────────┘└─────────────────────┘└──────────

Inference:   ├══ obs+infer B (270ms) ══╡
                                       ├══ obs+infer C (270ms) ══╡
             t=0                  t=267                     t=534
                                  promote B                 promote C
                                  (3ms gap)                 (3ms gap)
```

**How it works:**
1. When chunk A starts playing, the inference thread captures an observation and fires inference for chunk B
2. ~270ms later, chunk B arrives as `_pending_chunk`
3. When chunk A exhausts (t=267ms), the pending chunk promotes → chunk B starts instantly
4. The inference thread sees the pending slot empty → fires again for chunk C
5. Seamless: ≤3ms gap (270ms inference - 267ms chunk = 3ms, imperceptible)

**Key architecture change:** The inference thread no longer waits for chunk exhaustion. Instead:
```python
# OLD (AH=16): wait for chunk to finish, then start inference
if has_chunk:
    remaining = chunk_duration - elapsed
    if remaining > 0:
        time.sleep(min(remaining, 0.05))
        continue

# NEW (AH=4): start inference as soon as pending slot is empty
with self._action_lock:
    has_pending = self._pending_chunk is not None
if has_pending:
    time.sleep(0.01)
    continue
# Capture obs immediately and fire inference
```

### Observation Staleness Budget

With AH=4 double-buffered:
- Observation captured at chunk N start (t=0)
- Inference takes ~270ms
- Result sits as pending for ~0ms (chunk exhausts at 267ms)
- Chunk N+1 starts executing from observation that is **267ms old**
- Chunk N+1 finishes at 534ms after observation capture
- But the actions only target 267ms from the observed state

**Why this is OK:** With AH=16, we needed post-exhaust observation because 1.07s of motion means the arm could be in a completely different position. With AH=4, only 267ms passes — the arm moves ~10-20 degrees at most. The model's 4-action trajectory from a 267ms-stale observation is still valid because the scene barely changed.

### Per-Body-Part Strategy (AH=4)

| Body Part | Latency Skip | Smoothing | Blending | Rationale |
|-----------|-------------|-----------|----------|-----------|
| Base velocity [0:6] | skip=1 | alpha=0.95 | No blend | 1 action = 67ms ≈ 25% of staleness. Low skip because only 4 actions total. |
| Back [6] | skip=0 | alpha=0.95 | Blend (2 steps) | Slow-moving, position target. |
| Arms [7:18] | skip=0 | alpha=0.95 | Blend (2 steps) | Critical for grasping. Servo profiles handle convergence. |
| Grippers [12,18] | skip=0 | threshold | No blend | Binary open/close decision. Threshold at 50% range. |
| Head [19:22] | skip=0 | alpha=0.95 | Blend (2 steps) | Tracking head. Smooth transitions matter for visual stability. |

### Temporal Ensembling (Advanced Option)

Temporal ensembling (from ACT paper) creates ultra-smooth transitions by maintaining a weighted average of overlapping action predictions. Requires `n_action_steps < action_horizon` so consecutive chunks overlap.

**How it would work with AH=8, n_steps=4:**
- Train with AH=8 (8-action trajectory, 534ms)
- Execute first 4 actions (267ms), then re-query
- Chunk N's actions[4..7] overlap with chunk N+1's actions[0..3]
- Blend: `final[t] = w_new * chunk_new[t] + w_old * chunk_old[t]`
- Weight: exponential decay favoring the fresher chunk

**Trade-off:** Requires AH=8 training (more complexity) and the "praying mantis" risk returns if the scene doesn't change enough in 267ms. Start with AH=4, full chunk execution. Revisit ensembling only if chunk boundary jitter is a problem.

### Phase 5 Inference Speed Requirement

For seamless double-buffering: inference must complete within one chunk duration (267ms).

| Server Config | Inference Time | Fits 267ms? | Gap |
|---------------|---------------|-------------|-----|
| Remote 3090, PyTorch | ~270ms | Barely (3ms gap) | 3ms — imperceptible |
| Remote 3090, torch.compile | ~200ms | Yes | 0ms |
| Remote 3090, torch.compile + AH=4 DiT reduction | ~150ms | Yes, with headroom | 0ms |

The AH=4 DiT reduction matters: `sa_embs` shrinks from ~17 tokens to ~5 tokens. DiT attention is O(n²) on tokens, so the action head runs significantly faster. Combined with torch.compile, inference should comfortably fit within 267ms.

---

## Phase 6: On-Device — Further Latency Reduction

Eliminates the client-server split entirely. Direct policy calls on the Jetson Orin AGX.

**Latency savings:**

| Overhead | Remote (current) | On-device |
|----------|------------------|-----------|
| WiFi round-trip | 2-10ms (jitter) | 0ms |
| JPEG encode (4 cameras) | ~2ms | 0ms |
| JPEG decode (server) | ~2ms | 0ms |
| ZMQ/msgpack serialize | ~2ms | 0ms |
| **Total saved** | **~10-20ms** | |

**The real win:** GPU pipeline parallelism. Backbone(N+1) runs on a CUDA stream while DiT(N) processes. Benchmarked at **22% speedup** on 3090. Combined with AH=4 DiT reduction and torch.compile:

| Optimization | Estimated Orin AGX | Fits 267ms? |
|---|---|---|
| Baseline PyTorch | ~450ms | No |
| + torch.compile | ~350ms | No |
| + pipeline parallelism | ~280ms | Barely |
| + AH=4 DiT reduction | ~220ms | Yes |
| + TRT backbone | ~160ms | Comfortably |

**Even with on-device, observation staleness remains ~150-220ms.** The architectural principles (latency skip, per-body-part treatment, double-buffering) apply regardless of where inference runs. On-device doesn't eliminate latency — it just shrinks it.

---

## How Other Teams Solve This

### ACT (Action Chunking with Transformers) — Stanford/Google

- CVAE outputs 100-step trajectory in one shot
- Executes 5-10 steps, re-queries with fresh observation
- **Temporal ensembling:** exponentially weighted average of overlapping chunks
- `final_action[t] = Σ w_k * chunk_k[t]` where w_k = exp(-m * age_k)
- Creates ultra-smooth transitions at the cost of responsiveness
- Works because CVAE is deterministic (same input → same output), so overlapping predictions agree

### Diffusion Policy — CMU/Toyota

- Diffusion model outputs multi-step trajectory
- Receding horizon: execute k steps, discard rest, re-query
- Model's short-term predictions are most accurate → only trust first k actions
- Smooth because re-planning is frequent and model is self-consistent
- Latency handled by fast inference on powerful GPUs (~50ms on A100)

### NVIDIA GR00T (canonical sim eval)

- `MultiStepWrapper`: execute `n_action_steps` from `action_horizon`, capture post-exhaust observation
- **No blending, no latency skip** — sim has zero inference latency
- Purely sequential: inference → execute → observe → inference
- Works in sim because physics is deterministic and inference is near-instant

### What they all share

1. **Model predicts trajectories** — coherent multi-step plans, not reactive single actions
2. **Only trust the near future** — execute k < N actions, discard far predictions, re-plan
3. **Frequent re-planning corrects drift** — stale observations cause bounded error that gets corrected at next cycle
4. **Physical smoothing absorbs discontinuities** — servo profiles, impedance control, low-pass filters
5. **Base/velocity gets special treatment** — latency compensation, decay, or separate control loop

---

## Tuning Protocol

### CSV-Based Validation

Enable `csv_log_path` in `groot_client.yaml`. The CSV logs at 100 Hz:
- `raw_*`: action before smoothing
- `smoothed_*`: action after EMA
- `final_*`: action as published
- `state_*`: robot state at publish time

### What to Look For

1. **Chunk boundary discontinuities:** Plot `final_right_shoulder_pitch` over time. Look for jumps > 0.1 rad at chunk boundaries (every 1.07s for AH=16, every 0.267s for AH=4).

2. **Base velocity staleness:** Plot `final_cmd_vel_lx` vs `state_cmd_vel_lx`. With latency skip disabled, the state should lag the command by ~270ms. With skip enabled, they should track more closely.

3. **Base overshoot:** Plot integrated base position (cumulative sum of `final_cmd_vel_lx` × dt). Compare to target approach distance. Overshoot = decay too low. Undershoot = decay too high.

4. **Arm trajectory coherence:** Plot shoulder/elbow trajectories across multiple chunks. Smooth curves = good. Sawtooth or oscillation = bad (possible trajectory repetition or blending fighting).

### A/B Testing Workflow

For each parameter change:
1. Record 3 runs with CSV enabled (same starting pose, same task)
2. Plot the target metric (discontinuity, overshoot, etc.)
3. Compare quantitatively (max discontinuity, mean absolute error)
4. Keep the better setting, move to next parameter

### Tuning Order

| Priority | Change | Metric | Risk | Phase 0 Status |
|----------|--------|--------|------|----------------|
| 1 | Universal `latency_skip=6` (all parts except gripper) | Trajectory tracking | Low | **VALIDATED** — r>0.95 for base/arm/head at k=6 |

| 3 | Gripper threshold (no skip) | Grasp timing | Low | **VALIDATED** — gripper r=0.87, skip unreliable |
| 4 | Re-enable chunk blending (steps=2) | Joint discontinuity at boundaries | Medium — may fight corrections | Unchanged |
| 5 | Gap decay (150ms coast) | Motion smoothness at chunk end | Medium — potential overshoot | Unchanged |
| 6 | Servo speed/accel tuning | Overall tracking responsiveness | Low — servo firmware enforces limits | Unchanged |

---

## Implementation Roadmap

```
NOW ──────────────────────────────────────────────────────────────►

Phase 0: Horizon Consistency Analysis ✅ COMPLETE (2026-02-16)
├── Built action_horizon_analysis.py (standalone, no ROS2)
├── Ran against inference server with 6 episodes (317-322)
├── Measured per-body-part correlation decay at k=1..15
├── RESULT: Universal skip viable — all parts r>0.91 at k=4
│   (280ms latency). Model is a trajectory planner.
└── Skip=4 for all channels including gripper (r=0.915).

Phase 0.5: Overlapped Execution Simulation ✅ COMPLETE (2026-02-16)
├── Built overlapped_execution_test.py (standalone, no ROS2)
├── Simulated sequential vs overlapped on 6 episodes (1267 frames)
├── RESULT: Overlapped wins on every metric:
│   Dead time 25.9% → 1.9%, re-plan 0.95→1.86 Hz,
│   base MAE +37%, gripper MAE +31%, boundary disc. -39 to -78%
├── Active-frame model quality also better (not just dead-time win)
└── Robust: overlapped wins 5 of 6 episodes.

Phase A: AH=16 Overlapped Implementation (no retraining) ← CURRENT
├── Implement overlapped inference in groot_client.py:
│   n_exec=8, trigger@step4, skip=4 (universal, all chunks)
├── Re-enable universal latency_skip (Phase 0 confirmed r>0.91+)

├── CSV validation on live robot
└── Remove dead parameter cleanup (they're alive again)

Phase B: AH=4 Training (less urgent — retraining, ~1 week)
├── Phase 0 showed AH=16 predictions are reliable long-horizon,
│   so AH=4 is for faster course correction (3.7 Hz vs 0.75 Hz),
│   NOT for prediction accuracy
├── Training config already updated (alfiebot_config.py, delta_indices=range(4))
├── Train AH=4 checkpoint on H200/3090
├── Open-loop eval with groot_open_loop_eval.py
├── Re-run horizon analysis with AH=4 model to confirm
│   shorter horizons still have good consistency at k=1..3
└── Verify server returns 4 actions per request

Phase C: AH=4 Client (code changes, days)
├── Double-buffered inference loop (pending slot trigger)
├── Update parameter defaults (chunk_size=4, n_steps=4, skip=1)
├── Watchdog timeout adjustment (0.7s)
├── CSV validation: chunk boundary smoothness
└── Live robot testing

Phase D: On-Device (Phase 6, weeks)
├── Benchmark on Orin AGX
├── DirectPolicyBridge (replaces ZMQ)
├── Pipeline parallelism
├── torch.compile + TRT backbone
└── Validate <267ms inference
```
