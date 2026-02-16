# Client-Server Smooth Execution Gameplan

How to achieve smooth, natural robot motion despite ~270ms round-trip inference latency.

---

## The Latency Problem

**The numbers:**
- Round-trip inference: ~270ms (3.7 Hz) — observation capture through ZMQ through model through response
- Training data rate: 15 FPS → 67ms per action step
- Actions per inference cycle: 270ms / 67ms ≈ **4 actions**
- Current model: AH=16 → 16 actions × 67ms = **1.07s per chunk**

**The core challenge:** Every action the robot executes is based on an observation captured ~270ms ago. By the time the last action in a 16-step chunk executes, the observation is **1.34s stale** (1.07s chunk + 0.27s inference). The robot is running blind on a prediction of what should happen based on what it saw over a second ago.

**Why it still works (and why other teams ship this):**

1. **The model predicts trajectories, not reactions.** During training, the model learns: "from this visual scene + robot state, here's the full motion trajectory for the next N steps." It implicitly learns dynamics — where the arm will be, how the base will coast, where the can will appear. The actions at step 10 already account for the fact that the arm moved during steps 1-9.

2. **Joint positions are absolute targets.** When the model says "shoulder pitch = -1.2 rad at step 5", the servo converges to that target regardless of when the command arrives. The servo's internal speed/acceleration profile handles the physical smoothing. A 270ms-stale observation means the servo starts tracking ~270ms late, but it still tracks to the right place.

3. **Base velocity is the sensitive channel.** Velocity integrates to position continuously — 270ms of stale velocity command means ~270ms × v meters of position error before correction. This is why base needs special treatment (latency skip, decay).

4. **Frequent re-planning corrects drift.** With AH=4 and overlapped inference, the model corrects course every ~267ms. Any error from observation staleness is bounded to one chunk duration before a fresh observation corrects it.

---

## Open Question: Action Horizon Temporal Consistency

**The key measurement we haven't made:** When the model predicts 16 steps into the future at time T, how well does predicted step k match the actual inference output at time T+k?

Formally: does `H[T][k] ≈ H[T+k][0]` — does the model's k-step-ahead prediction agree with what it would actually output if given a fresh observation k steps later?

**Why this matters for latency compensation:**
- At ~290ms RTT, any new chunk is stale by **~4.4 steps** (290ms / 67ms ≈ 4.4)
- `latency_skip_base` already assumes this consistency holds for the base — it reads from `action[idx+4]` instead of `action[idx]`
- But we've only applied this to base velocity. If horizon consistency holds for joints too, we could apply per-body-part latency skip across the board
- If it **doesn't** hold (e.g., arm predictions diverge quickly), then latency skip is only valid for base and joints need a different strategy

**What we expect:**
- **Base velocity** — likely **low** consistency. Base velocity is reactive (obstacle avoidance, approach corrections). The model may change its mind about base commands quickly based on fresh visual input.
- **Joint positions** — likely **high** consistency. Arm trajectories are smooth, continuous motions (reach, grasp). The model plans a smooth arc; step 5 of that arc should match what it predicts if re-queried at step 5.
- **Head** — likely **medium**. Head tracking follows the can/target. Smooth when the target is static, but could diverge if the base moves the viewpoint.
- **Grippers** — likely **high** for most of the trajectory (stays open), then **low** at the grasp moment (binary decision timing-sensitive).

**Phase-dependent behavior:**
- During **approach** (base moving, arms neutral): base consistency matters most
- During **manipulation** (base stopped, arms moving): joint consistency matters most
- The analysis should separate these phases to give actionable per-phase skip values

### Analysis Tool: `action_horizon_analysis.py`

A standalone script (no ROS2 dependency) that quantifies horizon consistency using recorded episodes.

**Method:**
1. Load an episode from the LeRobot dataset (reuses `load_episode`, `load_video_frames` from `groot_open_loop_eval.py`)
2. Run inference at **every frame** to get overlapping 16-step horizons (~500 frames × ~130ms = ~65s per episode)
3. For each horizon offset k=1..15, collect all pairs `(predicted=H[T][k], actual=H[T+k][0])` across all valid T
4. Compute per body part: Pearson correlation, MAE, RMSE
5. Generate decay curves and summary statistics

**Output:**
- **Decay curve plot** — correlation vs horizon step k, one line per body part. The "safe skip" is where correlation drops below a usable threshold (~0.9).
- **Per-body-part summary table** — at k=4 (~290ms) and k=5 (~335ms): correlation, MAE, RMSE for each group.
- **Heatmap** — body parts × horizon steps, colored by correlation coefficient.
- **Raw data CSV** — all horizons saved for offline analysis.

**CLI:**
```
python action_horizon_analysis.py \
    --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-index 3 \
    --host 192.168.50.201 --port 5555 \
    --save-dir /tmp/horizon_analysis/
```

**What the results inform:**

| Result | Implication |
|--------|------------|
| All body parts high correlation at k=4 | Universal latency skip works — skip all channels by 4 |
| Base low, joints high at k=4 | Current approach is correct — skip base only, joints play from action[0] |
| Everything low by k=3 | Latency skip is unreliable — need shorter action horizons (AH=4) or observation prediction |
| High correlation at k=8+ | Model is very temporally consistent — could use even larger skip or fewer re-queries |

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
| `base_velocity_decay` | 0.15 | Scale down base velocity to prevent approach overshoot | Dead — never applied |
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

**Parameter:** `latency_skip_base: 4` (270ms / 67ms ≈ 4). With n_action_steps=16, base effectively uses actions 4-15 (12 useful steps = 800ms of base trajectory).

**Validation:** Phase 0 horizon consistency analysis will confirm whether skip=4 is correct for base velocity and whether joints should also skip. If base correlation at k=4 is >0.9, skip=4 is well-justified. If it's <0.7, the model's base predictions are too reactive for skip to help — AH=4 becomes the priority.

### 2. Re-enable `base_velocity_decay`

**What:** `action[0:6] *= (1.0 - decay)` — scales down all base velocity commands.

**Why:** Even with latency skip, the model's base velocity predictions can overshoot the target position. A small decay factor trims approach distance without changing trajectory shape.

**Parameter:** Start at `base_velocity_decay: 0.15` (the previously tuned value). Increase if robot overshoots approach, decrease if robot doesn't reach the can.

### 3. Improve gap behavior

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

### AH=16 tuning priority order

1. **Re-enable latency_skip_base** (highest impact — fixes base velocity staleness)
2. **Re-enable base_velocity_decay** (second highest — fixes approach overshoot)
3. **Re-enable chunk blending** (smooth chunk transitions)
4. **Gap behavior** (optional — only if chunk boundary pause is the dominant issue)
5. **Servo tuning** (fine-tuning — try after the above are stable)

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

| Priority | Change | Metric | Risk |
|----------|--------|--------|------|
| 1 | Re-enable `latency_skip_base=4` | Base velocity tracking | Low — only affects base |
| 2 | Re-enable `base_velocity_decay=0.15` | Approach distance | Low — easily reversible |
| 3 | Re-enable chunk blending (steps=2) | Joint discontinuity at boundaries | Medium — may fight corrections |
| 4 | Gap decay (150ms coast) | Motion smoothness at chunk end | Medium — potential overshoot |
| 5 | Servo speed/accel tuning | Overall tracking responsiveness | Low — servo firmware enforces limits |

---

## Implementation Roadmap

```
NOW ──────────────────────────────────────────────────────────────►

Phase 0: Horizon Consistency Analysis (prerequisite, hours)
├── Build action_horizon_analysis.py (standalone, no ROS2)
├── Run against inference server with 3-5 episodes
├── Measure per-body-part correlation decay at k=1..15
├── Determine safe skip values per body part at 290ms latency
├── Results directly inform Phase A skip values and
│   whether Phase B (AH=4) is urgently needed
└── If joints are consistent at k=4+, universal skip is viable

Phase A: AH=16 Tuning (no retraining, days)
├── Re-enable latency_skip_base in _command_callback
│   (set skip value from Phase 0 analysis — expect 4)
├── Optionally enable latency_skip for joints if Phase 0
│   shows high consistency (>0.9 correlation at k=4)
├── Re-enable base_velocity_decay in _command_callback
├── Re-enable chunk blending (joints only)
├── CSV validation on robot
└── Remove dead parameter cleanup (they're alive again)

Phase B: AH=4 Training (retraining, ~1 week)
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
