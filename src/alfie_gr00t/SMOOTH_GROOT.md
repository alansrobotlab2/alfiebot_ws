# GR00T N1.6: Smoothing, Timing & Action Execution Analysis

Cross-example comparison of how GR00T N1.6 achieves smooth, accurate robot behavior
across three official deployment examples: **G1 WholeBodyControl**, **BEHAVIOR R1Pro**,
and **RoboCasa GR1 Tabletop Tasks**.

References:
- [examples/GR00T-WholeBodyControl/README.md](../../examples/GR00T-WholeBodyControl/README.md)
- [examples/BEHAVIOR/README.md](../../examples/BEHAVIOR/README.md)
- [examples/robocasa-gr1-tabletop-tasks/README.md](../../examples/robocasa-gr1-tabletop-tasks/README.md)
- [GR00T N1 Paper (arXiv:2503.14734)](https://arxiv.org/abs/2503.14734)

---

## 1. How the Model Generates Actions (Shared Across All Examples)

The action head is a **Diffusion Transformer (DiT)** using **Flow Matching** with
**Euler integration** over **4 denoising steps**:

```python
# gr00t/model/gr00t_n1d6/gr00t_n1d6.py:310-357
actions = torch.randn(batch_size, action_horizon, action_dim)   # pure noise
dt = 1.0 / num_inference_timesteps                               # 0.25

for t in range(num_inference_timesteps):                          # 4 iterations
    pred_velocity = model(actions, t, context)
    actions = actions + dt * pred_velocity                        # Euler step
```

Each forward pass produces the **full action horizon** in one shot — all timesteps
are refined together, which naturally produces temporally coherent (smooth) trajectories.
There is no step-by-step autoregressive generation.

**Paper-stated frequencies:**
- **System 2** (Eagle VL backbone): refreshes at **10 Hz** (cached between action queries)
- **System 1** (DiT action head): generates at **120 Hz** effective throughput
- Inference time: **63.9 ms** on L40 GPU (bf16), **31–58 ms** on RTX 5090 (TRT → PyTorch)

**Config defaults** (`gr00t/configs/model/gr00t_n1d6.py:57`):
```
action_horizon = 16
num_inference_timesteps = 4
num_timestep_buckets = 1000
```

---

## 2. Per-Example Comparison

| Parameter | G1 WBC | BEHAVIOR R1Pro | RoboCasa GR1 |
|-----------|--------|----------------|--------------|
| **Embodiment tag** | `unitree_g1` | `behavior_r1_pro` | `gr1` |
| **Dataset FPS** | **50 Hz** | **~20 Hz** | **20 Hz** |
| **Action horizon** (delta_indices) | 30 | 32 | 16 (model default) |
| **n_action_steps** (executed) | 20 | 8 | 8 |
| **Discarded tail** | 10 steps | 24 steps | 8 steps |
| **Chunk overlap** | 33% | **75%** | 50% |
| **Execution window** | 0.40 s | 0.40 s | 0.40 s |
| **Re-plan rate** | 2.50 Hz | 2.50 Hz | 2.50 Hz |
| **Action DOF** | 30 | 23 | 44 |
| **State DOF** | 50+ | 82 | 44 |
| **Arm control** | Relative joint | Relative joint | Motor-level |
| **Post-model smoothing** | InterpolationPolicy (rate-limited) | OmniGibson PD (Kp=150) + gripper "smooth" | None (direct) |
| **Lower body** | RL locomotion (GearWBC ONNX) | Holonomic base velocity | N/A (tabletop) |
| **Max episode steps** | 1440 | 2× human demo | 720 |
| **Video inputs** | 1 (ego_view) | 3 (head + 2 wrist) | env-dependent |
| **n_envs** (parallel eval) | 5 | 1 | 5 |

**Key observation:** All three examples land on the same **0.4 s execution window** and
**~2.5 Hz re-plan rate**, despite very different dataset FPS and action horizons. This
appears to be a deliberate design choice — the model re-plans every 400 ms regardless
of embodiment.

---

## 3. The Smoothing Stack

GR00T achieves smooth behavior through **up to 3 layers** of smoothing. Not all layers
are present in every example.

### Layer 1: Chunk Overlap (All Examples)

**File:** `gr00t/eval/sim/wrapper/multistep_wrapper.py`

The model predicts more steps than it executes. After executing `n_action_steps`,
the remainder is discarded and a fresh prediction is made from new observations.
Because consecutive predictions share context (the robot is in a state anticipated
by the previous prediction's tail), chunk boundaries are implicitly smooth.

```
Model predicts:  [a₀ a₁ a₂ ... a₂₉]         ← 30 steps (G1 example)
Execute:         [a₀ a₁ ... a₁₉]              ← first 20
Discard:                          [a₂₀ ... a₂₉] ← last 10 (overlap zone)
Next prediction: [b₀ b₁ b₂ ... b₂₉]         ← starts from state after a₁₉
                  ↑ b₀ should ≈ old a₂₀ because the model "saw" this future
```

| Example | Predicted | Executed | Overlap % | Re-plan Hz |
|---------|-----------|----------|-----------|------------|
| **G1 WBC** | 30 @ 50Hz | 20 | 33% | 2.50 |
| **BEHAVIOR** | 32 @ 20Hz | 8 | **75%** | 2.50 |
| **RoboCasa GR1** | 16 @ 20Hz | 8 | 50% | 2.50 |

**Tuning rule:** Lower `n_action_steps` → more overlap → smoother transitions but
higher compute cost (more frequent inference). The tradeoff at 50 Hz (G1):

| n_action_steps | Window | Re-plan Hz | Overlap |
|----------------|--------|------------|---------|
| 8 | 0.16 s | 6.25 Hz | 73% |
| 16 | 0.32 s | 3.13 Hz | 47% |
| 20 | 0.40 s | 2.50 Hz | 33% |
| 30 | 0.60 s | 1.67 Hz | 0% — no overlap |

### Layer 2: Rate-Limited Interpolation (G1 Only)

**File:** `external_dependencies/GR00T-WholeBodyControl/gr00t_wbc/control/policy/interpolation_policy.py`

Upper-body joints pass through a `PoseTrajectoryInterpolator` that enforces a
**maximum angular velocity** per joint:

```python
# Minimum transition time = max displacement / max velocity
pose_min_duration = np.max(np.abs(end_pose - pose) / max_change_rate)
```

This converts the discrete action waypoints into continuous, velocity-capped
trajectories queried at the current wall-clock time via `scipy.interp1d`. Joints
physically cannot jerk between chunk boundaries because the interpolator won't
allow velocities above `upper_body_max_joint_speed` (rad/s).

**Why only G1?** The G1 is a full humanoid with legs — jerky upper-body commands
destabilize the whole-body balance. The interpolation layer is a safety requirement
for the WBC stack, not just a nicety.

### Layer 3: Controller-Level Smoothing (Varies)

Each example relies on different low-level controllers that provide their own
implicit smoothing:

**G1 WBC:**
- Upper body: PD position control through the interpolated trajectory
- Lower body: RL locomotion policy (`G1GearWbcPolicy`, ONNX model) trained for
  stable gaits — inherently smooth output
- Coupling: forward kinematics compute torso orientation from upper body pose,
  fed to lower body balance controller

**BEHAVIOR R1Pro:**
- Arms: `JointController` with `motor_type="position"`, `pos_kp=150`
  - PD control provides natural damping between discrete action targets
  - `use_delta_commands=False` — absolute position targets
- Grippers: `MultiFingerGripperController` with `mode="smooth"`
  - OmniGibson's internal smooth interpolation for gripper open/close
- Base: `HolonomicBaseJointController` with velocity limits ±[0.75, 0.75, 1.0] m/s
  - Velocity capping prevents sudden base jerks

**RoboCasa GR1:**
- Direct motor commands — no explicit smoothing layer
- Smoothness relies entirely on chunk overlap (Layer 1) and the model's learned behavior

---

## 4. Training Data & FPS Relationship

The training data collection frequency directly determines the expected control
cadence during deployment:

| Example | Collection FPS | Source |
|---------|---------------|--------|
| G1 WBC | 50 Hz | `info.json: "fps": 50.0` |
| BEHAVIOR | ~20 Hz | `og_teleop_cfg.py:151` ("Assuming 20 fps") |
| RoboCasa GR1 | 20 Hz | `demo_data/gr1.PickNPlace/meta/info.json` |

**Why this matters:** The model learns the temporal dynamics of the training data.
If data is collected at 20 Hz, each action step corresponds to 50 ms of real time.
Deploying at a different frequency would stretch or compress the learned dynamics.

**Timing math:**
```
execution_window = n_action_steps / dataset_fps
re_plan_rate     = dataset_fps / n_action_steps
inference_budget = execution_window (must complete before next re-plan)
```

At all three deployment frequencies, inference (~30-60 ms) comfortably fits within
the 400 ms execution window, leaving ~340 ms of idle GPU time per cycle.

---

## 5. What's NOT in the Codebase

An exhaustive search across the entire codebase confirmed these common VLA smoothing
techniques are **absent**:

| Technique | Status | Notes |
|-----------|--------|-------|
| Temporal ensembling | Not implemented | Weighted average of overlapping predictions (used in ACT) |
| Exponential moving average | Not implemented | EMA smoothing of successive action outputs |
| Action blending at boundaries | Not implemented | Crossfade between end of chunk N and start of chunk N+1 |
| Control frequency decimation | Not implemented | Running controller faster than policy |
| Kalman filtering on actions | Not implemented | State estimation for action smoothing |

GR00T relies instead on:
1. Flow matching's inherent temporal coherence (all timesteps refined together)
2. Chunk overlap (predict more than you execute)
3. Embodiment-specific post-processing (interpolation, PD control, RL locomotion)

---

## 6. Implications for Alfie Deployment

Alfie's current config uses `action_horizon=50` at 20 Hz data collection.

**Chunk overlap scenarios at 20 Hz:**

| n_action_steps | Window | Re-plan Hz | Overlap | Latency budget |
|----------------|--------|------------|---------|----------------|
| 4 | 0.20 s | 5.00 Hz | 92% (46/50) | comfortable |
| 8 | 0.40 s | 2.50 Hz | 84% (42/50) | very comfortable |
| 12 | 0.60 s | 1.67 Hz | 76% (38/50) | plenty |
| 16 | 0.80 s | 1.25 Hz | 68% (34/50) | generous |

**Recommendations:**
1. Start with `n_action_steps=8` (matches BEHAVIOR and RoboCasa defaults)
2. Implement a rate-limited interpolation layer (like G1's `InterpolationPolicy`)
   between the model output and motor commands — this is the single most impactful
   smoothing mechanism for real hardware
3. Consider implementing **temporal ensembling** as an enhancement beyond what NVIDIA
   provides — weighted averaging of overlapping predictions could further smooth
   chunk transitions
4. The 84% overlap with n_action_steps=8 is already very conservative; if the model
   runs well, try increasing to 12 or 16 for lower compute cost

---

## 7. Evaluation Commands Reference

### G1 WholeBodyControl

```bash
# Server
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path nvidia/GR00T-N1.6-G1-PnPAppleToPlate \
    --embodiment-tag UNITREE_G1 \
    --use-sim-policy-wrapper

# Client
gr00t/eval/sim/GR00T-WholeBodyControl/GR00T-WholeBodyControl_uv/.venv/bin/python \
    gr00t/eval/rollout_policy.py \
    --n_episodes 10 \
    --max_episode_steps=1440 \
    --env_name gr00tlocomanip_g1_sim/LMPnPAppleToPlateDC_G1_gear_wbc \
    --n_action_steps 20 \
    --n_envs 5
```

### BEHAVIOR R1Pro

```bash
# Server
uv run gr00t/eval/run_gr00t_server.py \
    --model-path nvidia/GR00T-N1.6-BEHAVIOR1k \
    --embodiment-tag BEHAVIOR_R1_PRO \
    --use-sim-policy-wrapper

# Client
uv run python gr00t/eval/rollout_policy.py \
    --n_episodes 10 \
    --policy_client_host 127.0.0.1 \
    --policy_client_port 5555 \
    --max_episode_steps=999999999 \
    --env_name sim_behavior_r1_pro/turning_on_radio \
    --n_action_steps 8 \
    --n_envs 1
```

### RoboCasa GR1 Tabletop

```bash
# Server
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path nvidia/GR00T-N1.6-3B \
    --embodiment-tag GR1 \
    --use-sim-policy-wrapper

# Client
gr00t/eval/sim/robocasa-gr1-tabletop-tasks/robocasa_uv/.venv/bin/python \
    gr00t/eval/rollout_policy.py \
    --n_episodes 10 \
    --policy_client_host 127.0.0.1 \
    --policy_client_port 5555 \
    --max_episode_steps=720 \
    --env_name gr1_unified/PnPBottleToCabinetClose_GR1ArmsAndWaistFourierHands_Env \
    --n_action_steps 8 \
    --n_envs 5
```

---

## 8. Key Takeaways

1. **All three examples converge on a ~0.4 s execution window / 2.5 Hz re-plan rate**
   despite different dataset FPS and action horizons. This is the sweet spot.

2. **Chunk overlap is the universal smoothing mechanism** — every example predicts more
   than it executes. BEHAVIOR is most aggressive (75% overlap), G1 least (33%).

3. **The model itself produces smooth trajectories** thanks to flow matching — all
   action timesteps are refined simultaneously through 4 Euler steps, not generated
   sequentially. This is fundamentally different from autoregressive action generation.

4. **Post-model smoothing is embodiment-specific** and ranges from sophisticated
   (G1's rate-limited interpolation + RL locomotion) to nonexistent (RoboCasa GR1's
   direct motor commands).

5. **Common VLA smoothing techniques (temporal ensembling, EMA, action blending) are
   absent** from the codebase — an opportunity for enhancement in custom deployments.

6. **Inference latency is never the bottleneck** — at 31–64 ms per inference vs 400 ms
   execution windows, there's plenty of headroom.
