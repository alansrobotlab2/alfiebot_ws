# GR00T G1 Whole-Body Control: Action Smoothing Analysis

Analysis of the server/client architecture and the 3-layer action smoothing pipeline
used when running the GR00T N1.6 policy on the Unitree G1 robot.

Reference: [examples/GR00T-WholeBodyControl/README.md](../../examples/GR00T-WholeBodyControl/README.md)

---

## Server/Client Commands

**Server** (Terminal 1 — runs the neural network on GPU):

```bash
uv run python gr00t/eval/run_gr00t_server.py \
    --model-path nvidia/GR00T-N1.6-G1-PnPAppleToPlate \
    --embodiment-tag UNITREE_G1 \
    --use-sim-policy-wrapper
```

**Client** (Terminal 2 — runs the sim environment + WBC stack):

```bash
gr00t/eval/sim/GR00T-WholeBodyControl/GR00T-WholeBodyControl_uv/.venv/bin/python \
    gr00t/eval/rollout_policy.py \
    --n_episodes 10 \
    --max_episode_steps=1440 \
    --env_name gr00tlocomanip_g1_sim/LMPnPAppleToPlateDC_G1_gear_wbc \
    --n_action_steps 20 \
    --n_envs 5
```

The server wraps the model in `Gr00tSimPolicyWrapper` and listens on ZMQ port 5555.
The client connects via `PolicyClient`, sends observations, receives action chunks,
and steps the sim environment with WBC applied.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│  SERVER (GPU)                                                       │
│                                                                     │
│  Gr00tPolicy  ──►  Gr00tSimPolicyWrapper  ──►  PolicyServer (ZMQ)  │
│  (DiT model)       (format adapter only)       (port 5555)         │
└──────────────────────────────────────┬──────────────────────────────┘
                                       │ action chunk (30 steps)
                                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│  CLIENT (CPU / Sim)                                                 │
│                                                                     │
│  PolicyClient ──► MultiStepWrapper ──► WholeBodyControlWrapper      │
│  (ZMQ)            (execute 20/30)      │                            │
│                                        ├─► InterpolationPolicy      │
│                                        │   (upper body smoothing)   │
│                                        └─► G1GearWbcPolicy          │
│                                            (lower body RL loco)     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Time Horizon

The G1 sim dataset runs at **50 Hz** (see `info.json`: `"fps": 50.0`).

| Parameter | Steps | Time |
|-----------|-------|------|
| **Action horizon** (model predicts) | 30 | **0.6 s** |
| **n_action_steps** (executed) | 20 | **0.4 s** |
| **Discarded tail** (overlap) | 10 | 0.2 s |
| **Re-query interval** | — | every 0.4 s (2.5 Hz planning) |

So the model looks **0.6 seconds into the future** on each inference call, but only
the first 0.4 seconds of that plan are actually sent to the robot. After 0.4 s, the
model is re-queried with fresh observations and produces a new 0.6 s plan. The 0.2 s
overlap between consecutive chunks is what provides implicit smoothing at boundaries.

---

## Layer 1: Chunk Overlap (MultiStepWrapper)

**File:** [`gr00t/eval/sim/wrapper/multistep_wrapper.py`](../../gr00t/eval/sim/wrapper/multistep_wrapper.py)

The model predicts an action chunk of **30 steps** (the full UNITREE_G1 action horizon,
defined by `delta_indices` in the embodiment config). The client only executes
**`n_action_steps`** of those before re-querying the model with fresh observations.

```python
# multistep_wrapper.py:249-263 — step() executes n_action_steps from the chunk
def step(self, action):
    for step in range(self.n_action_steps):
        act = {}
        for key, value in action.items():
            act[key] = value[step, :]
        observation, reward, done, truncated, info = super().step(act)
        ...
```

With `--n_action_steps 20`, the first 20 of 30 predicted actions are executed, then
a new prediction is made. The remaining 10 predicted steps are discarded. This overlap
means each new prediction starts from a state that's already been "seen" by the tail of
the previous chunk, providing implicit smoothing at chunk boundaries.

**Tuning:** Lower `n_action_steps` = more frequent re-planning = smoother but slower.
The README uses 20 for sim; real-robot deployments may use 8-16 for tighter control.

At 50 Hz, the tradeoffs look like:

| n_action_steps | Execution window | Re-plan rate | Overlap |
|----------------|------------------|--------------|---------|
| 8 | 0.16 s | 6.25 Hz | 0.44 s (73%) |
| 16 | 0.32 s | 3.13 Hz | 0.28 s (47%) |
| 20 | 0.40 s | 2.50 Hz | 0.20 s (33%) |
| 30 | 0.60 s | 1.67 Hz | 0.00 s (0%) — no overlap |

---

## Layer 2: InterpolationPolicy (Upper Body Smoothing)

**File (external submodule):** `external_dependencies/GR00T-WholeBodyControl/gr00t_wbc/control/policy/interpolation_policy.py`
**GitHub:** [NVlabs/GR00T-WholeBodyControl/.../interpolation_policy.py](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/gr00t_wbc/control/policy/interpolation_policy.py)

This is the primary explicit smoothing layer. Upper-body joints (arms, hands) pass
through a `PoseTrajectoryInterpolator` that uses rate-limited linear interpolation
via `scipy.interp1d`.

When a new waypoint (action target) arrives, the interpolator computes:

```python
# Minimum transition time based on max joint velocity
pose_min_duration = np.max(np.abs(end_pose - pose) / max_change_rate)
```

This ensures joints never exceed a configured maximum angular velocity, converting
the discrete action chunks into continuous, velocity-capped trajectories. The
`get_action()` method queries the interpolated trajectory at the current monotonic
timestamp.

**Key parameters:**
- `upper_body_max_joint_speed` — max rad/s per joint (configured in `BaseConfig`)
- `interpolation_garbage_collection_time` — how long to keep old waypoints

---

## Layer 3: Decoupled Whole-Body Control

**File (external submodule):** `external_dependencies/GR00T-WholeBodyControl/gr00t_wbc/control/policy/g1_decoupled_whole_body_policy.py`
**GitHub:** [NVlabs/GR00T-WholeBodyControl/.../g1_decoupled_whole_body_policy.py](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/gr00t_wbc/control/policy/g1_decoupled_whole_body_policy.py)

The `G1DecoupledWholeBodyPolicy` splits control into two independent subsystems:

| Subsystem | Joints | Policy | Smoothing |
|-----------|--------|--------|-----------|
| Upper body | arms, hands | `InterpolationPolicy` | Rate-limited interpolation (Layer 2) |
| Lower body | legs, waist, navigation | `G1GearWbcPolicy` (ONNX RL) | RL policy trained for smooth locomotion |

The coupling signal between them is **torso orientation**: forward kinematics compute
the torso roll-pitch-yaw from the upper body's interpolated pose, and this feeds into
the lower body's balance controller.

**Factory:** [`gr00t_wbc/control/policy/wbc_policy_factory.py`](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/main/gr00t_wbc/control/policy/wbc_policy_factory.py)

```python
# Default: upper body uses interpolation, lower body uses gear_wbc
upper_body_policy_type = wbc_config.get("upper_body_policy_type", "interpolation")
# Options: "interpolation" (rate-limited) or "identity" (passthrough)
```

---

## What Gr00tSimPolicyWrapper Does NOT Do

**File:** [`gr00t/policy/gr00t_policy.py:420-617`](../../gr00t/policy/gr00t_policy.py)

Despite the name, `Gr00tSimPolicyWrapper` performs **no action smoothing**. It is
purely a format adapter that translates between:

- **Sim format** (flat keys): `video.ego_view`, `state.left_arm`, `action.left_arm`
- **Policy format** (nested dicts): `observation["video"]["ego_view"]`

The `--use-sim-policy-wrapper` flag on the server just enables this key remapping.

---

## UNITREE_G1 Embodiment Config

**File:** [`gr00t/configs/data/embodiment_configs.py:13-90`](../../gr00t/configs/data/embodiment_configs.py)

| Property | Value |
|----------|-------|
| Action dimensions | 30 total |
| Action horizon | 30 steps (`delta_indices = list(range(30))`) |
| Arms (left + right) | 7 + 7 DoFs, RELATIVE joint commands |
| Hands (left + right) | 1 + 1 DoFs, ABSOLUTE (binary gripper) |
| Waist | 1 DoF, ABSOLUTE |
| Base height | 1 DoF, ABSOLUTE |
| Navigate command | 2 DoFs, ABSOLUTE (x, y velocity) |
| Video | Single `ego_view` camera |
| State groups | 7: left/right leg, waist, left/right arm, left/right hand |

---

## Open Loop Eval (Pretrained Model)

To validate the pretrained model without the WBC stack:

```bash
CUDA_VISIBLE_DEVICES=0 uv run python gr00t/eval/open_loop_eval.py \
    --dataset-path examples/GR00T-WholeBodyControl/PhysicalAI-Robotics-GR00T-X-Embodiment-Sim/unitree_g1.LMPnPAppleToPlateDC \
    --embodiment-tag UNITREE_G1 \
    --model-path nvidia/GR00T-N1.6-G1-PnPAppleToPlate \
    --traj-ids 0 \
    --action-horizon 16 \
    --denoising-steps 4 \
    --save-plot-path ./g1_pretrained_open_loop_eval.png
```

Results (trajectory 0, 200 steps):
- **MSE:** 0.00135
- **MAE:** 0.00567

---

## Key Takeaways

1. **The model itself outputs raw action chunks** with no smoothing — just 30 future
   joint targets from the DiT action head.

2. **Chunk overlap** (`n_action_steps < action_horizon`) provides planning-level
   smoothing by re-querying the model before the full chunk is consumed.

3. **InterpolationPolicy** is the critical safety layer for real robot deployment —
   it velocity-limits joint transitions so the robot never jerks between chunk
   boundaries. This is configured via `upper_body_max_joint_speed`.

4. **The lower body RL controller** (GearWBC) handles its own smoothing implicitly
   through the trained locomotion policy, which outputs stable walking gaits.

5. For deployment on a different robot (e.g., Alfie), you would need to implement
   your own equivalent of layers 2 and 3 — at minimum a rate-limited interpolator
   between the model's action chunks and your motor commands.
