# RTC (Real-Time Chunking) Freeze+Inpaint Analysis

## Problem

GR00T N1.6 uses flow matching to generate 16-step action chunks. Each inference starts from random noise and denoises to a clean trajectory. Because the noise seed differs between inferences, overlapping predictions for the same future timestep disagree — typically by 0.02-0.05 rad on arm joints and up to 0.07 rad on head joints. At chunk boundaries this disagreement manifests as discontinuities (jitter, popping).

## RTC Approach

Inspired by the Real-Time Chunking paper, we freeze the first `d` actions of each new chunk to match the tail of the previously committed chunk. During the flow matching Euler integration loop, after each denoising step we replace frozen indices with their noise-schedule target:

```
frozen_target_t = (1 - tau) * z_frozen + tau * a_committed_normalized
```

where `tau = (t+1) / num_steps`, `z_frozen` is the saved initial noise at frozen positions, and `a_committed_normalized` is the previous chunk's tail actions normalized to model space (min-max to [-1,1], zero-padded from 22D to 128D to match the model's internal action dimension).

A soft exponential blend zone (2 actions wide) at the freeze boundary prevents hard seams.

This is implemented as a monkey-patch on `action_head.get_action_with_features` in the inference server, activated with `--rtc --rtc-freeze-steps 4`. The client sends `prev_actions` (physical units) in the ZMQ observation payload; the server normalizes and pads them before injection.

## Methodology

- **Dataset**: alfiebot.CanDoChallenge episode 0, first 12 seconds (180 frames at 15 FPS)
- **Server**: RTX 3090, PyTorch backend (not TRT), varying denoising steps
- **Client**: Jetson Orin NX, realistic cadence (next inference fires only after previous completes)
- **Metric**: Mean standard deviation of overlapping predictions at the same timestep, measured at frames with depth >= 3 (or >= 5 for 2-step where overlap is deeper)
- **RTC config**: freeze_steps=4, blend_zone=2
- Each configuration was run twice (RTC on, RTC off) back-to-back on the same warmed server

## Results

### Operating Parameters

| Denoising Steps | Latency | Inference Rate | Max Overlap Depth |
|---|---|---|---|
| 2 | 208-211 ms | 5.0 Hz | 6 |
| 4 | 246-249 ms | 3.8 Hz | 4 |
| 8 | 320-325 ms | 3.0 Hz | 4 |
| 16 | 472-476 ms | 2.1-2.2 Hz | 3 |

RTC adds ~3ms latency overhead (negligible).

### Prediction Spread (mean_std, radians) — Active Joints Only

#### Right Arm

| Joint | 2-step | 2+RTC | 4-step | 4+RTC | 8-step | 8+RTC | 16-step | 16+RTC |
|---|---|---|---|---|---|---|---|---|
| r_shoulder_yaw | .00692 | .00682 | .00706 | .00717 | .00811 | **.00710** | .00861 | **.00727** |
| r_shoulder_pitch | .01757 | .01676 | .01729 | **.01615** | .01964 | **.01903** | .01641 | **.01530** |
| r_elbow_pitch | .03082 | .02891 | .02733 | **.02308** | .02297 | **.02186** | .00441 | **.00304** |
| r_wrist_pitch | .02778 | .02827 | .03024 | **.02700** | .02869 | .03315 | .02484 | .02449 |
| r_wrist_roll | .00865 | .00847 | .00877 | **.00732** | .00964 | **.00890** | .00797 | **.00718** |

#### Head

| Joint | 2-step | 2+RTC | 4-step | 4+RTC | 8-step | 8+RTC | 16-step | 16+RTC |
|---|---|---|---|---|---|---|---|---|
| head_yaw | .04640 | .07067 | .04125 | **.02754** | .04475 | **.02941** | .04122 | **.02974** |
| head_pitch | .01976 | .02526 | .02182 | **.01604** | .02492 | **.01871** | .02404 | **.01120** |
| head_roll | .00713 | .01332 | .00714 | **.00517** | .00726 | **.00473** | .00714 | **.00442** |

#### Base Velocity

| Joint | 2-step | 2+RTC | 4-step | 4+RTC | 8-step | 8+RTC | 16-step | 16+RTC |
|---|---|---|---|---|---|---|---|---|
| cmd_vel_az | .02192 | .03382 | .02554 | **.01591** | .02301 | **.01583** | .01477 | .01465 |

**Bold** = RTC improved over baseline at same step count.

### RTC Improvement by Denoising Steps (% change in mean_std)

| Joint | 2-step | 4-step | 8-step | 16-step |
|---|---|---|---|---|
| r_shoulder_pitch | -5% | **-7%** | -3% | -7% |
| r_elbow_pitch | -6% | **-16%** | -5% | -31% |
| r_wrist_roll | -2% | **-17%** | -8% | -10% |
| head_yaw | +52% | **-33%** | -34% | -28% |
| head_pitch | +28% | **-26%** | -25% | -53% |
| head_roll | +87% | **-28%** | -35% | -38% |
| cmd_vel_az | +54% | **-38%** | -31% | -1% |

## Conclusions

1. **RTC at 2 denoising steps is harmful.** With only 2 Euler iterations the model cannot adapt its free predictions around the frozen constraints. The abrupt clamping distorts the velocity field, increasing spread on most joints.

2. **4 denoising steps is the minimum for RTC to work.** At 4 steps the model has enough iterations to condition its free predictions on the frozen prefix. Head yaw spread drops 33%, head pitch 26%, rotational velocity 38%.

3. **4 steps + RTC is the sweet spot for real-time use.** It achieves spread reduction comparable to 8-step baseline while running at 3.8 Hz (249ms latency) instead of 3.0 Hz (320ms). You get 8-step quality at 4-step cost.

4. **More steps continue to help but with diminishing returns.** 8-step and 16-step RTC show the best absolute numbers, but the latency cost (320ms, 476ms) reduces inference rate to 3.0 and 2.1 Hz respectively, shrinking the execution window.

5. **RTC latency overhead is negligible (~3ms).** The inpainting operation (replacing frozen indices after each Euler step) is a trivial tensor operation compared to the DiT forward pass.

## Recommendation

Run the server with `--denoising-steps 4 --rtc --rtc-freeze-steps 4` for production inference. This provides:
- 249ms inference latency (3.8 Hz)
- 25-38% reduction in inter-chunk prediction spread on head and velocity joints
- Near-zero latency overhead vs vanilla 4-step

If latency budget permits (e.g., on a faster GPU or with backbone pipelining recovering the overhead), `--denoising-steps 8 --rtc` gives further improvement.

## Ground Truth Analysis: Arm Jitter During Hold-Still Phases

Model output shows noticeable jitter on right arm joints (shoulder yaw/pitch, elbow pitch) during the first several seconds of inference, when the arm should be held steady. Analysis of the ground truth training data for episode 0 rules out data quality as the cause.

### Training Labels Are Clean

The parquet action labels for right arm joints are **perfectly constant** for the first 209 frames (13.9s):

| Joint | Held Value (rad) | Std (frames 0-208) | First Movement |
|---|---|---|---|
| right_shoulder_yaw | 0.00000 | 0.00000 | Frame 210 (14.0s) |
| right_shoulder_pitch | 0.10738 | 0.00000 | Frame 209 (13.9s) |
| right_elbow_pitch | -1.48489 | 0.00000 | Frame 210 (14.0s) |
| right_wrist_pitch | -0.14419 | 0.00000 | Frame 209 (13.9s) |
| right_wrist_roll | 0.00000 | 0.00000 | Frame 210 (14.0s) |

The episode structure is: head scans + base rotates (0-8s) → base drives forward (10-13s) → arm reaches/picks (14s+). Arm commands are dead-flat throughout the entire approach phase. Actions are recorded from `servo_state[i].target_location` (commanded positions), not feedback.

### Root Cause: Model Inference Noise

The arm jitter is 100% model inference noise — the denoising process starts from random noise and must converge to a constant value. This is inherently harder for flow matching than smooth trajectories, because constant targets provide no gradient information to guide denoising.

The observation that jitter increases during base movements suggests **cross-attention leakage** in the DiT: base velocity changes create attention patterns in the shared 128D latent space that perturb the arm joint dimensions.

### Mitigations

1. **RTC freeze+inpaint** (implemented) — clamps the first 4 actions to the previous chunk's tail, directly suppressing hold-still noise. The 4-step RTC results showed 16% elbow pitch spread reduction.
2. **Action smoothing** (already in action_publisher) — dampens jitter at execution time via EMA/interpolation.
3. **Per-joint deadband** — suppress commanded deltas below a threshold (e.g., 0.005 rad) in the action publisher. If the joint isn't supposed to move, small perturbations get zeroed out.
4. **More training data** with held-still arms would strengthen the model's ability to predict constant trajectories.

## Implementation

- Server: `groot_inference_server.py` — `--rtc` flag installs monkey-patch on `action_head.get_action_with_features`
- Client: `zmq_client.py` — `send_observation(prev_actions=...)` sends committed actions in wire format
- Test tool: `ensemble_characterization.py --rtc` — A/B comparison with cached horizon replay
