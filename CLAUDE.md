# Environment ---

This is Alfie, a humanoid robot with:
 - two, 6dof arms, each with a 1dof gripper
 - 1dof back that raises and lowers the arms
 - 3dof neck
 - meccanum base
 - stereo cameras
 - respeaker microphone with angle sound source
 - amplified speaker

The alfiebot platform runs on:
 - Jetson AGX Orin 64gb developer kit (Gen2; Gen1 was an Orin NX 16gb)
 - Ubuntu 24.04
 - Jetpack 7.2.1 (L4T r39.2.1)
 - ROS 2 Jazzy (micro-ROS agent on the jazzy branch; Pico firmware builds with board_microros_distro = jazzy)


We are actively working on an nvidia gr00t n1.7 solution to run on alfie to complete a challenge to identify, localize, pick up, then set down a soda can
https://github.com/NVIDIA/Isaac-GR00T

We currently have 200 episodes with 2 tasks, about 100 episodes per task.

The episodes were recorded in varying environments and lighting conditions, with the soda can placed anywhere within 1 to 3 feed anywhere around the robot.

Both the groot server and the groot client run locally on the AGX Orin; there is no separate GPU workstation. The client talks to the server over localhost TCP.


# GR00T N1.7 Architecture ---

## Client-Server Split
The GR00T inference system is split into two processes, both on the Orin:
 - **Server** (`127.0.0.1`, REQ/REP 5555, async PUSH/PULL 5556/5557): Runs the GR00T model. Receives observations over ZMQ, returns action chunks (horizon = checkpoint `delta_indices`: 16 for the deployed `checkpoints/alfie/checkpoint-20000`; `alfiebot_config.py` targets 32 for the next retrain). Run it per `src/alfie_gr00t/ORIN_AGX_SERVER.md`. With `--trt-engine-dir` it loads Isaac-GR00T's full-pipeline INT8 TensorRT engines (`~/Isaac-GR00T/gr00t_trt_deployment/alfie_int8_rtc/engines`, RTC-capable, fused 2/4-step; other step counts fall back to per-step engines); must run from `~/Isaac-GR00T/.venv`. GPU image preprocessing is on by default.
 - **Client**: ROS2 node that collects camera images + robot state, sends them to the server via ZMQ, and publishes actions to the robot.

The server can run as either:
 - A standalone Python script: `scripts/groot_inference_server.py` (typical; runs in the Isaac-GR00T environment, TCP only)
 - A ROS2 node via the launch file (for on-device inference)

## Key Components
 - `nodes/groot_client.py` — ROS2 client node. Collects observations, sends to server, publishes actions at 100Hz.
 - `core/observation_bridge.py` — Subscribes to 4 camera topics + robot state, synchronizes them, center-crops/resizes images to 352x256 (`core/image_utils.fit_image`, no-op at native size), packages as JPEG bytes for ZMQ.
 - `core/action_publisher.py` — Receives action chunks from the server, steps through them at the training rate (`training_fps`, 15 FPS), applies smoothing and safety limits, publishes servo commands at 100Hz.
 - `core/zmq_client.py` — ZMQ REQ/REP client for communicating with the server.
 - `scripts/groot_inference_server.py` — Standalone inference server. Loads the GR00T checkpoint, receives JPEG images + state via ZMQ msgpack, runs inference, returns raw unnormalized actions.
 - `scripts/hybrid_image_test.py` — Diagnostic tool that sends saved images through ZMQ to isolate visual domain gap issues.
 - `scripts/ab_trial.py` — Runs one robot trial: moves arms + head to the pick-task start pose through the groot mux inputs, activates the client, deactivates after `--duration`.
 - `scripts/rtc_ab_analysis.py` — Compares live runs on raw policy output (seam jump, jerk vs demonstrations, RTT, late %, hold %) from the client's CSV + `_chunks.jsonl` logs.

## Communication Protocol (ZMQ msgpack)
Client sends: `{ 'left_wide': <jpeg_bytes>, 'right_wide': <jpeg_bytes>, ..., 'state': <22D float list>, 'language': <task string> }`
Server returns: `{ 'actions': <Hx22 float array> }` (H = checkpoint horizon, 16 for checkpoint-20000). With RTC the client adds `'prev_actions': <Kx22>` (actions already committed for the K steps after the observation) and `'rtc_frozen_steps': <int>`; see `core/rtc_prefix.py`.
Ping: Client sends `{'ping': True}`, server returns `{'pong': True}`.

## Config Files
 - `config/groot_client.yaml` — Client ROS2 parameters: transport, server address, inference FPS, action chunking, smoothing, safety limits, task description, debug settings.
 - `config/groot_server.yaml` — Server ROS2 parameters: transport, bind address, model checkpoint path, TensorRT toggle, mock mode, replay mode.
 - `launch/groot_inference.launch.py` — Launches the client only. All parameters come from `groot_client.yaml`; a launch arg overrides the yaml only when passed.

## Training Pipeline
 - Training config: `alfiebot_config.py` — Defines modality config (4 cameras, 22D state/action, 32-step horizon; RELATIVE back/arm/head actions, ABSOLUTE base velocity and grippers — N1.7 guidance; `launch_finetune.py` forces `use_relative_action=True`).
 - Data conversion: `scripts/rosbag_to_groot.py` — Converts ROS2 rosbags to LeRobot format (MP4 videos + parquet state/action data).
 - Datasets: one folder per dataset under `data/<dataset>/` (`alfie_gr00t/dataset_paths.py`): `demonstrations/` holds raw bags from `nodes/data_recorder.py`, `lerobot/` holds the converted GR00T LeRobot v2 output (pass this as `--dataset-path`). The recorder, converter and episode manager all take `--dataset NAME` / `dataset:=NAME` (env `ALFIE_DATASET`, `ALFIE_DATA_ROOT`). The soda-can challenge data is `data/can_do_challenge/` — 200 episodes, 2 tasks.
 - Episode review: `ros2 run alfie_gr00t episode_manager --ros-args -p dataset:=<dataset>` serves a React UI on port 7880 to convert new bags, play the 4 cameras in sync, relabel tasks (keys 1-9) and delete episodes (`alfie_gr00t/episode_manager/`, see DATA_COLLECTION.md).
 - `meta/stats.json` / `relative_stats.json` are NOT written by the converter; run `gr00t/data/stats.py --dataset-path data/<dataset>/lerobot --embodiment-tag NEW_EMBODIMENT --modality-config-path alfiebot_config.py` after converting and whenever action `delta_indices` change.
 - Fine-tuning: uses `Isaac-GR00T/gr00t/experiment/launch_finetune.py`.
 - Checkpoints saved to `Isaac-GR00T/alfie-gr00t/` with `processor_config.json` containing normalization stats and action config. The server loads config from the checkpoint, NOT from `alfiebot_config.py`.

## 22D State/Action Vector
`[0:6]` base velocity (linear xyz, angular xyz), `[6]` back height, `[7:12]` left arm (5 joints), `[12]` left gripper, `[13:18]` right arm (5 joints), `[18]` right gripper, `[19:22]` head (3 joints).

## Image Pipeline
Camera (352x256 JPEG; wide = 800x600 eye center-cropped to 800x582 then scaled, center = 1:1 352x256 crop) -> observation_bridge `fit_image` (center-crop to 352:256 aspect + resize; no-op for live frames) -> RGB -> JPEG Q95 encode -> ZMQ -> server JPEG decode -> BGR->RGB -> GR00T N1.7 processor: shortest edge 256, center crop fraction 0.95, then Qwen3-VL `smart_resize` at native aspect (no 224 square crop as in N1.6). 352x256 is already short-edge 256 and a multiple of 32, so Isaac's `--images-pretransformed` path can skip the resize.

Image size is defined once in `alfie_gr00t/core/image_utils.py` (IMAGE_WIDTH/IMAGE_HEIGHT); `scripts/rosbag_to_groot.py` keeps a standalone copy of `fit_image` that must stay in sync.

Training images go through an additional H.264 yuv420p encode/decode in rosbag_to_groot.py (libx264, CRF=23) before being stored as MP4. The observation_bridge has an optional `h264_conditioning` flag to replicate this on live images.

# Troubleshooting & Benchmarking ---

## CSV Diagnostic Files
The client logs three files when `csv_log_path` is set (default: `/tmp/groot_client_debug.csv`; `csv_log_path:=...` on the launch line gives each run its own set):
 - `/tmp/groot_client_debug.csv` — 100Hz full-fidelity log. Columns: `timestamp, step, chunk_id, action_idx, action_<joint>, smoothed_<joint>, state_<joint>` for all 22 DOF. Use this for hold-and-wait analysis, trajectory shape, tracking error, and smoothing behavior.
 - `/tmp/groot_client_debug_actions.csv` — One row per action step (15 FPS). Columns: `timestamp, chunk_id, action_idx, effective_skip, action_<joint>, state_<joint>`; `action_*` is the RAW chunk action, before EMA/rate limiting (in the 100Hz file `action_*` is already rate-limited). Use this for chunk timing, effective_skip validation, and action-level trajectory analysis.
 - `/tmp/groot_client_debug_chunks.jsonl` — One line per chunk from the server: raw actions, obs/arrival time, `rtt_ms`, whether an RTC prefix was sent.

## Key Metrics to Check After a Run
1. **effective_skip** (actions CSV): Should be constant 2 (= latency_skip). If inflated (5+), the overshoot bug has returned.
2. **Hold-and-wait %**: Count rows with action_idx==15 (last action of the 16-step chunk; 31 for a 32-step checkpoint) in the debug CSV. Should be 0% (overflow [10:16] = 0.4 s covers the RTT). Hold only starts when RTT exceeds the overflow; an RTT just over `latency_skip` steps shows up as late chunks (`late%` in `rtc_ab_analysis.py`) and bigger seam jumps instead.
3. **Base velocity (lx, az)**: Forward velocity should reach 0.02-0.06 m/s during approach. If <0.005, robot isn't moving.
4. **action_idx distribution**: Should be roughly uniform across the execution window (2-9), with a short tail into 10-11 while waiting for inference. If the tail reaches 15, inference is far slower than expected.
5. **Head yaw consistency** (Test 7): Spread <0.05 rad is a healthy backend. Spread >0.1 rad means a noisy engine — rebuild or fall back to PyTorch.
6. **Chunk cycle time**: Time between consecutive chunk_id transitions. Should be ~0.64s (533ms exec + ~100ms inference, bridged by overflow actions).
9. **RTC seam**: with `rtc_enabled`, the first `latency_skip` actions of each new chunk must equal the old chunk's actions at the same frames (the server holds them). Client log `[rtc] prefix` and server log `RTC #n` confirm the prefix is flowing; if the server never logs it, the client is not sending `prev_actions`.
7. **Arm tracking error**: `|action_right_shoulder_pitch - state_right_shoulder_pitch|` should be <0.1 rad. Persistent >0.2 rad means servos can't track.
8. **Gripper**: Right gripper should show full range (0→close) during pick attempts. If stuck near 0, model isn't commanding a grasp.

## Quick Diagnostic Commands
```bash
# Consistency check (run BEFORE debugging live robot)
# hybrid_image_test.py runs all its tests (there is no --test flag); read Test 7:
# 5x identical inputs, HEAD_YAW SPREAD <0.05 = good (the INT8 full-pipeline TRT
# engines measured 0.018), >0.1 = noisy backend (the old N1.6 TRT bf16 DiT)
python3 scripts/hybrid_image_test.py --host 127.0.0.1 --dataset-path data/CleanTheRoom/lerobot

# Analyze a run CSV with python
python3 -c "
import pandas as pd
df = pd.read_csv('/tmp/groot_client_debug_actions.csv')
print(f'Chunks: {df.chunk_id.nunique()}, Steps: {len(df)}')
print(f'effective_skip: {df.effective_skip.value_counts().to_dict()}')
print(f'Duration: {df.timestamp.iloc[-1] - df.timestamp.iloc[0]:.1f}s')
print(f'Mean |lx|: {df.action_cmd_vel_lx.abs().mean():.4f}')
print(f'Mean |az|: {df.action_cmd_vel_az.abs().mean():.4f}')
"
```

## Debug Image Comparison
Set `debug_save_images: true` in `config/groot_client.yaml` to save observation images to `/tmp/groot_debug_images/`. Compare against training frames to check for visual domain gap.

## Key Diagnostic Tools
 - `scripts/hybrid_image_test.py` — Sends saved training images + GT state through ZMQ to isolate image vs state issues. Test 7 (consistency) is the fastest TRT-vs-PyTorch check.
 - `scripts/groot_open_loop_eval.py` — Replays training episodes through the server (uses torchcodec + raw RGB ZMQ path) to verify model accuracy independent of live robot.

## Current Execution Parameters (groot_client.yaml)
 - `training_fps=15` (66.7ms per action step; must match `rosbag_to_groot.py --fps`. The camera runs at 30fps; the converter and the live client both take the latest frame at or before each step)
 - Classic execution (`continuous_inference: false`). Continuous mode steps its EMA/Butterworth at 100 Hz instead of per action step; don't use it without fixing that.
 - `action_chunk_size=16` (checkpoint-20000), `n_action_steps=8, latency_skip=2, inference_trigger_step=8`
 - Execution window: actions[2:10] = 8 steps = 533ms per chunk; overflow actions[10:16] bridge 0.4s
 - Inference RTT on the robot (TRT INT8, full stack + LLM loaded): ~100ms at 4 denoising steps, ~155ms at 8 steps through the per-step engines (then launch with `latency_skip:=3`)
 - RTC: `rtc_enabled=true, rtc_overlap_steps=6`; the server runs with `--rtc`. Frozen steps = latency_skip, i.e. exactly the steps the client skips, so RTC pins no executed step (only the ramp reaches them)
 - Motion matched to VR teleop (how the data was recorded): 4 rad/s software rate limit on arms/grippers/head, servo speed/accel 0 (unlimited), no deadband, back velocity 0.2 / accel 0.1; joint EMA 0.3
 - Start pose: `back_init_height=0.34`; `ab_trial.py` puts arms + head at the pick-task start pose before activating (the idle node steers the head between trials)
 - Base velocity limits: lx=0.15 m/s, ly=0.15 m/s, az=0.8 rad/s
 - Launch overrides: `n_action_steps`, `inference_trigger_step`, `latency_skip`, `rtc_enabled`, `rtc_overlap_steps`, `continuous_inference`, `joint_smoothing_alpha`, `csv_log_path`

## Robot findings, 2026-11-12 (checkpoint-20000, CleanTheRoom; logs in ~/gr00t_ab/2026-11-12, trials.csv)
 - 4 denoising steps: smooth, but the bimodal actions get averaged. Demonstrated base turns are 0 or ~0.46 rad/s; live az came out at 0.005-0.05 rad/s, no head scan, near misses.
 - 8 denoising steps: head scan and full-rate turns return, first completed pick (which ran on into the trash-can motion with the pick task text), but head-yaw and base commands flip direction step to step (jitter). Fused 6/8-step engines and an offline comparison are with the Isaac-GR00T session.
 - 4 steps per chunk made the arm command flip-flop between reach and hold on consecutive chunks (11 jumps >0.1 rad per run, RTC on or off); 8 per chunk fixed most of it.
 - Task text is a weak condition: holding the can with the back up looks like the start of a trash episode and the model continues into it.
 - A copycat (base state -> base action) test on training frames was negative: from zero base state the model starts turns at ~85% of the demonstrated rate.

## Known Pitfalls
 - **Never add inference RTT as extra latency skip.** `latency_skip=3` already compensates for observation-to-action delay. Adding overshoot causes a death spiral of shrinking execution windows (see 2026-02-17 incident).
 - **Never fire inference mid-chunk without RTC.** Without a committed prefix the model must see the RESULT of its full trajectory; mid-chunk triggers caused arm pogoing at chunk boundaries. RTC is the mechanism that makes overlapping chunks consistent.
 - **RTC step count.** The ">= 4 denoising steps" rule came from N1.6 (RTC_ANALYSIS.md); on N1.7's native inpainting 2-step RTC smooths seams as well as 4-step offline. The native N1.7 path lives in `groot_inference_server.py` (`_build_rtc`), not in Isaac-GR00T's `Gr00tPolicy`, which still drops `options`.
 - **Horizon changes mean new stats.** `action_chunk_size` in the client yaml must match the DEPLOYED checkpoint's horizon (16 for checkpoint-20000; `alfiebot_config.ACTION_HORIZON` is 32 for the next retrain) and `meta/relative_stats.json` must be regenerated after any `delta_indices` or representation change.
 - **Backend.** The old N1.6 single-file TRT bf16 DiT caused catastrophic noise in flow matching denoising and is no longer supported. The Isaac-GR00T full-pipeline INT8 engines (`--trt-engine-dir`) pass Test 7 and match PyTorch open-loop; PyTorch (~390 ms) is too slow for the current client timing. Run the Test 7 consistency check after any engine rebuild.
 - **Client runs on system python**, which needs `python3-zmq` and `python3-msgpack` (apt). `av` is only imported when `h264_conditioning` is on.
 - **Don't judge the policy from an off-distribution start.** Trials must start from the training start pose (back 0.34 m, arms tucked, head centred); 75 of 83 pick demos begin with a head scan and a large base turn, and only 8 have the can already in view.
 - **Config lives in the checkpoint**, not `alfiebot_config.py`. Always check `processor_config.json` in the checkpoint directory to know actual action representations and normalization stats.

# ROS2 ---
Coding standards, domain knowledge, and preferences that AI should follow.
 - when you make a new msg, add the entry to cmakeLists.txt
 - when you make a new srv, add the entry to cmakeLists.txt
 - when you make a new python node, add the entry to setup.py