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
 - Jetson Orin NX 16gb developer kit
 - Ubuntu 22.04
 - Jetpack 6.2.2


We are actively working on an nvidia gr00t n1.6 solution to run on alfie to complete a challenge to identify, localize, pick up, then set down a soda can
https://github.com/NVIDIA/Isaac-GR00T

We currently have 200 episodes with 2 tasks, about 100 episodes per task.

The episodes were recorded in varying environments and lighting conditions, with the soda can placed anywhere within 1 to 3 feed anywhere around the robot.

As of right now groot server runs on an rtx 3090 PC workstation and groot client runs on the orin nx on the robot over tcp and wifi.


# GR00T N1.6 Architecture ---

## Client-Server Split
The GR00T inference system is split across two machines:
 - **Server** (GPU workstation, e.g. 192.168.50.108): Runs the GR00T N1.6 model. Receives observations over ZMQ, returns 16-step action horizons.
 - **Client** (Jetson Orin): ROS2 node that collects camera images + robot state, sends them to the server via ZMQ, and publishes actions to the robot.

The server can run as either:
 - A standalone Python script: `scripts/groot_inference_server.py` (typical for remote GPU)
 - A ROS2 node via the launch file (for on-device inference)

## Key Components
 - `nodes/groot_client.py` — ROS2 client node. Collects observations, sends to server, publishes actions at 100Hz.
 - `core/observation_bridge.py` — Subscribes to 4 camera topics + robot state, synchronizes them, resizes images to 320x240, packages as JPEG bytes for ZMQ.
 - `core/action_publisher.py` — Receives 16-step action chunks from the server, steps through them at the training rate (15 FPS), applies smoothing and safety limits, publishes servo commands at 100Hz.
 - `core/zmq_client.py` — ZMQ REQ/REP client for communicating with the server.
 - `scripts/groot_inference_server.py` — Standalone inference server. Loads the GR00T checkpoint, receives JPEG images + state via ZMQ msgpack, runs inference, returns raw unnormalized actions.
 - `scripts/hybrid_image_test.py` — Diagnostic tool that sends saved images through ZMQ to isolate visual domain gap issues.

## Communication Protocol (ZMQ msgpack)
Client sends: `{ 'left_wide': <jpeg_bytes>, 'right_wide': <jpeg_bytes>, ..., 'state': <22D float list>, 'language': <task string> }`
Server returns: `{ 'actions': <16x22 float array> }`
Ping: Client sends `{'ping': True}`, server returns `{'pong': True}`.

## Config Files
 - `config/groot_client.yaml` — Client ROS2 parameters: transport, server address, inference FPS, action chunking, smoothing, safety limits, task description, debug settings.
 - `config/groot_server.yaml` — Server ROS2 parameters: transport, bind address, model checkpoint path, TensorRT toggle, mock mode, replay mode.
 - `launch/groot_inference.launch.py` — Launches both server + client with configurable launch args. Set `launch_server:=false` for remote GPU setups.

## Training Pipeline
 - Training config: `alfiebot_config.py` — Defines modality config (4 cameras, 22D state/action, all ABSOLUTE action representation).
 - Data conversion: `scripts/rosbag_to_groot.py` — Converts ROS2 rosbags to LeRobot format (MP4 videos + parquet state/action data).
 - Dataset: `data/alfiebot.CanDoChallenge/` — 200 episodes, 2 tasks, LeRobot format with `meta/modality.json`.
 - Fine-tuning: Runs on the GPU workstation using `Isaac-GR00T/gr00t/experiment/launch_finetune.py`.
 - Checkpoints saved to `Isaac-GR00T/alfie-gr00t/` with `processor_config.json` containing normalization stats and action config. The server loads config from the checkpoint, NOT from `alfiebot_config.py`.

## 22D State/Action Vector
`[0:6]` base velocity (linear xyz, angular xyz), `[6]` back height, `[7:12]` left arm (5 joints), `[12]` left gripper, `[13:18]` right arm (5 joints), `[18]` right gripper, `[19:22]` head (3 joints).

## Image Pipeline
Camera (640x480 JPEG) -> observation_bridge resizes to 320x240 RGB -> JPEG Q95 encode -> ZMQ -> server JPEG decode -> BGR->RGB -> GR00T processor applies eval transforms (SmallestMaxSize -> CenterCrop -> SmallestMaxSize to 224x224).

Training images go through an additional H.264 yuv420p encode/decode in rosbag_to_groot.py (libx264, CRF=23) before being stored as MP4. The observation_bridge has an optional `h264_conditioning` flag to replicate this on live images.

# Troubleshooting & Benchmarking ---

## CSV Diagnostic Files
The client logs two CSV files when `csv_log_path` is set (default: `/tmp/groot_client_debug.csv`):
 - `/tmp/groot_client_debug.csv` — 100Hz full-fidelity log. Columns: `timestamp, step, chunk_id, action_idx, action_<joint>, smoothed_<joint>, state_<joint>` for all 22 DOF. Use this for hold-and-wait analysis, trajectory shape, tracking error, and smoothing behavior.
 - `/tmp/groot_client_debug_actions.csv` — One row per action step (15 FPS). Columns: `timestamp, chunk_id, action_idx, effective_skip, action_<joint>, state_<joint>`. Use this for chunk timing, effective_skip validation, and action-level trajectory analysis.

## Key Metrics to Check After a Run
1. **effective_skip** (actions CSV): Should be constant 4. If inflated (7-9+), the overshoot bug has returned.
2. **Hold-and-wait %**: Count rows with action_idx==15 (last action) in the debug CSV. Should be <10%. If >50%, execution window is too narrow or inference is too slow.
3. **Base velocity (lx, az)**: Forward velocity should reach 0.02-0.06 m/s during approach. If <0.005, robot isn't moving.
4. **action_idx distribution**: Should be roughly uniform across 4-15. If >40% at idx 15, chunks are exhausting and stalling.
5. **Head yaw consistency**: Spread <0.05 rad confirms PyTorch backend. Spread >0.1 rad means TRT bf16 — switch to PyTorch immediately.
6. **Chunk cycle time**: Time between consecutive chunk_id transitions. Should be ~1.15s (804ms exec + 350ms inference).
7. **Arm tracking error**: `|action_right_shoulder_pitch - state_right_shoulder_pitch|` should be <0.1 rad. Persistent >0.2 rad means servos can't track.
8. **Gripper**: Right gripper should show full range (0→close) during pick attempts. If stuck near 0, model isn't commanding a grasp.

## Quick Diagnostic Commands
```bash
# Check TRT vs PyTorch backend (run BEFORE debugging live robot)
# Uses hybrid_image_test.py Test 7: 5x identical inputs, check HEAD_YAW SPREAD
# Spread <0.05 = PyTorch (good), >0.1 = TRT bf16 (bad)
python3 scripts/hybrid_image_test.py --test 7 --server-host 192.168.50.201

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
 - `action_chunk_size=16, n_action_steps=12, latency_skip=4, inference_trigger_step=12`
 - Execution window: actions[4:16] = 12 steps = 804ms per chunk
 - Overflow: actions[12:16] bridges 268ms of ~350ms inference RTT
 - Hold-and-wait gap: ~82ms (acceptable)
 - Re-planning rate: ~0.87 Hz
 - Rate-limited interpolator: enabled (G1-style velocity-capped joints)
 - Base velocity limits: lx=0.15 m/s, ly=0.15 m/s, az=0.8 rad/s

## Known Pitfalls
 - **Never add inference RTT as extra latency skip.** `latency_skip=4` already compensates for observation-to-action delay. Adding overshoot causes a death spiral of shrinking execution windows (see 2026-02-17 incident).
 - **Never fire inference mid-chunk.** The model must see the RESULT of its full trajectory. Mid-chunk triggers cause arm pogoing at chunk boundaries.
 - **Always use PyTorch backend** (not TRT bf16). TRT bf16 causes catastrophic noise in flow matching denoising. Run Test 7 consistency check before any live debugging session.
 - **Config lives in the checkpoint**, not `alfiebot_config.py`. Always check `processor_config.json` in the checkpoint directory to know actual action representations and normalization stats.

# ROS2 ---
Coding standards, domain knowledge, and preferences that AI should follow.
 - when you make a new msg, add the entry to cmakeLists.txt
 - when you make a new srv, add the entry to cmakeLists.txt
 - when you make a new python node, add the entry to setup.py