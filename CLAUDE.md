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

# ROS2 ---
Coding standards, domain knowledge, and preferences that AI should follow.
 - when you make a new msg, add the entry to cmakeLists.txt
 - when you make a new srv, add the entry to cmakeLists.txt
 - when you make a new python node, add the entry to setup.py