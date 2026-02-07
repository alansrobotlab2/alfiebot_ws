# GR00T Data Collection - Quick Reference

## Requirements
```bash
pip install zmq fastparquet msgpack opencv-python-headless pyarrow
```

## Pre-flight Check

```bash
./src/alfie_gr00t/scripts/check_topics.sh
```
Verify all camera, state, and command topics are available.

## Recording Commands

### Start Data Recorder
```bash
ros2 launch alfie_gr00t data_collection.launch.py
```

### Start Recording
```bash
ros2 topic pub --once /alfie/recording/start std_msgs/msg/Bool '{data: true}'
```

### Stop Recording
```bash
ros2 topic pub --once /alfie/recording/stop std_msgs/msg/Bool '{data: true}'
```

### Check Status
```bash
ros2 topic echo /alfie/recording/status
```

### Check Topics Publishing
```bash
ros2 topic hz /alfie/stereo_camera/left_wide/image_raw/compressed
ros2 topic hz /alfie/joint_states
ros2 topic hz /alfie/robotlowstate
```

## Workflow

1. **Launch recorder** → `ros2 launch alfie_gr00t data_collection.launch.py`
2. **Position can** → Place on floor in camera view
3. **Start recording** → Publish to `/alfie/recording/start`
4. **Teleoperate** → Drive → Align → Reach → Grasp → Lift
5. **Stop recording** → Publish to `/alfie/recording/stop`
6. **Repeat** → New position, record again

## Annotation

```bash
./src/alfie_gr00t/scripts/annotate_demo.py
```

## Playback

```bash
ros2 bag play ~/alfiebot_ws/data/demonstrations/demo_YYYYMMDD_HHMMSS
```

## Inspection

```bash
ros2 bag info ~/alfiebot_ws/data/demonstrations/demo_YYYYMMDD_HHMMSS
```

## Open-Loop Evaluation

Replays a recorded episode through the ZMQ client→server inference pipeline and
compares predicted actions against ground truth. Produces trajectory plots and
communication diagnostics. No ROS2 required.

```bash
python ./alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_open_loop_eval.py \
    --dataset-path ./alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-index 0 \
    --host 192.168.50.201 \
    --port 5555 \
    --closed-loop
```

### Options

| Option | Default | Description |
|--------|---------|-------------|
| `--dataset-path` | *(required)* | Path to LeRobot-format dataset |
| `--episode-index` | `0` | Episode index to evaluate |
| `--host` | `192.168.50.108` | Inference server host |
| `--port` | `5555` | Inference server port |
| `--transport` | `tcp` | ZMQ transport (`tcp` or `ipc`) |
| `--timeout-ms` | `5000` | Inference timeout (ms) |
| `--action-horizon` | `16` | Steps per inference chunk |
| `--task` | `"find the can and pick it up"` | Language instruction sent to model |
| `--stats-path` | `{dataset}/meta/stats.json` | Normalization statistics file |
| `--save-plot` | `/tmp/groot_eval/episode_N.png` | Trajectory plot output path |
| `--verbose` | off | Enable debug logging |

### Output

- **Trajectory plot** — per-joint comparison of state, ground truth action, and predicted action with inference points marked
- **Comms plot** — round-trip latency, message sizes, and effective FPS over time (`_comms.png` suffix)
- **Console** — MSE/MAE per joint, bandwidth summary with latency percentiles (mean/p50/p95/p99), projected 15 FPS bandwidth

## Inference Server (standalone)

```bash
# Real model inference
python src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
    --checkpoint /home/alfie/cando --transport tcp --port 5555

# Replay mode (no GPU needed)
python src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
    --dataset-path ~/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-index 0 --transport tcp --port 5555

# Mock mode (testing)
python src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
    --mock --transport tcp --port 5555
```

## Storage Location

Default: `~/alfiebot_ws/data/demonstrations/`

## Target

- **500+ successful demonstrations**
- Diverse positions, lighting, can types
- 2-3 hours recording/day for 10 days
