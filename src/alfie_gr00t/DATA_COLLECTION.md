# Data Collection Guide

This guide covers Phase 2 of the GR00T integration: collecting demonstration data for training.

## Overview

We use ROS2 bag recording to capture all sensor data and robot actions during teleoperation. The recorded data will later be converted to GR00T training format.

## Recorded Topics

The data recorder captures all sensor observations and robot actions:

### Observation Space (Input to GR00T)

**Camera Feeds (4 stereo streams - compressed):**
- `/alfie/stereo_camera/left_wide/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/left_center/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/right_center/image_raw/compressed` + `camera_info`
- `/alfie/stereo_camera/right_wide/image_raw/compressed` + `camera_info`

**Robot Proprioception:**
- `/alfie/robotlowstate` - Low-level robot state (joints, grippers, base)

### Action Space (Output from GR00T)

There is no separate command topic. `/alfie/robotlowstate` carries both the
measured state and the last command for every DOF (servo `target_location`,
back `command_position`, base `command_cmd_vel`), and `rosbag_to_groot.py`
derives the 22D action vector from those command fields.

**Metadata:**
- `/alfie/recording/metadata` - Annotations and labels (dataset name, start/stop)

> **Note:** The GR00T policy uses camera images + `/alfie/robotlowstate` as observations. The additional topics (`camera_info`, `tf`, `robot_description`, `rosout`) are recorded for dataset analysis and debugging.

## Dataset Layout

Every dataset (one challenge, one task family, one experiment) gets its own
folder under `~/alfiebot_ws/data/`, selected by a short `dataset` name:

```
~/alfiebot_ws/data/
└── <dataset>/                  e.g. can_do_challenge
    ├── demonstrations/         raw MCAP bags from the recorder
    │   ├── demo_20260102_195919/
    │   └── manifest.json       annotations (annotate_demo.py)
    └── lerobot/                GR00T LeRobot v2 output of rosbag_to_groot.py
        ├── meta/               info.json, modality.json, tasks.jsonl, episodes.jsonl, stats.json
        ├── data/chunk-000/
        └── videos/chunk-000/
```

The recorder, converter, episode manager and annotation tool all take
`--dataset NAME` (or the `ALFIE_DATASET` environment variable) and derive the
paths from `alfie_gr00t/dataset_paths.py`. `--data-root` / `ALFIE_DATA_ROOT`
moves the whole tree (e.g. to an external drive). Pass `<dataset>/lerobot` as
`--dataset-path` to Isaac-GR00T.

## Quick Start

### 1. Build the package

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_gr00t
source install/setup.bash
```

### 2. Start the data recorder

The recorder is started by `alfie_bringup.py` (`launch_recorder:=true` is the
default, `dataset:=<name>` picks the dataset). To run it on its own:

**Terminal 1: Launch data recorder**
```bash
ros2 launch alfie_gr00t data_collection.launch.py dataset:=can_do_challenge
```

Or use the convenience script:
```bash
./src/alfie_gr00t/scripts/record_demo.sh --dataset can_do_challenge
```

Switch datasets between episodes without relaunching:
```bash
ros2 param set /alfie/data_recorder dataset soda_sort
```

### 3. Start/Stop recording

**Terminal 2: Start recording**
```bash
ros2 topic pub --once /alfie/recording/start std_msgs/msg/Bool '{data: true}'
```

Now teleoperate Alfie to perform the can pickup task.

**When done, stop recording:**
```bash
ros2 topic pub --once /alfie/recording/stop std_msgs/msg/Bool '{data: true}'
```

### 4. Check recording status

```bash
ros2 topic echo /alfie/recording/status      # RECORDING:<dataset>/demo_... or STOPPED
```

## Recording Workflow

### Full Demo Recording Session

1. **Setup**: Position a soda can on the floor in view of cameras
2. **Start recorder**: Launch data_collection node
3. **Start recording**: Publish to `/recording/start`
4. **Teleoperate**: Drive to can, align, reach, grasp, lift
5. **Stop recording**: Publish to `/recording/stop`
6. **Repeat**: Move can to new position and record again

### Best Practices

**Diversity is key!** Collect demonstrations with:
- ✅ Various can positions (left, right, center, far, close)
- ✅ Different can orientations (standing, lying down)
- ✅ Multiple lighting conditions (bright, normal, dim)
- ✅ Different starting robot poses
- ✅ Various approach angles
- ✅ Different can types/brands (Coke, Pepsi, etc.)

**Quality over quantity:**
- 500+ successful demonstrations minimum
- Include ~100 partial successes (dropped can, grasp slippage)
- Avoid recordings with:
  - Camera occlusions
  - Network lag/stuttering
  - E-stop triggers
  - Hardware failures

## Annotation

After recording demonstrations, annotate them for training dataset curation:

```bash
./src/alfie_gr00t/scripts/annotate_demo.py --dataset can_do_challenge
```

This interactive tool lets you:
- Mark demonstrations as successful/failed/partial
- Rate grasp quality (1-5)
- Note lighting conditions
- Add free-form notes

Annotations are saved to `manifest.json` in the dataset's `demonstrations/` directory.

### View Statistics

```bash
./src/alfie_gr00t/scripts/annotate_demo.py --dataset can_do_challenge --stats
```

Shows:
- Total demonstrations
- Success rate
- Average quality score
- Distribution of conditions

## Data Storage

**Default location:** `~/alfiebot_ws/data/<dataset>/demonstrations/`

Each recording creates a directory:
```
<dataset>/demonstrations/
├── demo_20250129_143022/
│   ├── metadata.yaml
│   └── demo_20250129_143022_0.mcap
├── demo_20250129_143156/
│   └── demo_20250129_143156_0.mcap
└── manifest.json
```

**Storage format:** MCAP with zstd compression
- Efficient for large camera streams
- ~500MB-1GB per 2-minute demonstration
- Compatible with ROS2 bag tools

## Playback and Verification

### Play back a recording

```bash
ros2 bag play ~/alfiebot_ws/data/<dataset>/demonstrations/demo_20250129_143022
```

### Inspect bag info

```bash
ros2 bag info ~/alfiebot_ws/data/<dataset>/demonstrations/demo_20250129_143022
```

### Extract images for visualization

```bash
# Install tool if needed (rosbag2 metapackage: ros2 bag CLI + storage plugins)
sudo apt install ros-jazzy-rosbag2

# Extract images from a specific topic
ros2 bag play demo_20250129_143022 --topics /oak/rgb/image_raw
```

## Troubleshooting

### Recording doesn't start

**Check topics are publishing:**
```bash
ros2 topic list
ros2 topic hz /alfie/stereo_camera/left_wide/image_raw/compressed
ros2 topic hz /alfie/joint_states
ros2 topic hz /alfie/robotlowstate
```

**Check recorder node:**
```bash
ros2 node info /alfie/data_recorder
```

### High disk usage

**Enable compression** (already default):
- MCAP format with zstd compression
- Reduces size by ~3-5x vs raw

**Reduce camera resolution** (if needed):
- Edit camera driver config to lower resolution
- 640x480 usually sufficient for training

**Shorter recordings:**
- Keep demos <3 minutes
- Break long tasks into subtasks

### Timestamp synchronization issues

ROS2 bag automatically timestamps messages. If you see drift:
- Check system time is synchronized (NTP)
- Verify camera drivers publish correct timestamps
- Use `message_filters` for explicit sync (advanced)

## Converting to GR00T Training Format

Once you have recorded demonstrations, convert them to the GR00T-flavoured
LeRobot v2 format (Isaac-GR00T `getting_started/data_preparation.md`) using
`rosbag_to_groot.py`. This produces parquet files (state/action data), MP4
videos (camera observations), and the meta files required for training. On a
brand-new dataset it also creates `meta/info.json`, `meta/modality.json` and
`meta/tasks.jsonl`.

### Check available data

Run with `--report` to see a summary of recorded demos and any existing converted output:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py --dataset can_do_challenge --report
```

### Convert all demos

The first time a task index is used, name it with `--task-name`; that string is
the language instruction the model is trained on:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py \
    --dataset can_do_challenge \
    --task-index 0 --task-name "find the can and pick it up" \
    --num-threads 8
```

The converter is an incremental sync: re-running it adds new `demo_*` folders,
rebuilds episodes whose output is missing, and removes episodes whose bag is gone.

### GPU-accelerated video encoding

On the Jetson, use `--use-gpu` to encode videos with NVENC instead of CPU libx264:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py \
    --dataset can_do_challenge --task-index 0 --num-threads 8 --use-gpu
```

### Generate normalization statistics

The trainer refuses to load a dataset without `meta/stats.json`, and
`relative_stats.json` must be regenerated whenever the action `delta_indices`
in `alfiebot_config.py` change. From the Isaac-GR00T checkout:

```bash
python gr00t/data/stats.py \
    --dataset-path ~/alfiebot_ws/data/can_do_challenge/lerobot \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/alfiebot_config.py
```

### Fine-tune (GR00T N1.7)

`alfiebot_config.py` trains a 32-step horizon (2.1 s at 15 FPS) with RELATIVE
actions for the back, arms and head and ABSOLUTE actions for the base velocity
and grippers; `launch_finetune.py` turns relative actions on itself. The image
settings below match the base model's processor (short edge 256, 0.95 crop) so
the 352x256 camera frames are used at native aspect:

```bash
cd ~/Isaac-GR00T && source .venv/bin/activate && source scripts/activate_orin.sh
python gr00t/experiment/launch_finetune.py \
    --base-model-path nvidia/GR00T-N1.7-3B \
    --dataset-path ~/alfiebot_ws/data/can_do_challenge/lerobot \
    --embodiment-tag NEW_EMBODIMENT \
    --modality-config-path ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/alfiebot_config.py \
    --shortest-image-edge 256 --crop-fraction 0.95 \
    --color-jitter-params brightness 0.3 contrast 0.4 saturation 0.5 hue 0.08 \
    --global-batch-size 32 --max-steps 10000 --save-steps 2000 \
    --output-dir ~/Isaac-GR00T/alfie-gr00t
```

Several datasets train together by joining their paths with `:` in
`--dataset-path` (`--ds-weights-alpha` sets the mixture weighting). After
training, `action_chunk_size` in `groot_client.yaml` must equal the horizon.

### All options

Run with no arguments for a full summary of available options and examples:

```bash
python3 src/alfie_gr00t/scripts/rosbag_to_groot.py
```

### Output structure

After conversion, `<dataset>/lerobot/` will contain:

```
lerobot/
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── ...
├── videos/
│   └── chunk-000/
│       ├── observation.images.left_wide/
│       │   ├── episode_000000.mp4
│       │   └── ...
│       ├── observation.images.right_wide/
│       ├── observation.images.left_center/
│       └── observation.images.right_center/
└── meta/
    ├── info.json
    ├── modality.json
    ├── stats.json            (gr00t/data/stats.py)
    ├── relative_stats.json   (gr00t/data/stats.py)
    ├── episodes.jsonl
    └── tasks.jsonl
```

## Next Steps

After collecting 500+ demonstrations:

1. **Annotate all demos** - Use annotation tool to label quality
2. **Filter dataset** - Remove corrupted/failed recordings
3. **Convert to GR00T format** - Use `rosbag_to_groot.py` as described above
4. **Train policy** - Fine-tune GR00T with `--dataset-path ~/alfiebot_ws/data/<dataset>/lerobot`

## Advanced Options

### Custom data root or bag directory

```bash
# Whole tree on another drive: <data_root>/<dataset>/demonstrations
ros2 launch alfie_gr00t data_collection.launch.py \
    data_root:=/mnt/ssd/alfie_data dataset:=can_do_challenge

# Explicit bag directory, ignores the dataset layout
ros2 launch alfie_gr00t data_collection.launch.py \
    output_dir:=/path/to/custom/dir
```

### Auto-start recording

```bash
ros2 launch alfie_gr00t data_collection.launch.py \
    auto_start:=true
```

### Longer max duration

```bash
ros2 launch alfie_gr00t data_collection.launch.py \
    max_duration:=600.0  # 10 minutes
```

### Record specific topics only

Edit [data_recorder.py](alfie_gr00t/nodes/data_recorder.py:38) and modify `self.topics_to_record`.

## Tips for Efficient Data Collection

**Session planning:**
- Record in 2-hour blocks
- Take breaks to avoid operator fatigue
- 20-30 demos per session is realistic

**Setup optimization:**
- Pre-position multiple cans
- Quick reset between demos
- Use consistent starting pose

**Team approach:**
- One person teleoperates
- Another manages recording start/stop
- Third person annotates previous recordings

**Target: 500 demos in ~2 weeks**
- 50 demos/day × 10 days
- ~2-3 hours of recording per day
- ~3-4 hours of annotation per day

---

**Questions?** See main [README.md](README.md) or open an issue.
