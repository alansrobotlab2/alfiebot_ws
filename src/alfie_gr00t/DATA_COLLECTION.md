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

**Robot Commands:**
- `/alfie/robotlowcmd` - Low-level robot commands (arms, grippers, base)

**Metadata:**
- `/recording/metadata` - Annotations and labels

> **Note:** The GR00T policy will primarily use camera images + `/alfie/robotlowstate` as observations and output `/alfie/robotlowcmd` as actions. The additional topics are recorded for dataset analysis and debugging.

## Quick Start

### 1. Build the package

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_gr00t
source install/setup.bash
```

### 2. Start the data recorder

**Terminal 1: Launch data recorder**
```bash
ros2 launch alfie_gr00t data_collection.launch.py
```

Or use the convenience script:
```bash
./src/alfie_gr00t/scripts/record_demo.sh
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
ros2 topic echo /recording/status
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
./src/alfie_gr00t/scripts/annotate_demo.py
```

This interactive tool lets you:
- Mark demonstrations as successful/failed/partial
- Rate grasp quality (1-5)
- Note lighting conditions
- Add free-form notes

Annotations are saved to `manifest.json` in the data directory.

### View Statistics

```bash
./src/alfie_gr00t/scripts/annotate_demo.py --stats
```

Shows:
- Total demonstrations
- Success rate
- Average quality score
- Distribution of conditions

## Data Storage

**Default location:** `~/alfiebot_ws/data/demonstrations/`

Each recording creates a directory:
```
demonstrations/
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
ros2 bag play ~/alfiebot_ws/data/demonstrations/demo_20250129_143022
```

### Inspect bag info

```bash
ros2 bag info ~/alfiebot_ws/data/demonstrations/demo_20250129_143022
```

### Extract images for visualization

```bash
# Install tool if needed
sudo apt install ros-humble-rosbag2-tools

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

## Next Steps

After collecting 500+ demonstrations:

1. **Annotate all demos** - Use annotation tool to label quality
2. **Filter dataset** - Remove corrupted/failed recordings
3. **Convert to GR00T format** - Use conversion script (Phase 3)
4. **Train policy** - Fine-tune GR00T on RTX 5090 (Phase 3)

## Advanced Options

### Custom output directory

```bash
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
