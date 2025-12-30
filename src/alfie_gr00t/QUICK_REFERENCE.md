# GR00T Data Collection - Quick Reference

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

## Storage Location

Default: `~/alfiebot_ws/data/demonstrations/`

## Target

- **500+ successful demonstrations**
- Diverse positions, lighting, can types
- 2-3 hours recording/day for 10 days
