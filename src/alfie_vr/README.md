# Alfie VR Teleoperation

This package provides VR-based teleoperation for the Alfie robot using a VR headset (Meta Quest browser, WebXR) and controllers.

## Running

```bash
# Started automatically by alfie_bringup. To run it standalone instead:
ros2 launch alfie_vr alfie_teleop_vr.py   # or: ros2 run alfie_vr alfie_teleop_vr
```

Then open `https://<robot-ip>:8443` in the headset browser. See [CERTIFICATES.md](CERTIFICATES.md) for the self-signed certificate setup.

Python dependencies: `websockets`, `numpy`, `scipy`, `pyyaml` (`pygame` only for the optional debug visualizer).

## How VR takes and releases control

The node publishes to the per-subsystem command_mux inputs `/alfie/cmd/<subsystem>/vr` (priority 100), but only for subsystems it is actually driving. Anything VR has not claimed falls through to the next mux source (GR00T, idle, joy, ...).

| Subsystem | Claimed when | On link loss |
|-----------|--------------|--------------|
| Head | headset is tracked | released immediately (idle takes over) |
| Base | a controller is tracked | zero velocity for `base_stop_sec`, then released |
| Left / right arm | first grip on that arm, or Y reset | held at last target (`hold_timeout`) |
| Back | first A/B press, or setting `back_height` | held at last target (`hold_timeout`) |

- A device counts as lost when no data has arrived for `link_timeout` (0.3 s). The browser also sends an explicit release when the XR session ends, the page is hidden, or a controller disconnects. Closing the websocket releases immediately.
- On claim, targets are seeded from the measured pose (`/alfie/robotlowstate`), so nothing moves until you move.
- Published arm and head joint targets are rate-limited to `max_joint_speed` (4 rad/s). Joint targets are clamped to the URDF limits; head targets to `head_*_limit`.
- **E-stop** (`/alfie/estop_state`): VR input is ignored and every claimed subsystem holds the measured pose, so after reset VR resumes from where the robot actually is.
- `hold_timeout < 0` (default) holds arms/back until a new VR session takes over. A positive value releases them after that many seconds without VR input. Note that released arms go limp (firmware watchdog) unless another source commands them.

## VR Controller Mapping

### Head Tracking
The headset orientation (relative to where you faced when the VR session started) drives the head, at 2x gain:
- **Yaw** (looking left/right) → Head yaw servo
- **Pitch** (looking up/down) → Head pitch servo
- **Roll** (tilting head) → Head roll servo

### Left Controller

| Input | Function |
|-------|----------|
| **Thumbstick Y** | Drive forward/backward |
| **Thumbstick X** | Strafe left/right |
| **Grip Button (hold)** | Enable left arm tracking |
| **X Button** | Toggle passthrough mode |
| **Y Button** | Move both arms to the zero position |

### Right Controller

| Input | Function |
|-------|----------|
| **Thumbstick X** | Rotate robot left/right |
| **Thumbstick press** | Toggle episode recording |
| **A Button** | Lower back height |
| **B Button** | Raise back height |
| **Grip Button (hold)** | Enable right arm tracking |

### Arm Control (while grip held, either arm)
- **Controller Z movement** (forward/back) → Arm reach (forward/back)
- **Controller Y movement** (up/down) → Arm height (up/down)
- **Controller X movement** (left/right) → Shoulder pan (rotate)
- **Controller pitch rotation** → Wrist pitch
- **Controller roll rotation** → Wrist roll
- **Trigger** → Gripper, proportional (released = open, fully pulled = closed)

Arm control uses **delta tracking**: the arm moves relative to controller movement while the grip button is held.

1. Press and hold the grip button to start tracking
2. Move the controller to move the arm
3. Release the grip button to stop tracking; the arm stays where it is
4. Press grip again to resume from the current position

This lets you reposition your hand in VR space without moving the robot arm.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `link_timeout` | 0.3 | Seconds without data before a VR device counts as lost |
| `hold_timeout` | -1.0 | Seconds to hold arms/back after link loss before releasing (< 0 = hold) |
| `base_stop_sec` | 0.3 | Seconds of explicit zero velocity after link loss |
| `max_joint_speed` | 4.0 | Rate limit on published arm/head targets (rad/s) |
| `head_yaw_limit` / `head_pitch_limit` / `head_roll_limit` | 3.142 / 1.047 / 0.35 | Head command limits (rad; yaw is the full ±180° the neck supports) |
| `head_*_multiplier` | 2.0 | Headset → head gain |
| `head_yaw_sign` / `head_pitch_sign` / `head_roll_sign` | -1.0 | Headset → servo direction (servo +yaw turns right, +pitch looks down) |
| `max_linear_vel` / `max_angular_vel` | 0.15 / 0.5 | Joystick velocity scaling (accel limits are in command_mux) |
| `back_height` | — | Set at runtime to command a back height (0.0–0.390 m) |

## Debug Visualizer

Set `enable_debug_viz = True` in `alfie_teleop_vr.py` for a pygame window showing arm kinematics:

| Key | Function |
|-----|----------|
| **L** | View left arm |
| **R** | View right arm |
| **Tab** | Toggle between arms |
| **Q / Esc** | Close visualizer |

## Topics

### Published (command_mux inputs, only while claimed)
- `/alfie/cmd/left_arm/vr`, `/alfie/cmd/right_arm/vr` (alfie_msgs/ArmCmd)
- `/alfie/cmd/head/vr` (alfie_msgs/HeadCmd)
- `/alfie/cmd/back/vr` (alfie_msgs/BackCmd)
- `/alfie/cmd/base/vr` (geometry_msgs/Twist)
- `/alfie/recording/start`, `/alfie/recording/stop` (std_msgs/Bool)

### Subscribed
- `/alfie/robotlowstate` (alfie_msgs/RobotLowState) - Robot state feedback
- `/alfie/estop_state` (std_msgs/Bool, latched) - command_mux e-stop latch

## Browser ↔ robot protocol

WebSocket `wss://<robot>:8442`, JSON:
- Pose frame: `{seq, timestamp, headset?, leftController?, rightController?}`. A device key is omitted while that device is not tracked.
- `{"type": "release_all", "reason": ...}` and `{"type": "release_hand", "hand": "left"|"right", "reason": ...}`.
- Server → client: `{"command": "passthrough_mode", "active": bool}`.
