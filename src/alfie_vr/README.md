# Alfie VR Teleoperation

This package provides VR-based teleoperation for the Alfie robot using a VR headset and controllers.

## Running

```bash
ros2 run alfie_vr alfie_teleop_vr
```

## VR Controller Mapping

### Head Tracking
The VR headset directly controls the robot's head orientation:
- **Yaw** (looking left/right) → Head yaw servo
- **Pitch** (looking up/down) → Head pitch servo
- **Roll** (tilting head) → Head roll servo

### Left Controller

| Input | Function |
|-------|----------|
| **Thumbstick Y** | Drive forward/backward |
| **Thumbstick X** | Strafe left/right |
| **Grip Button (hold)** | Enable left arm tracking |
| **Y Button** | Reset both arms to zero position |

#### Left Arm Control (while grip held)
- **Controller Z movement** (forward/back) → Arm reach (forward/back)
- **Controller Y movement** (up/down) → Arm height (up/down)
- **Controller X movement** (left/right) → Shoulder pan (rotate)
- **Controller pitch rotation** → Wrist pitch
- **Controller roll rotation** → Wrist roll
- **Trigger** → Gripper close (>50% = closed)

### Right Controller

| Input | Function |
|-------|----------|
| **Thumbstick X** | Rotate robot left/right |
| **A Button** | Lower back height |
| **B Button** | Raise back height |
| **Grip Button (hold)** | Enable right arm tracking |

#### Right Arm Control (while grip held)
- **Controller Z movement** (forward/back) → Arm reach (forward/back)
- **Controller Y movement** (up/down) → Arm height (up/down)
- **Controller X movement** (left/right) → Shoulder pan (rotate)
- **Controller pitch rotation** → Wrist pitch
- **Controller roll rotation** → Wrist roll
- **Trigger** → Gripper close (>50% = closed)

## Arm Control Details

Arm control uses **delta tracking** - the arm moves relative to controller movement while the grip button is held. This means:

1. Press and hold the grip button to start tracking
2. Move the controller to move the arm
3. Release the grip button to stop tracking
4. The arm stays in its current position
5. Press grip again to resume tracking from the current position

This allows you to:
- Reposition your hand in VR space without moving the robot arm
- Make precise adjustments by releasing and re-gripping

## Debug Visualizer

When running, a pygame window shows real-time arm kinematics visualization:

| Key | Function |
|-----|----------|
| **L** | View left arm |
| **R** | View right arm |
| **Tab** | Toggle between arms |
| **Q / Esc** | Close visualizer |

The visualizer shows:
- Current joint angles (radians and degrees)
- End effector position (IK input vs FK result)
- VR controller input data
- Arm configuration and workspace limits

## Topics

### Published
- `/alfie/robotlowcmd` (alfie_msgs/RobotLowCmd) - Robot commands at 100Hz

### Subscribed
- `/alfie/robotlowstate` (alfie_msgs/RobotLowState) - Robot state feedback
