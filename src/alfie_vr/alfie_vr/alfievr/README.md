# VR Input Data Structures

This document describes the data structures used for VR controller and headset tracking in the AlfieVR teleoperation system.

## Overview

The VR system uses WebSocket communication between the VR headset (browser) and the robot control server. Data flows in the following path:

```
VR Headset (A-Frame/WebXR) → WebSocket → VRWebSocketServer → ControlGoal → Robot Control Loop
```

---

## WebSocket Message Format

The VR client sends JSON messages containing tracking data for both controllers and the headset.

### Complete Message Structure

```json
{
  "timestamp": 1701619200000,
  "leftController": { ... },
  "rightController": { ... },
  "headset": { ... }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `number` | Unix timestamp in milliseconds (`Date.now()`) |
| `leftController` | `object` | Left controller data (see Controller Object) |
| `rightController` | `object` | Right controller data (see Controller Object) |
| `headset` | `object` | Headset tracking data (see Headset Object) |

---

## Controller Object

Each controller (`leftController` / `rightController`) contains:

```json
{
  "hand": "left",
  "position": { "x": 0.0, "y": 1.5, "z": -0.3 },
  "rotation": { "x": -45.0, "y": 0.0, "z": 0.0 },
  "quaternion": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 },
  "gripActive": false,
  "trigger": 0.75,
  "thumbstick": { "x": 0.0, "y": 0.5 },
  "buttons": {
    "squeeze": false,
    "thumbstick": false,
    "a": false,
    "b": false,
    "x": false,
    "y": false,
    "menu": false
  }
}
```

### Controller Fields

| Field | Type | Description |
|-------|------|-------------|
| `hand` | `string` | `"left"` or `"right"` |
| `position` | `object` | 3D position in VR space (meters) |
| `rotation` | `object` | Euler angles in degrees (XYZ order) |
| `quaternion` | `object` | Orientation as quaternion (more stable than Euler) |
| `gripActive` | `boolean` | `true` when grip button is held down |
| `trigger` | `number` | Analog trigger value `0.0` to `1.0` |
| `thumbstick` | `object` | Thumbstick/joystick position |
| `buttons` | `object` | Button press states |

### Position Object

```json
{
  "x": 0.0,
  "y": 1.5,
  "z": -0.3
}
```

| Axis | Description | Typical Range |
|------|-------------|---------------|
| `x` | Left/Right (positive = right) | -1.0 to 1.0 meters |
| `y` | Up/Down (positive = up) | 0.0 to 2.5 meters |
| `z` | Forward/Back (positive = forward, towards user) | -1.0 to 1.0 meters |

### Rotation Object (Euler Angles)

```json
{
  "x": -45.0,
  "y": 0.0,
  "z": 0.0
}
```

| Axis | Description | Range |
|------|-------------|-------|
| `x` | Pitch (tilt up/down) | -180° to 180° |
| `y` | Yaw (rotate left/right) | -180° to 180° |
| `z` | Roll (twist clockwise/counter-clockwise) | -180° to 180° |

### Quaternion Object

```json
{
  "x": 0.0,
  "y": 0.0,
  "z": 0.0,
  "w": 1.0
}
```

Quaternions provide more stable rotation representation without gimbal lock issues. Used for calculating relative wrist rotations.

| Component | Description |
|-----------|-------------|
| `x` | X component of rotation axis × sin(angle/2) |
| `y` | Y component of rotation axis × sin(angle/2) |
| `z` | Z component of rotation axis × sin(angle/2) |
| `w` | cos(angle/2) |

### Thumbstick Object

```json
{
  "x": 0.0,
  "y": 0.5
}
```

| Axis | Description | Range |
|------|-------------|-------|
| `x` | Left/Right | -1.0 to 1.0 |
| `y` | Up/Down | -1.0 to 1.0 |

### Buttons Object

Button mapping differs between left and right controllers:

**Left Controller:**
```json
{
  "squeeze": false,
  "thumbstick": false,
  "x": false,
  "y": false,
  "menu": false
}
```

**Right Controller:**
```json
{
  "squeeze": false,
  "thumbstick": false,
  "a": false,
  "b": false,
  "menu": false
}
```

| Button | Description | Gamepad Index |
|--------|-------------|---------------|
| `squeeze` | Side grip button | 1 |
| `thumbstick` | Thumbstick click | 3 |
| `x` / `a` | Lower face button | 4 |
| `y` / `b` | Upper face button | 5 |
| `menu` | Menu button | 6 |

---

## Headset Object

```json
{
  "position": { "x": 0.0, "y": 1.6, "z": 0.0 },
  "rotation": { "x": -10.0, "y": 45.0, "z": 0.0 },
  "quaternion": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `position` | `object` | 3D head position in VR space (meters) |
| `rotation` | `object` | Euler angles in degrees (where user is looking) |
| `quaternion` | `object` | Head orientation as quaternion |

---

## ControlGoal (Internal)

After processing, VR data is converted to `ControlGoal` objects for the robot control loop:

```python
@dataclass
class ControlGoal:
    arm: Literal["left", "right", "headset"]  # Target arm or headset
    mode: Optional[ControlMode] = None         # POSITION_CONTROL or IDLE
    target_position: Optional[np.ndarray] = None  # [x, y, z] in robot coords
    wrist_roll_deg: Optional[float] = None     # Wrist roll angle (degrees)
    wrist_flex_deg: Optional[float] = None     # Wrist pitch angle (degrees)
    gripper_closed: Optional[bool] = None      # Gripper state
    metadata: Optional[Dict[str, Any]] = None  # Additional data
```

### ControlGoal Metadata

The `metadata` field contains additional context:

```python
metadata = {
    "source": "vr_absolute_position",   # Origin of the command
    "relative_position": False,          # True if target is a delta
    "vr_position": [0.1, 0.2, 0.3],     # Original VR position
    "scaled_position": [0.05, 0.1, 0.15], # After vr_to_robot_scale applied
    "trigger": 0.75,                      # Raw trigger value
    "trigger_active": True,               # Trigger above threshold
    "thumbstick": {"x": 0.0, "y": 0.5},  # Thumbstick state
    "grip_active": True,                  # Grip button state
    "buttons": {...}                      # Button states
}
```

---

## Control Behavior

### Gripper Control

The trigger controls the gripper with **inverted logic**:
- **Trigger released** → Gripper **closed**
- **Trigger pressed** → Gripper **open**

Threshold: `trigger > 0.5` is considered active.

### Position Control

Position tracking is automatic (no grip required):
1. On first position data, the current position becomes the origin
2. Controller position is scaled by `vr_to_robot_scale` config value
3. Absolute position is sent to the robot arm

### Wrist Rotation

Wrist rotation is calculated from quaternions:
- **Roll** (Z-axis): Twisting the controller clockwise/counter-clockwise → `wrist_roll_deg`
- **Pitch** (X-axis): Tilting the controller up/down → `wrist_flex_deg`

Rotations are relative to the origin position set when tracking began.

---

## Example Usage

### Reading Controller Data (Python)

```python
async def process_controller_data(self, data: Dict):
    # Access left controller
    left = data.get('leftController', {})
    
    # Get position
    pos = left.get('position', {})
    x, y, z = pos.get('x', 0), pos.get('y', 0), pos.get('z', 0)
    
    # Check trigger
    trigger = left.get('trigger', 0)
    if trigger > 0.5:
        print("Trigger pressed!")
    
    # Check buttons
    buttons = left.get('buttons', {})
    if buttons.get('x'):
        print("X button pressed!")
    
    # Get thumbstick
    thumbstick = left.get('thumbstick', {})
    if abs(thumbstick.get('y', 0)) > 0.1:
        print(f"Thumbstick Y: {thumbstick['y']}")
```

### Reading Headset Data (Python)

```python
# Access headset
headset = data.get('headset', {})
if headset and headset.get('position'):
    pos = headset['position']
    rot = headset.get('rotation', {})
    
    print(f"Head position: {pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f}")
    print(f"Looking direction: yaw={rot.get('y', 0):.1f}°")
```

---

## Coordinate Systems

### VR Space (WebXR)
- Right-handed coordinate system
- Y-axis is up
- Negative Z is forward (away from user at session start)
- Origin is at floor level where VR session started

### Robot Space
- VR coordinates are scaled by `vr_to_robot_scale`
- Mapping depends on robot arm configuration
- See `AlfieVRConfig` for scaling and offset parameters

---

## Meta Quest 3 Button Indices

For reference, the raw gamepad button indices on Meta Quest 3:

| Index | Button |
|-------|--------|
| 0 | Trigger (analog) |
| 1 | Grip/Squeeze |
| 3 | Thumbstick press |
| 4 | A (right) / X (left) |
| 5 | B (right) / Y (left) |
| 6 | Menu |

Axes:
| Index | Axis |
|-------|------|
| 2 | Thumbstick X |
| 3 | Thumbstick Y |
