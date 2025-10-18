# Joydrive - Tank Drive Control via USB Joystick

This module provides joystick-based tank drive control for the Alfie robot.

## Features

- Tank drive control using USB joystick analog sticks
- Left stick Y-axis controls left track
- Right stick Y-axis controls right track
- Configurable axis mapping and inversion
- Deadzone support to ignore small stick movements
- Direct PWM control (0-255 range)

## Hardware Requirements

- USB joystick/gamepad (e.g., Xbox controller, PlayStation controller, or generic USB gamepad)
- The joystick should appear as `/dev/input/js0` (or similar) on Linux

## Usage

### Quick Start

Launch both the joy node and joydrive node:

```bash
ros2 launch alfie_tools joydrive.launch.py
```

### Manual Launch

If you prefer to launch nodes separately:

```bash
# Terminal 1: Start the joy node
ros2 run joy joy_node

# Terminal 2: Start the joydrive node
ros2 run alfie_tools joydrive
```

## Configuration

### Parameters

The joydrive node accepts the following parameters:

- `left_axis` (int, default: 1): Joystick axis index for left track control
- `right_axis` (int, default: 3): Joystick axis index for right track control
- `max_pwm` (int, default: 255): Maximum PWM value
- `deadzone` (float, default: 0.1): Deadzone threshold (0.0-1.0)
- `invert_left` (bool, default: false): Invert left stick direction
- `invert_right` (bool, default: false): Invert right stick direction

### Axis Mapping

Common joystick axis mappings:
- **Xbox/PlayStation controllers:**
  - Left stick Y: axis 1
  - Right stick Y: axis 3 (or axis 4 on some controllers)
- **Generic USB gamepads:** May vary, use `ros2 topic echo /joy` to check

### Custom Parameters

Launch with custom parameters:

```bash
ros2 launch alfie_tools joydrive.launch.py left_axis:=1 right_axis:=4 deadzone:=0.15
```

Or run the node directly:

```bash
ros2 run alfie_tools joydrive --ros-args -p left_axis:=1 -p right_axis:=4 -p deadzone:=0.15
```

## PWM Mapping

The joystick axis values (-1.0 to +1.0) are mapped to PWM values (0 to 255):

- **+1.0** (stick forward) → **255** (full forward)
- **0.0** (stick neutral) → **128** (stop)
- **-1.0** (stick backward) → **0** (full backward)

## Topics

### Subscribed

- `/joy` (sensor_msgs/Joy): Joystick input from joy_node

### Published

- `/alfie/robotlowcmd` (alfie_msgs/RobotLowCmd): Low-level robot control commands

## Troubleshooting

### Joystick not detected

Check if the joystick is recognized:

```bash
ls -l /dev/input/js*
```

Test joystick input directly:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

### Wrong axis mapping

Find the correct axes by moving the sticks and observing output:

```bash
ros2 topic echo /joy
```

Update the launch file or parameters accordingly.

### Robot not moving

1. Check that `/alfie/robotlowcmd` messages are being published:
   ```bash
   ros2 topic echo /alfie/robotlowcmd
   ```

2. Verify the robot firmware is receiving commands

3. Check if deadzone is too large (stick movements too small)

### Inverted controls

If forward/backward is reversed, set the appropriate invert parameter:

```bash
ros2 run alfie_tools joydrive --ros-args -p invert_left:=true -p invert_right:=true
```

## Safety

- The node sends continuous commands based on joystick position
- Releasing the sticks to neutral position stops the robot (PWM 128, 128)
- Always test in a safe area with room to stop
- Keep emergency stop available

## Development

To enable debug logging:

```bash
ros2 run alfie_tools joydrive --ros-args --log-level debug
```

This will show detailed joystick values and PWM mappings.
