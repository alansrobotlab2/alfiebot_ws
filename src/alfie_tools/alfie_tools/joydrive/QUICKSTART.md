# Joydrive Quick Reference

## Quick Start

```bash
# Launch everything (joy_node + joydrive)
ros2 launch alfie_tools joydrive.launch.py

# Or manually:
ros2 run joy joy_node
ros2 run alfie_tools joydrive
```

## Testing Joystick

```bash
# Check joystick device
ls -l /dev/input/js*

# Monitor joystick input
ros2 topic echo /joy

# Monitor robot commands
ros2 topic echo /alfie/robotlowcmd
```

## Common Axis Configurations

**Xbox/PlayStation Controllers:**
```bash
ros2 launch alfie_tools joydrive.launch.py left_axis:=1 right_axis:=3
```

**Alternative mapping (some controllers):**
```bash
ros2 launch alfie_tools joydrive.launch.py left_axis:=1 right_axis:=4
```

## Invert Controls

If forward/backward is reversed:
```bash
ros2 run alfie_tools joydrive --ros-args -p invert_left:=true -p invert_right:=true
```

## PWM Values

- Forward (stick up): PWM = 255
- Neutral (stick center): PWM = 128
- Backward (stick down): PWM = 0
- Full forward both tracks: [255, 255]

## Debug Mode

```bash
ros2 run alfie_tools joydrive --ros-args --log-level debug
```
