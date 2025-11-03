# Right Arm Echo Node

## Overview

The `rightarmecho` node reads the current positions of the right arm servos from the `robotlowstate` topic and continuously writes them back as target positions to the `robotlowcmd` topic. This effectively makes the right arm "echo" or hold its current positions.

## Purpose

This node is useful for:
- Testing right arm servo control without external input
- Maintaining the right arm in its current position
- Debugging right arm servo feedback and control loops
- Creating a "freeze" or "hold position" behavior for the right arm

## Topics

### Subscribed Topics

- `/robotlowstate` (`alfie_msgs/RobotLowState`) - Best effort QoS
  - Reads current servo states, focusing on right arm servos (indices 6-11)

### Published Topics

- `/robotlowcmd` (`alfie_msgs/RobotLowCmd`) - Reliable QoS
  - Publishes commands to control servos, with right arm servos set to their current positions

## Servo Mapping

The node specifically handles the right arm servos:

- **Servo 6**: Right shoulder roll
- **Servo 7**: Right shoulder pitch  
- **Servo 8**: Right shoulder yaw
- **Servo 9**: Right elbow pitch
- **Servo 10**: Right wrist pitch
- **Servo 11**: Right wrist roll

All other servos in the command message are disabled (enabled=false).

## Running the Node

### Standard Run

```bash
ros2 run alfie_tools rightarmecho
```

### With Source

```bash
source /home/alfie/alfiebot_ws/install/setup.bash
ros2 run alfie_tools rightarmecho
```

## Parameters

This node currently has no configurable parameters. It runs at a fixed 100 Hz publishing rate.

## Safety Features

- All non-right-arm servos are explicitly disabled in the command output
- cmd_vel (base velocity) is set to zero
- eye_pwm is set to [0, 0]
- shoulder_height is set to 0.0
- Warnings are logged if no robot state has been received yet (throttled to once per second)

## Technical Details

- **Publishing Rate**: 100 Hz
- **QoS for Subscription**: Best Effort (matches typical state publishers)
- **QoS for Publishing**: Reliable (matches command consumers)

## Monitoring

To verify the node is working:

```bash
# Check if the node is running
ros2 node list | grep right_arm_echo

# Monitor the command output
ros2 topic echo /robotlowcmd

# Monitor the state input
ros2 topic echo /robotlowstate
```

## Troubleshooting

If the node publishes warnings about "No robot state received yet":
1. Verify that `robotlowstate` is being published: `ros2 topic hz /robotlowstate`
2. Check that the QoS policies match between publisher and subscriber
3. Ensure the master_status node or equivalent state publisher is running

## Integration with Other Nodes

This node can be run alongside:
- `master_status` - Provides the robotlowstate data
- `master_cmd` - Processes the robotlowcmd messages
- Other control nodes that may publish competing commands (note: last writer wins)

⚠️ **Warning**: Running this node alongside other nodes that publish to `robotlowcmd` may cause conflicts. Ensure only one command source is active at a time.
