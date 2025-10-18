# ServoTool2 - Web-Based Servo Configuration Tool

A modern web interface for configuring and monitoring ST3215 servos on the Alfie robot.

## Features

- 🌐 **Web-based Interface** - Access from any device on the network, no X forwarding needed
- 📊 **Real-time Monitoring** - Live servo status updates (position, speed, temperature, current, voltage)
- ⚙️ **Complete Configuration** - Full access to all EPROM and SRAM parameters
- 🎯 **Interactive Control** - Slider-based position control with visual feedback
- 🔒 **Safety Features** - EPROM lock/unlock, torque enable/disable
- 📋 **Memory Map Viewer** - Raw access to complete servo memory
- 💾 **Batch Operations** - Write all settings at once or individual fields
- 🎨 **Modern UI** - Clean, organized interface with helpful tooltips

## Installation

### 1. Install Gradio

```bash
pip install gradio
```

Or install from requirements:

```bash
pip install -r src/alfie_tools/alfie_tools/servotool2/requirements.txt
```

### 2. Build the Package

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_tools
source install/setup.bash
```

## Usage

### Starting ServoTool2

```bash
ros2 run alfie_tools servotool2
```

Then open your browser to: **http://localhost:7860**

Or from another device on the network: **http://<robot-ip>:7860**

### Interface Overview

#### ⚙️ EPROM Settings Tab
Configure persistent parameters that survive power cycles:
- **Basic Limits**: Angle limits, temperature limits
- **Voltage & Current**: Protection thresholds, torque limits
- **PID Control**: Position loop tuning parameters
- **Advanced Settings**: Dead zones, resolution, operation modes

Use the **"Write All EPROM Settings"** button to save changes.

#### 🎮 SRAM Control Tab
Control volatile parameters and real-time operation:
- **Interactive Position Slider**: Move servo by dragging slider (requires torque enabled)
- **Torque Enable/Disable**: Control motor holding torque
- **Quick Actions**: Zero servo, set min/max angles from current position
- **Motion Parameters**: Acceleration, speed, running time

Changes are applied immediately when modified.

#### 📊 Live Status Tab
Real-time monitoring (updates every second):
- **Position & Motion**: Current location, speed, acceleration, movement status
- **Electrical**: Voltage, current draw, temperature
- **Servo Info**: ID, baudrate, firmware version, error status

#### 🗺️ Memory Map Tab
View complete raw memory contents for debugging and verification.

## Configuration

### Servo Selection
Use the dropdown at the top to select which servo to configure:
- Driver 0 (right side servos 1-10)
- Driver 1 (left side servos 1-7)

### Safety Features

**EPROM Lock**: When enabled, prevents accidental writes to persistent memory.

**Torque Control**: 
- OFF (🔓): Servo is free-running, can be moved manually
- ON (⚡): Servo holds position and responds to commands

### Network Access

To access from other devices on your network:

1. Find your robot's IP address:
   ```bash
   hostname -I
   ```

2. Open browser on any device and navigate to:
   ```
   http://<robot-ip>:7860
   ```

## Troubleshooting

### "Service not available" Error
The servo driver services are not running. Start them with:
```bash
ros2 launch alfie_bringup servos.launch.py
```

### Browser Can't Connect
- Check that servotool2 is running
- Verify firewall settings allow port 7860
- Ensure you're on the same network

### Real-time Updates Not Working
- Check that the GDB state topics are publishing:
  ```bash
  ros2 topic hz /alfie/gdb0state
  ros2 topic hz /alfie/gdb1state
  ```

## Comparison with Original ServoTool

| Feature | ServoTool (Qt) | ServoTool2 (Web) |
|---------|---------------|------------------|
| Interface | Desktop GUI | Web Browser |
| Access Method | SSH + X forwarding | Any browser on network |
| Mobile Access | ❌ No | ✅ Yes |
| Real-time Updates | ✅ Yes | ✅ Yes |
| All Parameters | ✅ Yes | ✅ Yes |
| Memory Map View | ❌ No | ✅ Yes |
| Setup Required | PyQt5, X server | Just Python + Gradio |

## Architecture

```
servotool2/
├── servotool2_node.py          # ROS2 entry point
├── app/
│   ├── servotool_app.py        # Main Gradio interface
│   └── field_config.py         # Parameter definitions
└── ros/
    ├── servo_client.py         # Service client for read/write
    └── state_monitor.py        # Real-time state subscriber
```

## Development

The tool uses:
- **Gradio**: Web UI framework
- **ROS2**: Communication with servo drivers
- **rclpy**: ROS2 Python client library

To modify the interface, edit `servotool_app.py` and rebuild.

## Credits

Based on the original ServoTool Qt application, reimagined as a modern web interface.
