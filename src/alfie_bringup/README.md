# Alfie Bringup Package

This package contains the main launch file and configuration for bringing up the Alfie robot system.

## Overview

The `alfie_bringup` package orchestrates all the necessary nodes and components to run the Alfie robot, including:

- Micro-ROS agents for firmware communication
- Master status and command nodes
- Text-to-Speech (TTS) system
- Audio capture and publishing
- Automatic Speech Recognition (ASR)
- Large Language Model (LLM) integration
- Camera and sensor drivers
- Foxglove bridge for visualization

## Manual Launch

To manually start all Alfie systems:

```bash
cd ~/alfiebot_ws
source install/setup.bash
ros2 launch alfie_bringup alfie_bringup.py
```

## Automatic Startup on Boot

For production use, you can configure Alfie to start automatically when the Jetson boots.

### Installation

Navigate to the systemd directory and run the installer:

```bash
cd ~/alfiebot_ws/src/alfie_bringup/systemd
./install.sh
```

The installer will:
- Add your user to required groups (video, dialout, i2c, gpio)
- Install the systemd service
- Configure it to start on boot with a 10-second delay
- Optionally start the service immediately

### Quick Control

Use the control script for easy service management:

```bash
cd ~/alfiebot_ws/src/alfie_bringup/systemd

# Show status
./alfie-control.sh status

# Start/stop/restart
./alfie-control.sh start
./alfie-control.sh stop
./alfie-control.sh restart

# View logs
./alfie-control.sh logs      # Live logs
./alfie-control.sh recent    # Recent logs

# Enable/disable autostart
./alfie-control.sh enable
./alfie-control.sh disable
```

### Service Management

Standard systemd commands:

```bash
# Check status
sudo systemctl status alfiebot.service

# Start/stop/restart
sudo systemctl start alfiebot.service
sudo systemctl stop alfiebot.service
sudo systemctl restart alfiebot.service

# Enable/disable autostart
sudo systemctl enable alfiebot.service
sudo systemctl disable alfiebot.service

# View logs
sudo journalctl -u alfiebot.service -f
```

### Uninstallation

To remove the autostart service:

```bash
cd ~/alfiebot_ws/src/alfie_bringup/systemd
./uninstall.sh
```

## Detailed Documentation

For complete documentation on the systemd service, including:
- Troubleshooting
- Configuration options
- Log management
- Security settings

See: [systemd/README.md](systemd/README.md)

## Package Contents

- `launch/alfie_bringup.py` - Main launch file
- `systemd/` - Autostart service files
  - `alfiebot.service` - Systemd service unit file
  - `install.sh` - Installation script
  - `uninstall.sh` - Uninstallation script
  - `alfie-control.sh` - Quick control script
  - `README.md` - Detailed systemd documentation

## Requirements

- ROS2 Humble
- Built workspace with all Alfie packages
- User account with access to required hardware groups
- Network connectivity (for remote features)

## Troubleshooting

### Service won't start

1. Check logs: `sudo journalctl -u alfiebot.service -n 200`
2. Test manually: `ros2 launch alfie_bringup alfie_bringup.py`
3. Verify build: `cd ~/alfiebot_ws && colcon build`

### Permission issues

Ensure your user is in required groups:
```bash
sudo usermod -a -G video,dialout,i2c,gpio $USER
# Log out and back in for changes to take effect
```

### Hardware not ready

If USB devices aren't ready at startup, increase the delay in the service file:
```bash
sudo nano /etc/systemd/system/alfiebot.service
# Change: ExecStartPre=/bin/sleep 10
# To:     ExecStartPre=/bin/sleep 20
sudo systemctl daemon-reload
sudo systemctl restart alfiebot.service
```

## Support

For issues or questions, check:
1. Service logs: `sudo journalctl -u alfiebot.service -f`
2. Individual node logs in the journal
3. Manual launch to identify specific problems
