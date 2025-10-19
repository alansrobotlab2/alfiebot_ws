# Alfie Robot Systemd Service

This directory contains the systemd service configuration for automatically starting the Alfie robot on boot.

## Overview

The `alfiebot.service` systemd unit will:
- Wait for network connectivity to be established
- Wait an additional 10 seconds after boot for system stabilization
- Source the ROS2 Humble environment
- Source the Alfie workspace
- Launch the `alfie_bringup.py` launch file
- Automatically restart the service if it crashes
- Log all output to the system journal

## Installation

### Prerequisites

1. **Build the workspace** (if not already done):
   ```bash
   cd ~/alfiebot_ws
   colcon build
   source install/setup.bash
   ```

2. **Test the launch file manually** to ensure it works:
   ```bash
   ros2 launch alfie_bringup alfie_bringup.py
   ```
   Press `Ctrl+C` to stop once verified.

### Install the Service

Run the installation script from this directory:

```bash
cd ~/alfiebot_ws/systemd
chmod +x install.sh
./install.sh
```

The script will:
- Check and add your user to required groups (video, dialout, i2c, gpio)
- Update the service file with your username and home directory
- Copy the service to `/etc/systemd/system/`
- Enable the service to start on boot
- Optionally start the service immediately

## Usage

### Service Management Commands

**Start the service:**
```bash
sudo systemctl start alfiebot.service
```

**Stop the service:**
```bash
sudo systemctl stop alfiebot.service
```

**Restart the service:**
```bash
sudo systemctl restart alfiebot.service
```

**Check service status:**
```bash
sudo systemctl status alfiebot.service
```

**Enable autostart on boot:**
```bash
sudo systemctl enable alfiebot.service
```

**Disable autostart on boot:**
```bash
sudo systemctl disable alfiebot.service
```

### Viewing Logs

**View live logs:**
```bash
sudo journalctl -u alfiebot.service -f
```

**View recent logs:**
```bash
sudo journalctl -u alfiebot.service -n 100
```

**View logs since last boot:**
```bash
sudo journalctl -u alfiebot.service -b
```

**View logs for a specific time range:**
```bash
sudo journalctl -u alfiebot.service --since "2025-01-01 00:00:00" --until "2025-01-01 23:59:59"
```

## Uninstallation

To remove the service and disable autostart:

```bash
cd ~/alfiebot_ws/systemd
chmod +x uninstall.sh
./uninstall.sh
```

## Troubleshooting

### Service fails to start

1. **Check the status:**
   ```bash
   sudo systemctl status alfiebot.service
   ```

2. **View detailed logs:**
   ```bash
   sudo journalctl -u alfiebot.service -n 200
   ```

3. **Common issues:**
   - **Workspace not built:** Make sure you've run `colcon build` successfully
   - **Permissions:** Ensure your user is in the required groups (video, dialout, i2c, gpio)
   - **USB devices not ready:** The 10-second delay should handle this, but you may need to increase it
   - **Network not ready:** Check that network-online.target is working

### Service starts but nodes crash

- Check individual node logs in the journal
- Verify all dependencies are installed
- Test the launch file manually to identify the issue

### Increase startup delay

If devices aren't ready after 10 seconds, edit the service file:

```bash
sudo nano /etc/systemd/system/alfiebot.service
```

Change the `ExecStartPre` line:
```ini
ExecStartPre=/bin/sleep 20  # Increase to 20 seconds
```

Then reload and restart:
```bash
sudo systemctl daemon-reload
sudo systemctl restart alfiebot.service
```

### Disable autostart temporarily

If you need to prevent autostart without uninstalling:

```bash
sudo systemctl disable alfiebot.service
```

Re-enable later with:
```bash
sudo systemctl enable alfiebot.service
```

## Service Configuration

The service file includes:

- **User/Group:** Runs as your user account (not root)
- **Working Directory:** Your workspace directory
- **Restart Policy:** Automatically restarts on failure with 10-second delay
- **Resource Limits:** Increased file descriptor limit for ROS2
- **Priority:** Nice value of -10 for higher priority
- **Groups:** Access to video, dialout, i2c, and gpio devices
- **Logging:** All output goes to systemd journal

## Manual Service File Editing

If you need to customize the service:

1. Edit the service file:
   ```bash
   sudo nano /etc/systemd/system/alfiebot.service
   ```

2. Reload systemd:
   ```bash
   sudo systemctl daemon-reload
   ```

3. Restart the service:
   ```bash
   sudo systemctl restart alfiebot.service
   ```

## Environment Variables

You can add additional environment variables in the service file under the `[Service]` section:

```ini
Environment="ROS_DOMAIN_ID=0"
Environment="CUSTOM_VAR=value"
```

## Security Notes

- The service runs as your user account, not root
- `NoNewPrivileges=true` prevents privilege escalation
- Access to hardware is granted through group membership
- Logs are accessible via journalctl

## Support

If you encounter issues:

1. Check the logs: `sudo journalctl -u alfiebot.service -n 200`
2. Test manually: `ros2 launch alfie_bringup alfie_bringup.py`
3. Verify permissions: `groups $USER`
4. Check systemd status: `systemctl status alfiebot.service`

For additional help, refer to the main project documentation.
