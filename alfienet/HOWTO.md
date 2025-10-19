# Alfienet - WiFi Hotspot Setup Guide

This guide explains how to set up and use the Alfienet WiFi hotspot script on your Jetson/Linux system. This script creates a wireless access point (WiFi hotspot) named "alfienet" that allows other devices to connect to your robot.

## 🎯 Typical Use Case

**This script is designed to allow for quickly enabling the wifi hotspot for remote connection, especially when you're away from your home wifi network.**

### Common Scenario:

1. **Connect to robot** via USB-C serial debug port (no WiFi configured yet)
2. **Wait for boot** and log in through serial console
3. **Run this script** to temporarily convert `wlan0` into a WiFi hotspot
4. **Disconnect from serial** - the hotspot session persists in the background
5. **Connect your laptop/phone** to the "alfienet" WiFi network
6. **Work wirelessly** - SSH, configure, test, etc.
7. **Reconnect to serial** when you're done
8. **Press Ctrl+C** to stop the script and restore normal WiFi settings
9. **Reboot the robot** - all original WiFi settings are automatically restored

### Why This is Useful:

- ✅ **No permanent WiFi changes** - Settings revert when script stops or system reboots
- ✅ **Serial port freedom** - Start the script then disconnect the USB cable
- ✅ **Quick wireless access** - Get WiFi connectivity without configuring network settings
- ✅ **Debugging tool** - Perfect for field work or initial robot setup
- ✅ **Reversible** - Stop the script or reboot to return to normal WiFi operation

## What This Script Does

The `alfienet.sh` script:
- Creates a WiFi hotspot (access point) on your wireless interface
- Network name (SSID): **alfienet**
- Password: **alfienet**
- Uses 5GHz band for better performance
- Channel 6
- Allows devices to connect and communicate with your robot

## Prerequisites

### 1. Hardware Requirements

- A Linux system with WiFi capability (Jetson Orin Nano, Xavier, etc.)
- WiFi adapter that supports AP (Access Point) mode
- At least one wireless interface (usually `wlan0`)

### 2. Software Dependencies

Install the required packages:

```bash
sudo apt update
sudo apt install -y \
    hostapd \
    dnsmasq \
    iptables \
    iproute2 \
    haveged \
    iw \
    procps \
    network-manager
```

### 3. Check WiFi Interface

Verify your wireless interface name:

```bash
ip link show
# or
ifconfig
```

Look for `wlan0` or similar wireless interface. If your interface has a different name (like `wlp2s0`), you'll need to modify the script.

### 4. Verify AP Mode Support

Check if your WiFi adapter supports AP mode:

```bash
iw list | grep -A 10 "Supported interface modes"
```

You should see `AP` in the list. If not, your WiFi adapter may not support hotspot functionality.

## Installation

### Step 1: Navigate to the Alfienet Directory

```bash
cd ~/alfiebot_ws/alfienet
```

### Step 2: Make Scripts Executable

```bash
chmod +x alfienet.sh lnxrouter
```

### Step 3: Verify lnxrouter

The `lnxrouter` script should already be in this directory. Verify it exists:

```bash
ls -l lnxrouter
```

## Usage

### Typical Workflow (Serial Debug Port)

This is the most common way to use this script:

**Step 1: Connect via Serial Debug Port**
```bash
# Connect USB-C cable to robot's debug port
# Use your serial terminal (screen, minicom, putty, etc.)
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0
```

**Step 2: Wait for Boot and Login**
- Robot boots up
- Login with your credentials

**Step 3: Start the Hotspot**
```bash
cd ~/alfiebot_ws/alfienet
sudo ./alfienet.sh
```
- Script starts and creates the hotspot
- You'll see status messages
- **Leave the terminal running**

**Step 4: Disconnect Serial Cable (Optional)**
- The hotspot continues running in the background
- You can physically disconnect the USB cable
- The script keeps the hotspot active

**Step 5: Connect Wirelessly**
- On your laptop/phone, connect to WiFi "alfienet"
- Password: "alfienet"
- You now have wireless access to the robot!

**Step 6: Do Your Work**
- SSH into the robot: `ssh alfie@192.168.12.1` (or check DHCP lease)
- Configure settings
- Run tests
- Whatever you need to do

**Step 7: Reconnect and Stop (When Done)**
- Reconnect USB-C serial cable (if disconnected)
- Return to the terminal where the script is running
- Press **Ctrl+C**
- WiFi settings automatically restored
- `wlan0` returns to normal operation

**Step 8: Reboot (Optional)**
- If you reboot the robot, all original WiFi settings return automatically
- No cleanup needed

### Starting the WiFi Hotspot (Alternative Methods)

**Option 1: Run directly**

```bash
cd ~/alfiebot_ws/alfienet
sudo ./alfienet.sh
```

**Option 2: Run from anywhere (after making executable)**

```bash
sudo ~/alfiebot_ws/alfienet/alfienet.sh
```

### What Happens When You Run It

1. NetworkManager releases control of `wlan0`
2. The hotspot is created with these settings:
   - **SSID:** alfienet
   - **Password:** alfienet
   - **Band:** 5GHz
   - **Channel:** 6
   - **Security:** WPA2
   - **Country:** US
3. DHCP server starts (assigns IP addresses to connected devices)
4. Internet sharing is enabled (if you have another network connection)

### Session Persistence

**Important characteristics of this script:**

✅ **Session persists when you disconnect serial**
- Once started, the hotspot runs in the background
- You can disconnect the USB-C cable
- The hotspot continues to work
- Other devices stay connected

✅ **Requires reconnecting to stop**
- To stop the script, reconnect to the serial port
- Return to the terminal session
- Press Ctrl+C to gracefully stop

✅ **Settings are temporary**
- All WiFi changes are temporary
- Stopping the script restores original WiFi configuration
- Rebooting the robot restores original WiFi configuration
- No permanent changes to network settings

✅ **Use with screen/tmux for better control** (see Advanced Usage section)

### Stopping the WiFi Hotspot

Press `Ctrl+C` in the terminal where the script is running.

This will:
- Stop the access point
- Restore NetworkManager control of `wlan0`
- Clean up iptables rules
- Return WiFi to its previous state

**If you lose the terminal session:**
- Reboot the robot to restore WiFi settings
- Or SSH in and kill the process: `sudo killall hostapd dnsmasq`

### Connecting Devices

On your phone, laptop, or other device:

1. Open WiFi settings
2. Look for network: **alfienet**
3. Enter password: **alfienet**
4. Connect!

Your device will typically get an IP address in the range `192.168.12.x`

## Configuration

### Changing WiFi Settings

Edit `alfienet.sh` to customize:

```bash
nano ~/alfiebot_ws/alfienet/alfienet.sh
```

**Change the network name (SSID):**
```bash
--ap wlan0 alfienet
#          ^^^^^^^^ change this
```

**Change the password:**
```bash
-p alfienet
#  ^^^^^^^^ change this (minimum 8 characters)
```

**Change the channel:**
```bash
-c 6
#  ^ change this (1-11 for 2.4GHz, 36-165 for 5GHz)
```

**Change frequency band:**
```bash
-g 5    # 5GHz
-g 2.4  # 2.4GHz
```

**Change country code:**
```bash
--country US
#         ^^ change this (US, GB, DE, etc.)
```

### Different WiFi Interface

If your wireless interface is not `wlan0`:

```bash
# Change this line:
sudo nmcli device set wlan0 managed no
#                     ^^^^^

# And this line:
--ap wlan0 alfiebot
#    ^^^^^
```

## Troubleshooting

### Error: "wlan0 not found"

**Solution:** Check your interface name with `ip link show` and update the script.

### Error: "Device or resource busy"

**Solution:** 
1. Stop NetworkManager from managing the interface:
   ```bash
   sudo nmcli device set wlan0 managed no
   ```
2. Kill any conflicting processes:
   ```bash
   sudo killall hostapd dnsmasq
   ```

### Error: "Operation not permitted"

**Solution:** Make sure you're running with `sudo`.

### Can't Connect to Hotspot

**Check if hotspot is running:**
```bash
iw dev wlan0 info
```

**Check DHCP server:**
```bash
ps aux | grep dnsmasq
```

**Check iptables rules:**
```bash
sudo iptables -L -v -n
```

### 5GHz Not Working

Some regions restrict 5GHz channels. Try:
- Switching to 2.4GHz: `-g 2.4`
- Using a different channel: `-c 36` or `-c 48` for 5GHz
- Verifying your country code is correct

### NetworkManager Interferes

Permanently prevent NetworkManager from managing `wlan0`:

```bash
sudo nano /etc/NetworkManager/NetworkManager.conf
```

Add under `[keyfile]`:
```ini
[keyfile]
unmanaged-devices=interface-name:wlan0
```

Restart NetworkManager:
```bash
sudo systemctl restart NetworkManager
```

## Advanced Usage

### Running in Background with Screen/Tmux

**Recommended for serial debug port usage!**

Using `screen` or `tmux` allows you to:
- Start the hotspot
- Detach from the session
- Disconnect the serial cable
- Reconnect later and reattach to stop the script

**Using Screen:**

```bash
# Start a screen session
screen -S alfienet

# Run the hotspot script
cd ~/alfiebot_ws/alfienet
sudo ./alfienet.sh

# Press Ctrl+A then D to detach (hotspot keeps running)

# Disconnect USB cable if desired

# Later, reconnect and reattach:
screen -r alfienet

# Press Ctrl+C to stop the hotspot
```

**Using Tmux:**

```bash
# Start a tmux session
tmux new -s alfienet

# Run the hotspot script
cd ~/alfiebot_ws/alfienet
sudo ./alfienet.sh

# Press Ctrl+B then D to detach (hotspot keeps running)

# Later, reconnect and reattach:
tmux attach -t alfienet

# Press Ctrl+C to stop the hotspot
```

**Benefits:**
- ✅ Clean session management
- ✅ Can safely disconnect serial cable
- ✅ Easy to reconnect and stop
- ✅ No risk of losing the process
- ✅ Can check status anytime by reattaching

### Automatic Startup on Boot

**Note:** This is typically NOT recommended for this script since it's meant for temporary debugging access. But if you need it:

Create a systemd service (similar to the alfiebot.service):

```bash
sudo nano /etc/systemd/system/alfienet.service
```

```ini
[Unit]
Description=Alfie WiFi Hotspot
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/alfie/alfiebot_ws/alfienet
ExecStart=/home/alfie/alfiebot_ws/alfienet/alfienet.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl enable alfienet.service
sudo systemctl start alfienet.service
```

### Internet Sharing

If your Jetson has internet access via ethernet (`eth0`), connected devices will automatically have internet access through the WiFi hotspot.

Check your internet connection:
```bash
ip route show
ping -c 3 8.8.8.8
```

### View Connected Devices

```bash
# Show DHCP leases
cat /var/lib/misc/dnsmasq.leases

# Or check ARP table
arp -n
```

## Command Reference

### Script Parameters Explained

```bash
sudo ./lnxrouter \
     -n              # No internet sharing (remove to enable)
     -g 5            # 5GHz band (use 2.4 for 2.4GHz)
     --ap wlan0 alfienet  # Create AP on wlan0, SSID: alfienet
     -p alfienet     # Password
     -c 6            # WiFi channel
     --country US    # Regulatory domain
```

### Common lnxrouter Options

- `-n` - No internet sharing (NAT disabled)
- `-g FREQ` - Frequency band (2.4 or 5)
- `--ap INTERFACE SSID` - Create AP
- `-p PASSWORD` - Set password (WPA2)
- `-c CHANNEL` - WiFi channel
- `--country CODE` - Country code
- `--no-dns` - Disable DNS server
- `--dhcp-range START,END` - Custom DHCP range

For more options:
```bash
./lnxrouter --help
```

## Security Notes

⚠️ **Important Security Considerations:**

1. **Change the default password!** The default `alfienet` is not secure.
2. Use WPA2 encryption (default)
3. Consider hiding SSID for added security (add `--hidden` option)
4. Monitor connected devices regularly
5. Use strong passwords (minimum 8 characters, mix of letters/numbers/symbols)

## Quick Reference Card

```
Start Hotspot:  cd ~/alfiebot_ws/alfienet && sudo ./alfienet.sh
Stop Hotspot:   Ctrl+C
SSID:           alfiebot
Password:       alfienet
IP Range:       192.168.12.x
Band:           5GHz
Channel:        6
```

## Support

If you encounter issues:

1. Check WiFi hardware support: `iw list`
2. Verify interface status: `ip link show wlan0`
3. Check system logs: `sudo journalctl -xe`
4. Test with 2.4GHz instead of 5GHz
5. Ensure no other hotspot software is running

For more information about lnxrouter, see:
- [lnxrouter GitHub](https://github.com/garywill/linux-router)
- [lnxrouter Documentation](./README.md)

## Related Documentation

- [Alfie Robot Main Documentation](../README.md)
- [Alfie Bringup Service](../src/alfie_bringup/systemd/README.md)
