#!/bin/bash
# Quick Reference for Alfienet WiFi Hotspot

cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║              Alfienet WiFi Hotspot Quick Guide              ║
╚══════════════════════════════════════════════════════════════╝

⚠️  TYPICAL USE CASE:
  This script is designed for debugging and initial setup via the
  USB-C serial debug port.

  WORKFLOW:
  1. Connect to robot via USB-C serial debug port
  2. After robot boots, log in and run this script
  3. Script temporarily converts wlan0 into a WiFi hotspot
  4. You can DISCONNECT from serial - the hotspot persists
  5. Connect your device to the "alfienet" WiFi network
  6. Access the robot wirelessly for configuration
  7. RECONNECT to serial and press Ctrl+C to stop the hotspot
  8. WiFi returns to normal settings automatically
  9. On reboot, all original WiFi settings are restored

  This provides temporary wireless access without permanently
  changing your WiFi configuration!

CURRENT SETTINGS:
  Network Name (SSID): alfienet
  Password:            alfienet
  Frequency:           5GHz
  Channel:             6
  Security:            WPA2

HOW TO USE THIS SCRIPT:
  1. Install dependencies (one-time setup):
     sudo apt install hostapd dnsmasq iptables iw network-manager

  2. Connect to robot via USB-C serial debug port

  3. After boot, start the hotspot:
     cd ~/alfiebot_ws/alfienet
     sudo ./alfienet.sh

  4. Disconnect from serial (hotspot keeps running!)

  5. Connect your device to WiFi "alfienet"

  6. Access robot wirelessly for your work

  7. Reconnect to serial when done

  8. Press Ctrl+C to stop hotspot and restore WiFi

CONNECTING TO THE HOTSPOT:
  On your device:
  1. Search for WiFi network "alfienet"
  2. Enter password "alfienet"
  3. Connect!

CUSTOMIZATION:
  Edit alfienet.sh to change:
  • Network name:  --ap wlan0 YOUR_NAME
  • Password:      -p YOUR_PASSWORD
  • Channel:       -c CHANNEL_NUMBER
  • Frequency:     -g 2.4 or -g 5

TROUBLESHOOTING:
  Interface busy:
    sudo nmcli device set wlan0 managed no
    sudo killall hostapd dnsmasq

  Check if running:
    iw dev wlan0 info
    ps aux | grep dnsmasq

  View connected devices:
    cat /var/lib/misc/dnsmasq.leases
    arp -n

  Change to 2.4GHz if 5GHz doesn't work:
    Edit script: -g 2.4 and -c 6

COMMON COMMANDS:
  Check WiFi interface:
    ip link show

  Verify AP mode support:
    iw list | grep -A 10 "Supported interface modes"

  View logs:
    sudo journalctl -xe

SECURITY WARNING:
  ⚠️  Change the default password!
  Edit alfienet.sh and change "alfienet" to a strong password.

FULL DOCUMENTATION:
  ~/alfiebot_ws/alfienet/HOWTO.md

EOF
