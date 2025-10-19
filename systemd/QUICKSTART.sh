#!/bin/bash
# Quick Setup Guide for Alfie Autostart Service

cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║           Alfie Robot Autostart Service Setup               ║
╚══════════════════════════════════════════════════════════════╝

This will install a systemd service to automatically start Alfie
when your Jetson boots up.

FEATURES:
  ✓ Waits for network connectivity
  ✓ 10-second delay for hardware initialization
  ✓ Automatic restart on failure
  ✓ Comprehensive logging via journalctl
  ✓ Runs as your user account (not root)

QUICK SETUP:
  1. Make sure your workspace is built:
     cd ~/alfiebot_ws && colcon build

  2. Install the service:
     cd ~/alfiebot_ws/src/alfie_bringup/systemd
     ./install.sh

  3. The service will start on next boot, or start now with:
     sudo systemctl start alfiebot.service

MANAGEMENT:
  Use the control script for easy management:
    cd ~/alfiebot_ws/src/alfie_bringup/systemd
    ./alfie-control.sh status    # Check status
    ./alfie-control.sh logs      # View live logs
    ./alfie-control.sh restart   # Restart service

  Or use standard systemd commands:
    sudo systemctl status alfiebot.service
    sudo systemctl restart alfiebot.service
    sudo journalctl -u alfiebot.service -f

REMOVAL:
  To uninstall the service:
    cd ~/alfiebot_ws/src/alfie_bringup/systemd
    ./uninstall.sh

DOCUMENTATION:
  Full documentation available at:
    ~/alfiebot_ws/src/alfie_bringup/systemd/README.md

TROUBLESHOOTING:
  • Service won't start: Check logs with journalctl
  • Permission errors: Ensure you're in video,dialout,i2c,gpio groups
  • Hardware not ready: Increase delay in service file

Ready to install? Run:
  cd ~/alfiebot_ws/src/alfie_bringup/systemd && ./install.sh

EOF
