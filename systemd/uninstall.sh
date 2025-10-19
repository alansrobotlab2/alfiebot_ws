#!/bin/bash
# Alfie Robot Systemd Service Uninstaller
# This script removes the alfiebot.service systemd unit

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SERVICE_NAME="alfiebot.service"
SYSTEMD_DIR="/etc/systemd/system"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Alfie Robot Systemd Service Uninstaller${NC}"
echo -e "${GREEN}========================================${NC}"
echo

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}ERROR: Do not run this script as root or with sudo${NC}"
    echo "Run it as your normal user. It will ask for sudo password when needed."
    exit 1
fi

# Check if service file exists
if [ ! -f "$SYSTEMD_DIR/$SERVICE_NAME" ]; then
    echo -e "${YELLOW}Service file not found at $SYSTEMD_DIR/$SERVICE_NAME${NC}"
    echo "Service may not be installed."
    exit 0
fi

# Stop the service if it's running
if systemctl is-active --quiet "$SERVICE_NAME"; then
    echo -e "${YELLOW}Stopping alfie service...${NC}"
    sudo systemctl stop "$SERVICE_NAME"
    echo -e "${GREEN}Service stopped.${NC}"
fi

# Disable the service if it's enabled
if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo -e "${YELLOW}Disabling alfie service...${NC}"
    sudo systemctl disable "$SERVICE_NAME"
    echo -e "${GREEN}Service disabled.${NC}"
fi

# Remove service file
echo -e "${YELLOW}Removing service file...${NC}"
sudo rm -f "$SYSTEMD_DIR/$SERVICE_NAME"
echo -e "${GREEN}Service file removed.${NC}"

# Reload systemd daemon
echo -e "${YELLOW}Reloading systemd daemon...${NC}"
sudo systemctl daemon-reload
sudo systemctl reset-failed

echo
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Uninstallation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo
echo "The alfie service has been removed and will no longer start on boot."
echo
