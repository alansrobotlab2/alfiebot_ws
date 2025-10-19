#!/bin/bash
# Alfie Robot Systemd Service Installer
# This script installs the alfie.service systemd unit

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_FILE="$SCRIPT_DIR/alfie.service"
SERVICE_NAME="alfie.service"
SYSTEMD_DIR="/etc/systemd/system"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Alfie Robot Systemd Service Installer${NC}"
echo -e "${GREEN}========================================${NC}"
echo

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}ERROR: Do not run this script as root or with sudo${NC}"
    echo "Run it as your normal user. It will ask for sudo password when needed."
    exit 1
fi

# Check if service file exists
if [ ! -f "$SERVICE_FILE" ]; then
    echo -e "${RED}ERROR: Service file not found at $SERVICE_FILE${NC}"
    exit 1
fi

echo -e "${YELLOW}Checking current user and groups...${NC}"
CURRENT_USER=$(whoami)
echo "Current user: $CURRENT_USER"

# Check if user is in required groups
REQUIRED_GROUPS=("video" "dialout" "i2c" "gpio")
MISSING_GROUPS=()

for group in "${REQUIRED_GROUPS[@]}"; do
    if ! groups "$CURRENT_USER" | grep -q "\b$group\b"; then
        MISSING_GROUPS+=("$group")
    fi
done

if [ ${#MISSING_GROUPS[@]} -ne 0 ]; then
    echo -e "${YELLOW}Warning: User $CURRENT_USER is not in the following groups: ${MISSING_GROUPS[*]}${NC}"
    echo -e "${YELLOW}Adding user to required groups...${NC}"
    for group in "${MISSING_GROUPS[@]}"; do
        # Check if group exists, create if it doesn't
        if ! getent group "$group" > /dev/null 2>&1; then
            echo "Creating group: $group"
            sudo groupadd "$group"
        fi
        echo "Adding $CURRENT_USER to $group"
        sudo usermod -a -G "$group" "$CURRENT_USER"
    done
    echo -e "${GREEN}User added to groups. You may need to log out and back in for changes to take effect.${NC}"
fi

# Update service file with current user
echo -e "${YELLOW}Updating service file with current user ($CURRENT_USER)...${NC}"
TEMP_SERVICE="/tmp/alfie.service.tmp"
sed "s/User=alfie/User=$CURRENT_USER/g; s/Group=alfie/Group=$CURRENT_USER/g; s|/home/alfie|$HOME|g" "$SERVICE_FILE" > "$TEMP_SERVICE"

# Stop the service if it's already running
if systemctl is-active --quiet "$SERVICE_NAME"; then
    echo -e "${YELLOW}Stopping existing alfie service...${NC}"
    sudo systemctl stop "$SERVICE_NAME"
fi

# Disable the service if it's enabled
if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo -e "${YELLOW}Disabling existing alfie service...${NC}"
    sudo systemctl disable "$SERVICE_NAME"
fi

# Copy service file to systemd directory
echo -e "${YELLOW}Installing service file to $SYSTEMD_DIR...${NC}"
sudo cp "$TEMP_SERVICE" "$SYSTEMD_DIR/$SERVICE_NAME"
sudo chmod 644 "$SYSTEMD_DIR/$SERVICE_NAME"
rm "$TEMP_SERVICE"

# Reload systemd daemon
echo -e "${YELLOW}Reloading systemd daemon...${NC}"
sudo systemctl daemon-reload

# Enable the service
echo -e "${YELLOW}Enabling alfie service to start on boot...${NC}"
sudo systemctl enable "$SERVICE_NAME"

echo
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo
echo "Service Status Commands:"
echo "  Start service:    sudo systemctl start $SERVICE_NAME"
echo "  Stop service:     sudo systemctl stop $SERVICE_NAME"
echo "  Restart service:  sudo systemctl restart $SERVICE_NAME"
echo "  Check status:     sudo systemctl status $SERVICE_NAME"
echo "  View logs:        sudo journalctl -u $SERVICE_NAME -f"
echo "  Disable autostart: sudo systemctl disable $SERVICE_NAME"
echo
echo -e "${YELLOW}Note: The service will start automatically on next boot.${NC}"
echo -e "${YELLOW}To start it now, run: sudo systemctl start $SERVICE_NAME${NC}"
echo

read -p "Do you want to start the service now? (y/N) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}Starting alfie service...${NC}"
    sudo systemctl start "$SERVICE_NAME"
    sleep 2
    sudo systemctl status "$SERVICE_NAME" --no-pager
fi

echo
echo -e "${GREEN}Done!${NC}"
