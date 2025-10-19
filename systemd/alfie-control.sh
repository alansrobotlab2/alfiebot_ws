#!/bin/bash
# Quick status and control script for Alfie service

SERVICE_NAME="alfie.service"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

show_status() {
    echo -e "${GREEN}=== Alfie Service Status ===${NC}"
    sudo systemctl status "$SERVICE_NAME" --no-pager
}

show_logs() {
    echo -e "${GREEN}=== Recent Alfie Logs (Ctrl+C to exit) ===${NC}"
    sudo journalctl -u "$SERVICE_NAME" -f
}

show_help() {
    echo "Alfie Service Control Script"
    echo
    echo "Usage: $0 [command]"
    echo
    echo "Commands:"
    echo "  status    - Show service status"
    echo "  start     - Start the service"
    echo "  stop      - Stop the service"
    echo "  restart   - Restart the service"
    echo "  enable    - Enable autostart on boot"
    echo "  disable   - Disable autostart on boot"
    echo "  logs      - Show live logs (Ctrl+C to exit)"
    echo "  recent    - Show recent logs"
    echo "  help      - Show this help message"
    echo
}

case "$1" in
    status)
        show_status
        ;;
    start)
        echo -e "${YELLOW}Starting alfie service...${NC}"
        sudo systemctl start "$SERVICE_NAME"
        sleep 1
        show_status
        ;;
    stop)
        echo -e "${YELLOW}Stopping alfie service...${NC}"
        sudo systemctl stop "$SERVICE_NAME"
        sleep 1
        show_status
        ;;
    restart)
        echo -e "${YELLOW}Restarting alfie service...${NC}"
        sudo systemctl restart "$SERVICE_NAME"
        sleep 1
        show_status
        ;;
    enable)
        echo -e "${YELLOW}Enabling alfie service autostart...${NC}"
        sudo systemctl enable "$SERVICE_NAME"
        echo -e "${GREEN}Service will now start on boot${NC}"
        ;;
    disable)
        echo -e "${YELLOW}Disabling alfie service autostart...${NC}"
        sudo systemctl disable "$SERVICE_NAME"
        echo -e "${GREEN}Service will not start on boot${NC}"
        ;;
    logs)
        show_logs
        ;;
    recent)
        echo -e "${GREEN}=== Recent Alfie Logs ===${NC}"
        sudo journalctl -u "$SERVICE_NAME" -n 100 --no-pager
        ;;
    help|"")
        show_help
        ;;
    *)
        echo -e "${RED}Unknown command: $1${NC}"
        echo
        show_help
        exit 1
        ;;
esac
