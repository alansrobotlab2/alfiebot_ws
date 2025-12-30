#!/bin/bash
# Convenience script for recording demonstrations

set -e

# Default values
OUTPUT_DIR="${HOME}/alfiebot_ws/data/demonstrations"
DURATION=300

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -o, --output DIR      Output directory (default: ~/alfiebot_ws/data/demonstrations)"
            echo "  -d, --duration SECS   Max duration in seconds (default: 300)"
            echo "  -h, --help           Show this help message"
            echo ""
            echo "Controls:"
            echo "  Start recording:  ros2 topic pub --once /recording/start std_msgs/msg/Bool '{data: true}'"
            echo "  Stop recording:   ros2 topic pub --once /recording/stop std_msgs/msg/Bool '{data: true}'"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo "================================================"
echo "GR00T Data Collection - Demonstration Recording"
echo "================================================"
echo ""
echo "Output directory: $OUTPUT_DIR"
echo "Max duration:     $DURATION seconds"
echo ""
echo "Starting data recorder node..."
echo ""

# Launch data collection
ros2 launch alfie_gr00t data_collection.launch.py \
    output_dir:="$OUTPUT_DIR" \
    max_duration:="$DURATION"
