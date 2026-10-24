#!/bin/bash
# Convenience script for recording demonstrations

set -e

# Default values
DATA_ROOT="${ALFIE_DATA_ROOT:-${HOME}/alfiebot_ws/data}"
DATASET="${ALFIE_DATASET:-default}"
OUTPUT_DIR=""
DURATION=300

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -D|--dataset)
            DATASET="$2"
            shift 2
            ;;
        -r|--data-root)
            DATA_ROOT="$2"
            shift 2
            ;;
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
            echo "Bags are written to <data-root>/<dataset>/demonstrations/"
            echo ""
            echo "Options:"
            echo "  -D, --dataset NAME    Dataset name (default: ${DATASET}, or \$ALFIE_DATASET)"
            echo "  -r, --data-root DIR   Data root (default: ${DATA_ROOT}, or \$ALFIE_DATA_ROOT)"
            echo "  -o, --output DIR      Explicit bag directory, overrides dataset layout"
            echo "  -d, --duration SECS   Max duration in seconds (default: 300)"
            echo "  -h, --help            Show this help message"
            echo ""
            echo "Controls:"
            echo "  Start recording:  ros2 topic pub --once /alfie/recording/start std_msgs/msg/Bool '{data: true}'"
            echo "  Stop recording:   ros2 topic pub --once /alfie/recording/stop std_msgs/msg/Bool '{data: true}'"
            echo "  Switch dataset:   ros2 param set /alfie/data_recorder dataset <name>"
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
echo "Dataset:          $DATASET"
echo "Data root:        $DATA_ROOT"
if [[ -n "$OUTPUT_DIR" ]]; then
    echo "Output override:  $OUTPUT_DIR"
fi
echo "Max duration:     $DURATION seconds"
echo ""
echo "Starting data recorder node..."
echo ""

# Launch data collection
ros2 launch alfie_gr00t data_collection.launch.py \
    data_root:="$DATA_ROOT" \
    dataset:="$DATASET" \
    output_dir:="$OUTPUT_DIR" \
    max_duration:="$DURATION"
