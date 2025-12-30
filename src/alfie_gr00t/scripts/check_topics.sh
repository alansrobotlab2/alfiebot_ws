#!/bin/bash
# Check if all required topics for data recording are available

set -e

echo "================================================"
echo "GR00T Data Collection - Topic Availability Check"
echo "================================================"
echo ""

# Define required topics
CAMERA_TOPICS=(
    "/alfie/stereo_camera/left_wide/image_raw/compressed"
    "/alfie/stereo_camera/left_center/image_raw/compressed"
    "/alfie/stereo_camera/right_center/image_raw/compressed"
    "/alfie/stereo_camera/right_wide/image_raw/compressed"
)

STATE_TOPICS=(
    "/alfie/joint_states"
    "/alfie/robotlowstate"
    "/alfie/low/gdb0state"
    "/alfie/low/gdb1state"
)

COMMAND_TOPICS=(
    "/alfie/robotlowcmd"
    "/alfie/low/gdb0cmd"
    "/alfie/low/gdb1cmd"
    "/alfie/low/backcmd"
)

TF_TOPICS=(
    "/alfie/tf"
    "/alfie/tf_static"
)

# Get list of all available topics
ALL_TOPICS=$(ros2 topic list)

check_topic() {
    local topic=$1
    if echo "$ALL_TOPICS" | grep -q "^${topic}$"; then
        echo "  ✓ $topic"
        return 0
    else
        echo "  ✗ $topic (MISSING)"
        return 1
    fi
}

# Check cameras
echo "Camera Topics:"
missing_cameras=0
for topic in "${CAMERA_TOPICS[@]}"; do
    check_topic "$topic" || ((missing_cameras++))
done
echo ""

# Check state topics
echo "Robot State Topics:"
missing_state=0
for topic in "${STATE_TOPICS[@]}"; do
    check_topic "$topic" || ((missing_state++))
done
echo ""

# Check command topics
echo "Robot Command Topics:"
missing_commands=0
for topic in "${COMMAND_TOPICS[@]}"; do
    check_topic "$topic" || ((missing_commands++))
done
echo ""

# Check TF
echo "Transform Topics:"
missing_tf=0
for topic in "${TF_TOPICS[@]}"; do
    check_topic "$topic" || ((missing_tf++))
done
echo ""

# Summary
total_missing=$((missing_cameras + missing_state + missing_commands + missing_tf))

echo "================================================"
if [ $total_missing -eq 0 ]; then
    echo "✓ All required topics are available!"
    echo "  Ready to start data collection."
    exit 0
else
    echo "✗ Missing $total_missing topic(s)"
    echo ""
    echo "Suggestions:"
    if [ $missing_cameras -gt 0 ]; then
        echo "  - Start camera nodes (OAK-D stereo cameras)"
    fi
    if [ $missing_state -gt 0 ] || [ $missing_commands -gt 0 ]; then
        echo "  - Start Alfie robot driver/controller nodes"
    fi
    if [ $missing_tf -gt 0 ]; then
        echo "  - Verify robot_state_publisher or TF broadcasting nodes"
    fi
    echo ""
    echo "Run 'ros2 topic list' to see all available topics"
    exit 1
fi
