#!/bin/bash
# Quick start script for ServoTool2

# Check if gradio is installed
if ! python3 -c "import gradio" 2>/dev/null; then
    echo "📦 Gradio not found. Installing..."
    pip install gradio
fi

# Source ROS2 workspace
echo "🔧 Sourcing workspace..."
source /home/alfie/alfiebot_ws/install/setup.bash

# Launch servotool2
echo "🚀 Launching ServoTool2..."
echo ""
echo "Open your browser to: http://localhost:7860"
echo "Or from another device: http://$(hostname -I | awk '{print $1}'):7860"
echo ""
ros2 run alfie_tools servotool2
