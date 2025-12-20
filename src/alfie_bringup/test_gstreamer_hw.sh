#!/bin/bash
# Test script for GStreamer hardware acceleration on Jetson

echo "========================================="
echo "GStreamer Hardware Acceleration Test"
echo "========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check GStreamer installation
echo "1. Checking GStreamer installation..."
if command -v gst-inspect-1.0 &> /dev/null; then
    echo -e "${GREEN}✓${NC} GStreamer found"
else
    echo -e "${RED}✗${NC} GStreamer not found"
    exit 1
fi
echo ""

# Test 2: Check for required Nvidia plugins
echo "2. Checking Nvidia GStreamer plugins..."
REQUIRED_PLUGINS=("nvv4l2decoder" "nvvidconv" "nvjpegenc")
ALL_FOUND=true

for plugin in "${REQUIRED_PLUGINS[@]}"; do
    if gst-inspect-1.0 "$plugin" &> /dev/null; then
        echo -e "   ${GREEN}✓${NC} $plugin"
    else
        echo -e "   ${RED}✗${NC} $plugin NOT FOUND"
        ALL_FOUND=false
    fi
done
echo ""

if [ "$ALL_FOUND" = false ]; then
    echo -e "${RED}Missing required plugins!${NC}"
    echo "Install with: sudo apt-get install gstreamer1.0-plugins-nvvideo4linux2"
    echo ""
fi

# Test 3: Check camera device
DEVICE="/dev/video0"
echo "3. Checking camera device ($DEVICE)..."
if [ -e "$DEVICE" ]; then
    echo -e "${GREEN}✓${NC} Camera device exists"
    v4l2-ctl --device=$DEVICE --list-formats-ext 2>/dev/null | head -20
else
    echo -e "${RED}✗${NC} Camera device not found"
fi
echo ""

# Test 4: Test simple v4l2 capture (no decode)
echo "4. Testing v4l2 MJPEG capture (5 seconds)..."
echo "   Pipeline: v4l2src -> fakesink"
timeout 5 gst-launch-1.0 \
    v4l2src device=/dev/video0 num-buffers=150 ! \
    'image/jpeg,width=2560,height=720,framerate=30/1' ! \
    fakesink 2>&1 | tail -5
echo ""

# Test 5: Test hardware decoder
if [ "$ALL_FOUND" = true ]; then
    echo "5. Testing hardware MJPEG decode (5 seconds)..."
    echo "   Pipeline: v4l2src -> nvv4l2decoder -> fakesink"
    timeout 5 gst-launch-1.0 \
        v4l2src device=/dev/video0 num-buffers=150 ! \
        'image/jpeg,width=2560,height=720,framerate=30/1' ! \
        nvv4l2decoder mjpeg=1 ! \
        fakesink 2>&1 | tail -5
    echo ""
    
    # Test 6: Test full hardware pipeline
    echo "6. Testing full hardware pipeline with flip (5 seconds)..."
    echo "   Pipeline: v4l2src -> nvv4l2decoder -> nvvidconv(flip) -> nvjpegenc -> fakesink"
    timeout 5 gst-launch-1.0 \
        v4l2src device=/dev/video0 num-buffers=150 ! \
        'image/jpeg,width=2560,height=720,framerate=30/1' ! \
        nvv4l2decoder mjpeg=1 ! \
        nvvidconv flip-method=2 ! \
        'video/x-raw(memory:NVMM)' ! \
        nvjpegenc quality=95 ! \
        fakesink 2>&1 | tail -5
    echo ""
    
    # Test 7: Test tee pipeline (ROS + WebRTC)
    echo "7. Testing tee pipeline (ROS JPEG + WebRTC RGB) (5 seconds)..."
    echo "   This simulates the actual node pipeline"
    timeout 5 gst-launch-1.0 \
        v4l2src device=/dev/video0 num-buffers=150 ! \
        'image/jpeg,width=2560,height=720,framerate=30/1' ! \
        nvv4l2decoder mjpeg=1 ! \
        nvvidconv flip-method=2 ! \
        'video/x-raw(memory:NVMM)' ! \
        tee name=t ! \
        queue max-size-buffers=1 leaky=downstream ! \
        nvjpegenc quality=95 ! \
        fakesink \
        t. ! queue max-size-buffers=1 leaky=downstream ! \
        nvvidconv ! \
        'video/x-raw,format=RGB' ! \
        fakesink 2>&1 | tail -5
    echo ""
fi

echo "========================================="
echo "Test complete!"
echo "========================================="
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo "1. If all tests passed, restart the camera node"
echo "2. Monitor CPU usage with: htop"
echo "3. Check node logs for '[HW]' to confirm hardware mode"
echo ""
