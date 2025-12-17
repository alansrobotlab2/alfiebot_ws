#!/bin/bash
# Test nvjpegenc with different settings to find what works

echo "Testing nvjpegenc with various configurations..."
echo ""

# Test 1: Basic nvjpegenc
echo "Test 1: Basic pipeline (10 frames)..."
timeout 5 gst-launch-1.0 -e \
    v4l2src device=/dev/video0 num-buffers=10 ! \
    'image/jpeg,width=1600,height=600,framerate=15/1' ! \
    nvv4l2decoder mjpeg=1 ! \
    nvvidconv flip-method=2 ! \
    'video/x-raw(memory:NVMM)' ! \
    nvjpegenc ! \
    fakesink 2>&1 | tail -5
echo ""

# Test 2: With quality parameter
echo "Test 2: With quality=85 (10 frames)..."
timeout 5 gst-launch-1.0 -e \
    v4l2src device=/dev/video0 num-buffers=10 ! \
    'image/jpeg,width=1600,height=600,framerate=15/1' ! \
    nvv4l2decoder mjpeg=1 ! \
    nvvidconv flip-method=2 ! \
    'video/x-raw(memory:NVMM)' ! \
    nvjpegenc quality=85 ! \
    fakesink 2>&1 | tail -5
echo ""

# Test 3: Without NVMM memory (force download to CPU then encode)
echo "Test 3: Without NVMM (10 frames)..."
timeout 5 gst-launch-1.0 -e \
    v4l2src device=/dev/video0 num-buffers=10 ! \
    'image/jpeg,width=1600,height=600,framerate=15/1' ! \
    nvv4l2decoder mjpeg=1 ! \
    nvvidconv flip-method=2 ! \
    'video/x-raw' ! \
    nvjpegenc quality=85 ! \
    fakesink 2>&1 | tail -5
echo ""

# Test 4: Check nvjpegenc caps
echo "Test 4: nvjpegenc capabilities..."
gst-inspect-1.0 nvjpegenc | grep -A 20 "Pad Templates"
echo ""

echo "Tests complete!"
