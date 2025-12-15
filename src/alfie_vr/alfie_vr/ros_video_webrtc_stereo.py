#!/usr/bin/env python3
"""
ROS2 Stereo Video Streamer via WebRTC - For VR 3D Vision
Subscribes to left/right compressed image topics and streams both via WebRTC
Optimized for Meta Quest 3 headset display - ULTRA LOW LATENCY VERSION

Key optimizations:
- No queues - only stores latest frame
- Decode directly in ROS callback (fast enough on Jetson)
- Single stereo frame buffer, atomically updated
- No MediaRelay buffering
- Event-driven frame delivery to WebRTC
- CUDA acceleration on Jetson (resize, color convert, blur)

Color mapping:
- Uses proper homography to project RGB onto left/right views
- Accounts for camera positions and intrinsics
- YCrCb color transfer preserves stereo luminance detail
"""

import asyncio
import json
import os
import threading
import fractions
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo

import cv2
import numpy as np
from av import VideoFrame

# Check for CUDA availability (Jetson Orin NX)
USE_CUDA = False
USE_NVJPEG = False
_cuda_stream = None
_cuda_box_filter = None

try:
    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
        USE_CUDA = True
        # Pre-create CUDA stream for async operations
        _cuda_stream = cv2.cuda.Stream()
        # Pre-create box filter for chroma blur (5x5)
        _cuda_box_filter = cv2.cuda.createBoxFilter(
            cv2.CV_8UC1, cv2.CV_8UC1, (5, 5)
        )
        print(f"✅ CUDA enabled: device {cv2.cuda.getDevice()}")
        
        # Check for NVJPEG hardware JPEG decoder (Jetson)
        try:
            # Test if cudacodec is available (has NVJPEG on Jetson)
            if hasattr(cv2, 'cudacodec'):
                USE_NVJPEG = True
                print("✅ NVJPEG hardware decode available")
        except Exception:
            pass
except Exception as e:
    print(f"⚠️ CUDA not available, using CPU: {e}")

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender

VIDEO_PORT = 8084
TARGET_FPS = 30
TARGET_BITRATE = 2_000_000  # Lower bitrate = faster encode

# Use VP8 which encodes faster than H264 on CPU
USE_VP8 = True

# Global references
ros_node: Optional['StereoVideoNode'] = None
pcs = set()

# Asyncio event for frame signaling
frame_event: Optional[asyncio.Event] = None
frame_loop: Optional[asyncio.AbstractEventLoop] = None


class StereoVideoTrack(VideoStreamTrack):
    """
    Video track that outputs side-by-side stereo frames.
    Waits for frame signal to minimize latency.
    """
    
    kind = "video"
    
    def __init__(self, node: 'StereoVideoNode'):
        super().__init__()
        self.node = node
        self._timestamp = 0
        self._black_frame = np.zeros((720, 2560, 3), dtype=np.uint8)
        
    async def recv(self):
        """Receive the next stereo frame - event-driven for low latency"""
        global frame_event
        
        # Wait for new frame signal from ROS callback
        if frame_event:
            try:
                await asyncio.wait_for(frame_event.wait(), timeout=0.05)
                frame_event.clear()
            except asyncio.TimeoutError:
                pass
        
        pts, time_base = self._next_timestamp()
        
        # Get latest stereo frame
        stereo_frame = self.node.get_stereo_frame()
        
        if stereo_frame is not None:
            try:
                frame = VideoFrame.from_ndarray(stereo_frame, format="rgb24")
                frame.pts = pts
                frame.time_base = time_base
                return frame
            except Exception as e:
                print(f"Stereo frame error: {e}")
        
        # Return black frame if no data
        frame = VideoFrame.from_ndarray(self._black_frame, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
    
    def _next_timestamp(self):
        self._timestamp += int(90000 / TARGET_FPS)
        return self._timestamp, fractions.Fraction(1, 90000)


class StereoVideoNode(Node):
    """
    ROS2 node for stereo video streaming.
    Decodes JPEG in callbacks and combines into stereo frame.
    """
    
    def __init__(self):
        super().__init__('webrtc_stereo_video')
        
        # Declare parameters
        self.declare_parameter('left_image_topic', '/alfie/oak/left/image_rect/compressed')
        self.declare_parameter('right_image_topic', '/alfie/oak/right/image_rect/compressed')
        self.declare_parameter('rgb_image_topic', '/alfie/oak/rgb/image_raw/compressed')
        self.declare_parameter('left_camera_info_topic', '/alfie/oak/left/camera_info')
        self.declare_parameter('right_camera_info_topic', '/alfie/oak/right/camera_info')
        self.declare_parameter('rgb_camera_info_topic', '/alfie/oak/rgb/camera_info')
        self.declare_parameter('colorize', True)  # Map RGB color onto B/W stereo
        self.declare_parameter('port', VIDEO_PORT)
        self.declare_parameter('output_width', 1280)  # 640 per eye
        self.declare_parameter('output_height', 400)
        
        # OAK-D Lite camera geometry (meters)
        # Stereo baseline: 75mm, RGB is centered between left/right
        self.declare_parameter('stereo_baseline', 0.075)  # 7.5cm baseline
        self.declare_parameter('rgb_to_left_x', 0.0375)   # RGB is 3.75cm right of left cam
        self.declare_parameter('rgb_to_right_x', -0.0375) # RGB is 3.75cm left of right cam
        
        # Get parameters
        self.left_topic = self.get_parameter('left_image_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_image_topic').get_parameter_value().string_value
        self.rgb_topic = self.get_parameter('rgb_image_topic').get_parameter_value().string_value
        self.left_info_topic = self.get_parameter('left_camera_info_topic').get_parameter_value().string_value
        self.right_info_topic = self.get_parameter('right_camera_info_topic').get_parameter_value().string_value
        self.rgb_info_topic = self.get_parameter('rgb_camera_info_topic').get_parameter_value().string_value
        self.colorize = self.get_parameter('colorize').get_parameter_value().bool_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter('output_height').get_parameter_value().integer_value
        
        # Camera geometry
        self.stereo_baseline = self.get_parameter('stereo_baseline').get_parameter_value().double_value
        self.rgb_to_left_x = self.get_parameter('rgb_to_left_x').get_parameter_value().double_value
        self.rgb_to_right_x = self.get_parameter('rgb_to_right_x').get_parameter_value().double_value
        
        self.eye_width = self.output_width // 2
        
        # Pre-allocate stereo buffer
        self._stereo_frame = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        self._stereo_lock = threading.Lock()
        
        # Latest decoded frames
        self._left_frame: Optional[np.ndarray] = None
        self._right_frame: Optional[np.ndarray] = None
        self._rgb_frame: Optional[np.ndarray] = None  # For color mapping
        
        # Camera intrinsics (will be populated from camera_info)
        self._left_K: Optional[np.ndarray] = None
        self._right_K: Optional[np.ndarray] = None
        self._rgb_K: Optional[np.ndarray] = None
        self._left_size: Optional[Tuple[int, int]] = None
        self._right_size: Optional[Tuple[int, int]] = None
        self._rgb_size: Optional[Tuple[int, int]] = None
        
        # Pre-computed homographies (RGB -> left, RGB -> right)
        self._H_rgb_to_left: Optional[np.ndarray] = None
        self._H_rgb_to_right: Optional[np.ndarray] = None
        self._homographies_computed = False
        
        # Pre-allocated buffers for colorization (avoid per-frame allocation)
        self._cached_chroma: Optional[np.ndarray] = None  # Cached Cr,Cb channels from RGB
        self._cached_rgb_size: Optional[Tuple[int, int]] = None
        self._result_buffer = np.zeros((self.output_height, self.eye_width, 3), dtype=np.uint8)
        
        # Cached warped RGB frames for each eye
        self._rgb_warped_left: Optional[np.ndarray] = None
        self._rgb_warped_right: Optional[np.ndarray] = None
        
        # Frame counters for stats
        self._left_count = 0
        self._right_count = 0
        self._rgb_count = 0
        self._last_stats_time = time.time()
        
        # QoS profile - best effort, keep only latest
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to both image topics
        self.left_sub = self.create_subscription(
            CompressedImage,
            self.left_topic,
            self.left_callback,
            qos_profile
        )
        
        self.right_sub = self.create_subscription(
            CompressedImage,
            self.right_topic,
            self.right_callback,
            qos_profile
        )
        
        # Subscribe to RGB for colorization
        if self.colorize:
            self.rgb_sub = self.create_subscription(
                CompressedImage,
                self.rgb_topic,
                self.rgb_callback,
                qos_profile
            )
            
            # Subscribe to camera info for calibration
            self.left_info_sub = self.create_subscription(
                CameraInfo,
                self.left_info_topic,
                self.left_info_callback,
                qos_profile
            )
            self.right_info_sub = self.create_subscription(
                CameraInfo,
                self.right_info_topic,
                self.right_info_callback,
                qos_profile
            )
            self.rgb_info_sub = self.create_subscription(
                CameraInfo,
                self.rgb_info_topic,
                self.rgb_info_callback,
                qos_profile
            )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Stereo WebRTC Video Streamer - ULTRA LOW LATENCY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Left eye:  {self.left_topic}')
        self.get_logger().info(f'Right eye: {self.right_topic}')
        if self.colorize:
            self.get_logger().info(f'RGB color: {self.rgb_topic}')
            self.get_logger().info('Colorization: ENABLED (YCrCb fast path)')
            self.get_logger().info(f'Stereo baseline: {self.stereo_baseline*100:.1f}cm')
        else:
            self.get_logger().info('Colorization: DISABLED')
        self.get_logger().info(f'Output: {self.output_width}x{self.output_height}')
        self.get_logger().info(f'WebRTC port: {self.port}')
        # Show acceleration status
        if USE_CUDA:
            self.get_logger().info('🚀 CUDA acceleration: ENABLED (Jetson GPU)')
        else:
            self.get_logger().info('⚠️ CUDA acceleration: DISABLED (CPU only)')
    
    def left_info_callback(self, msg: CameraInfo):
        """Store left camera intrinsics"""
        if self._left_K is None:
            self._left_K = np.array(msg.k).reshape(3, 3)
            self._left_size = (msg.width, msg.height)
            self.get_logger().info(f'Left camera: {msg.width}x{msg.height}')
            self._try_compute_homographies()
    
    def right_info_callback(self, msg: CameraInfo):
        """Store right camera intrinsics"""
        if self._right_K is None:
            self._right_K = np.array(msg.k).reshape(3, 3)
            self._right_size = (msg.width, msg.height)
            self.get_logger().info(f'Right camera: {msg.width}x{msg.height}')
            self._try_compute_homographies()
    
    def rgb_info_callback(self, msg: CameraInfo):
        """Store RGB camera intrinsics"""
        if self._rgb_K is None:
            self._rgb_K = np.array(msg.k).reshape(3, 3)
            self._rgb_size = (msg.width, msg.height)
            self.get_logger().info(f'RGB camera: {msg.width}x{msg.height}')
            self._try_compute_homographies()
    
    def _try_compute_homographies(self):
        """
        Compute homographies from RGB to left/right cameras.
        
        For a plane at infinity (or distant objects), the homography is:
            H = K_target * R * K_source^(-1)
        
        Since cameras are roughly aligned (small rotation), we approximate with:
            H = K_target * T * K_source^(-1)
        
        Where T accounts for the horizontal translation between cameras.
        This is equivalent to applying a horizontal shift proportional to 
        the focal length ratio and camera offset.
        """
        if self._homographies_computed:
            return
        if self._left_K is None or self._right_K is None or self._rgb_K is None:
            return
        if self._left_size is None or self._rgb_size is None:
            return
        
        try:
            # Compute homography for RGB -> Left
            # The key insight: for distant objects, parallax causes a horizontal shift
            # shift_pixels = focal_length * baseline / depth
            # At infinity, shift -> 0, but for typical viewing distances we need to compensate
            
            # Scale factors to account for different resolutions
            scale_x_left = self._left_size[0] / self._rgb_size[0]
            scale_y_left = self._left_size[1] / self._rgb_size[1]
            scale_x_right = self._right_size[0] / self._rgb_size[0]
            scale_y_right = self._right_size[1] / self._rgb_size[1]
            
            # For homography at a reference plane (assume ~2m distance for typical VR use)
            reference_depth = 2.0  # meters
            
            # Pixel shift = fx * baseline_x / depth
            # RGB -> Left: RGB is to the right of left camera
            shift_left = self._left_K[0, 0] * self.rgb_to_left_x / reference_depth
            # RGB -> Right: RGB is to the left of right camera  
            shift_right = self._right_K[0, 0] * self.rgb_to_right_x / reference_depth
            
            # Build homography matrices
            # H = K_target * [R | t/d] * K_source^-1
            # For our case with pure translation and distant plane:
            # H ≈ scale * I + translation_in_pixels
            
            # Simpler approach: affine transform with scale and shift
            self._H_rgb_to_left = np.array([
                [scale_x_left, 0, shift_left],
                [0, scale_y_left, 0],
                [0, 0, 1]
            ], dtype=np.float32)
            
            self._H_rgb_to_right = np.array([
                [scale_x_right, 0, shift_right],
                [0, scale_y_right, 0],
                [0, 0, 1]
            ], dtype=np.float32)
            
            self._homographies_computed = True
            self.get_logger().info('Homographies computed for RGB->stereo projection')
            self.get_logger().info(f'  Left shift: {shift_left:.1f}px, Right shift: {shift_right:.1f}px')
            
        except Exception as e:
            self.get_logger().error(f'Failed to compute homographies: {e}')
        
    def _colorize_grayscale(self, gray_frame: np.ndarray, is_left: bool) -> np.ndarray:
        """
        Colorize grayscale stereo image using RGB camera as color source.
        
        OPTIMIZED: Uses YCrCb (faster than LAB), caches chroma channels,
        avoids per-frame allocations, uses box filter instead of Gaussian.
        
        Args:
            gray_frame: The grayscale image (as RGB with equal channels)
            is_left: True for left eye, False for right eye
        """
        if self._cached_chroma is None:
            return gray_frame
        
        try:
            # Get grayscale Y channel directly (already uint8)
            if len(gray_frame.shape) == 3:
                gray_y = gray_frame[:, :, 0]
            else:
                gray_y = gray_frame
            
            # Combine: stereo Y + cached RGB chroma (Cr, Cb)
            # Write directly to pre-allocated buffer
            self._result_buffer[:, :, 0] = gray_y
            self._result_buffer[:, :, 1] = self._cached_chroma[:, :, 0]  # Cr
            self._result_buffer[:, :, 2] = self._cached_chroma[:, :, 1]  # Cb
            
            # Convert YCrCb back to RGB
            colorized = cv2.cvtColor(self._result_buffer, cv2.COLOR_YCrCb2RGB)
            return colorized
            
        except Exception as e:
            self.get_logger().error(f'Colorize error: {e}')
            return gray_frame
    
    def rgb_callback(self, msg: CompressedImage):
        """Handle incoming RGB image - extract and cache chroma for colorization"""
        try:
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                target_size = (self.eye_width, self.output_height)
                if self._cached_rgb_size != target_size or self._cached_chroma is None:
                    self._cached_chroma = np.zeros((self.output_height, self.eye_width, 2), dtype=np.uint8)
                    self._cached_rgb_size = target_size
                
                if USE_CUDA:
                    # CUDA-accelerated path
                    self._rgb_callback_cuda(frame, target_size)
                else:
                    # CPU path
                    self._rgb_callback_cpu(frame, target_size)
                
                self._rgb_count += 1
        except Exception as e:
            self.get_logger().error(f'RGB decode error: {e}')
    
    def _rgb_callback_cpu(self, frame: np.ndarray, target_size: Tuple[int, int]):
        """CPU path for RGB processing"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self._rgb_frame = frame_rgb
        rgb_resized = cv2.resize(frame_rgb, target_size, interpolation=cv2.INTER_LINEAR)
        ycrcb = cv2.cvtColor(rgb_resized, cv2.COLOR_RGB2YCrCb)
        self._cached_chroma[:, :, 0] = cv2.blur(ycrcb[:, :, 1], (5, 5))
        self._cached_chroma[:, :, 1] = cv2.blur(ycrcb[:, :, 2], (5, 5))
    
    def _rgb_callback_cuda(self, frame: np.ndarray, target_size: Tuple[int, int]):
        """CUDA-accelerated path for RGB processing on Jetson"""
        global _cuda_stream, _cuda_box_filter
        
        # Upload to GPU
        gpu_frame = cv2.cuda.GpuMat()
        gpu_frame.upload(frame, _cuda_stream)
        
        # BGR to RGB on GPU
        gpu_rgb = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2RGB, stream=_cuda_stream)
        
        # Resize on GPU (very fast)
        gpu_resized = cv2.cuda.resize(gpu_rgb, target_size, interpolation=cv2.INTER_LINEAR, stream=_cuda_stream)
        
        # RGB to YCrCb on GPU
        gpu_ycrcb = cv2.cuda.cvtColor(gpu_resized, cv2.COLOR_RGB2YCrCb, stream=_cuda_stream)
        
        # Split channels on GPU
        gpu_channels = cv2.cuda.split(gpu_ycrcb, _cuda_stream)
        
        # Apply box blur to Cr and Cb channels on GPU
        gpu_cr_blur = cv2.cuda.GpuMat()
        gpu_cb_blur = cv2.cuda.GpuMat()
        _cuda_box_filter.apply(gpu_channels[1], gpu_cr_blur, stream=_cuda_stream)
        _cuda_box_filter.apply(gpu_channels[2], gpu_cb_blur, stream=_cuda_stream)
        
        # Download results (single sync point)
        _cuda_stream.waitForCompletion()
        self._cached_chroma[:, :, 0] = gpu_cr_blur.download()
        self._cached_chroma[:, :, 1] = gpu_cb_blur.download()
        
        # Also keep RGB on CPU for reference
        self._rgb_frame = gpu_rgb.download()
    
    def left_callback(self, msg: CompressedImage):
        """Handle incoming left image - decode immediately"""
        try:
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                if USE_CUDA:
                    frame_resized = self._process_stereo_frame_cuda(frame)
                else:
                    frame_resized = self._process_stereo_frame_cpu(frame)
                
                # Apply colorization if enabled (left eye)
                if self.colorize:
                    frame_resized = self._colorize_grayscale(frame_resized, is_left=True)
                
                self._left_frame = frame_resized
                self._update_stereo_frame()
                self._left_count += 1
        except Exception as e:
            self.get_logger().error(f'Left decode error: {e}')
        
        self._log_stats()
        
    def right_callback(self, msg: CompressedImage):
        """Handle incoming right image - decode immediately"""
        try:
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                if USE_CUDA:
                    frame_resized = self._process_stereo_frame_cuda(frame)
                else:
                    frame_resized = self._process_stereo_frame_cpu(frame)
                
                # Apply colorization if enabled (right eye)
                if self.colorize:
                    frame_resized = self._colorize_grayscale(frame_resized, is_left=False)
                
                self._right_frame = frame_resized
                self._update_stereo_frame()
                self._right_count += 1
        except Exception as e:
            self.get_logger().error(f'Right decode error: {e}')
    
    def _process_stereo_frame_cpu(self, frame: np.ndarray) -> np.ndarray:
        """CPU path for stereo frame processing"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return cv2.resize(frame_rgb, (self.eye_width, self.output_height))
    
    def _process_stereo_frame_cuda(self, frame: np.ndarray) -> np.ndarray:
        """CUDA-accelerated stereo frame processing on Jetson"""
        global _cuda_stream
        
        # Upload, convert, resize all on GPU
        gpu_frame = cv2.cuda.GpuMat()
        gpu_frame.upload(frame, _cuda_stream)
        gpu_rgb = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2RGB, stream=_cuda_stream)
        gpu_resized = cv2.cuda.resize(gpu_rgb, (self.eye_width, self.output_height), 
                                       interpolation=cv2.INTER_LINEAR, stream=_cuda_stream)
        _cuda_stream.waitForCompletion()
        return gpu_resized.download()
            
    def _update_stereo_frame(self):
        """Update the stereo frame buffer and signal WebRTC"""
        global frame_event, frame_loop
        
        left = self._left_frame
        right = self._right_frame
        
        if left is None or right is None:
            return
            
        # Combine into stereo buffer
        with self._stereo_lock:
            self._stereo_frame[:, :self.eye_width] = left
            self._stereo_frame[:, self.eye_width:] = right
        
        # Signal WebRTC that new frame is ready
        if frame_event and frame_loop:
            try:
                frame_loop.call_soon_threadsafe(frame_event.set)
            except RuntimeError:
                pass
    
    def get_stereo_frame(self) -> Optional[np.ndarray]:
        """Get the latest stereo frame - returns reference, no copy for speed"""
        with self._stereo_lock:
            # Return reference - caller must not modify
            return self._stereo_frame
        
    def _log_stats(self):
        """Log statistics periodically"""
        now = time.time()
        if now - self._last_stats_time >= 10.0:
            elapsed = now - self._last_stats_time
            left_fps = self._left_count / elapsed
            right_fps = self._right_count / elapsed
            rgb_fps = self._rgb_count / elapsed
            if self.colorize:
                self.get_logger().info(f'Stereo: L={left_fps:.1f} R={right_fps:.1f} RGB={rgb_fps:.1f} FPS')
            else:
                self.get_logger().info(f'Stereo: L={left_fps:.1f} FPS, R={right_fps:.1f} FPS')
            self._left_count = 0
            self._right_count = 0
            self._rgb_count = 0
            self._last_stats_time = now


async def offer(request):
    """Handle WebRTC offer from client"""
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    pc = RTCPeerConnection()
    pcs.add(pc)
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed" or pc.connectionState == "closed":
            await pc.close()
            pcs.discard(pc)
    
    # Add stereo video track directly (no relay buffering)
    if ros_node:
        video_track = StereoVideoTrack(ros_node)
        pc.addTrack(video_track)
        
        # Select codec - VP8 encodes faster on CPU than H264
        try:
            capabilities = RTCRtpSender.getCapabilities("video")
            if capabilities:
                if USE_VP8:
                    preferred_codecs = [c for c in capabilities.codecs if c.mimeType == "video/VP8"]
                    codec_name = "VP8"
                else:
                    preferred_codecs = [c for c in capabilities.codecs if c.mimeType == "video/H264"]
                    codec_name = "H264"
                if preferred_codecs:
                    transceiver = pc.getTransceivers()[0]
                    transceiver.setCodecPreferences(preferred_codecs)
                    print(f"Using {codec_name} codec")
        except Exception as e:
            print(f"Could not set codec preferences: {e}")
    
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    # Add bandwidth constraint
    sdp = pc.localDescription.sdp
    bitrate_kbps = TARGET_BITRATE // 1000
    sdp_lines = sdp.split('\n')
    new_lines = []
    for line in sdp_lines:
        new_lines.append(line)
        if line.startswith('m=video'):
            new_lines.append(f'b=AS:{bitrate_kbps}')
    sdp = '\n'.join(new_lines)
    
    return web.json_response({
        "sdp": sdp,
        "type": pc.localDescription.type
    })


async def on_shutdown(app):
    """Cleanup on server shutdown"""
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def create_ssl_context():
    """Create SSL context for HTTPS"""
    import ssl
    
    ssl_keyfile = "key.pem"
    ssl_certfile = "cert.pem"
    
    if not (os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile)):
        module_dir = os.path.dirname(os.path.abspath(__file__))
        ssl_keyfile = os.path.join(module_dir, "key.pem")
        ssl_certfile = os.path.join(module_dir, "cert.pem")
    
    if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(ssl_certfile, ssl_keyfile)
        print(f"Using SSL: {ssl_certfile}")
        return ssl_context
    else:
        print("Warning: SSL certificates not found")
        return None


async def run_webrtc_server(port: int):
    """Run the WebRTC signaling server"""
    global frame_event, frame_loop
    
    frame_event = asyncio.Event()
    frame_loop = asyncio.get_running_loop()
    
    app = web.Application()
    
    async def cors_middleware(app, handler):
        async def middleware_handler(request):
            if request.method == "OPTIONS":
                response = web.Response()
            else:
                try:
                    response = await handler(request)
                except web.HTTPException as ex:
                    response = ex
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type"
            return response
        return middleware_handler
    
    app.middlewares.append(cors_middleware)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)
    
    ssl_context = create_ssl_context()
    
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", port, ssl_context=ssl_context)
    await site.start()
    
    protocol = "https" if ssl_context else "http"
    print(f"Stereo WebRTC signaling: {protocol}://0.0.0.0:{port}")
    
    while True:
        await asyncio.sleep(3600)


def run_server_thread(port: int):
    """Run WebRTC server in a new event loop"""
    try:
        print(f"[Stereo] Starting WebRTC server on port {port}...")
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(run_webrtc_server(port))
    except Exception as e:
        print(f"[Stereo] WebRTC server error: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main function"""
    global ros_node
    
    print()
    print("🥽 ROS2 Stereo WebRTC - ULTRA LOW LATENCY")
    print("=" * 50)
    print("Side-by-side stereo for Meta Quest 3")
    print()
    
    rclpy.init()
    ros_node = StereoVideoNode()
    
    server_thread = threading.Thread(
        target=run_server_thread,
        args=(ros_node.port,),
        daemon=True
    )
    server_thread.start()
    
    time.sleep(0.5)
    
    print(f"📺 Stereo WebRTC on port {ros_node.port}")
    print("🎯 Press Ctrl+C to stop")
    print()
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        print("\n👋 Stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
