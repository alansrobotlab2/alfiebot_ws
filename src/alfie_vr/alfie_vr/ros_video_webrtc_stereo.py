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
"""

import asyncio
import json
import os
import threading
import fractions
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from av import VideoFrame

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
        self.declare_parameter('port', VIDEO_PORT)
        self.declare_parameter('output_width', 2560)  # 1280 per eye
        self.declare_parameter('output_height', 720)
        
        # Get parameters
        self.left_topic = self.get_parameter('left_image_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_image_topic').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter('output_height').get_parameter_value().integer_value
        
        self.eye_width = self.output_width // 2
        
        # Pre-allocate stereo buffer
        self._stereo_frame = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        self._stereo_lock = threading.Lock()
        
        # Latest decoded frames
        self._left_frame: Optional[np.ndarray] = None
        self._right_frame: Optional[np.ndarray] = None
        
        # Frame counters for stats
        self._left_count = 0
        self._right_count = 0
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
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Stereo WebRTC Video Streamer - ULTRA LOW LATENCY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Left eye:  {self.left_topic}')
        self.get_logger().info(f'Right eye: {self.right_topic}')
        self.get_logger().info(f'Output: {self.output_width}x{self.output_height}')
        self.get_logger().info(f'WebRTC port: {self.port}')
        
    def left_callback(self, msg: CompressedImage):
        """Handle incoming left image - decode immediately"""
        try:
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                # Convert BGR to RGB and resize
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb, (self.eye_width, self.output_height))
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
                # Convert BGR to RGB and resize
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb, (self.eye_width, self.output_height))
                self._right_frame = frame_resized
                self._update_stereo_frame()
                self._right_count += 1
        except Exception as e:
            self.get_logger().error(f'Right decode error: {e}')
            
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
            self.get_logger().info(f'Stereo: L={left_fps:.1f} FPS, R={right_fps:.1f} FPS')
            self._left_count = 0
            self._right_count = 0
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
