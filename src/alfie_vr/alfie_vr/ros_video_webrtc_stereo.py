#!/usr/bin/env python3
"""
ROS2 Stereo Video Streamer via WebRTC - For VR 3D Vision
Subscribes to side-by-side stereo compressed image topic and streams via WebRTC
Optimized for Meta Quest 3 headset display - ULTRA LOW LATENCY VERSION

Key optimizations:
- No queues - only stores latest frame
- Decode directly in ROS callback (fast enough on Jetson)
- Single stereo frame buffer, atomically updated
- No MediaRelay buffering
- Event-driven frame delivery to WebRTC
- CUDA acceleration on Jetson (resize, color convert)
- Receives side-by-side stereo (2560x720) at 10Hz
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
TARGET_FPS = 10  # Match camera input rate of 10Hz
TARGET_BITRATE = 3_000_000  # 3 Mbps - balanced quality/performance

# Use VP8 - better software encoder performance than H264 without hardware accel
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
        self.declare_parameter('stereo_image_topic', '/alfie/stereo_camera/image_raw/compressed')
        self.declare_parameter('port', VIDEO_PORT)
        self.declare_parameter('output_width', 2560)  # Side-by-side: 1280 per eye
        self.declare_parameter('output_height', 720)
        
        # Get parameters
        self.stereo_topic = self.get_parameter('stereo_image_topic').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter('output_height').get_parameter_value().integer_value
        
        self.eye_width = self.output_width // 2
        
        # Pre-allocate stereo buffer
        self._stereo_frame = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)
        self._stereo_lock = threading.Lock()
        
        # Frame counters for stats
        self._frame_count = 0
        self._last_stats_time = time.time()
        self._total_msg_age = 0.0
        self._total_decode_time = 0.0
        self._total_process_time = 0.0
        self._total_callback_time = 0.0
        
        # QoS profile - best effort, keep only latest
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to side-by-side stereo image topic
        self.stereo_sub = self.create_subscription(
            CompressedImage,
            self.stereo_topic,
            self.stereo_callback,
            qos_profile
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Stereo WebRTC Video Streamer - ULTRA LOW LATENCY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Stereo topic: {self.stereo_topic}')
        self.get_logger().info(f'Input: 2560x720 side-by-side @ 10Hz (Motion-JPEG)')
        self.get_logger().info(f'Output: {self.output_width}x{self.output_height}')
        self.get_logger().info(f'WebRTC port: {self.port}')
        # Show acceleration status
        if USE_CUDA:
            self.get_logger().info('🚀 CUDA acceleration: ENABLED (Jetson GPU)')
        else:
            self.get_logger().info('⚠️ CUDA acceleration: DISABLED (CPU only)')
    
    def stereo_callback(self, msg: CompressedImage):
        """Handle incoming side-by-side stereo image (2560x720)"""
        global frame_event, frame_loop
        
        try:
            callback_start = time.time()
            
            # Calculate message age
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            msg_age = callback_start - msg_time
            
            # Decode JPEG
            decode_start = time.time()
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            decode_time = (time.time() - decode_start) * 1000  # ms
            
            if frame is None:
                return
            
            # Verify expected size (2560x720)
            if frame.shape[1] != 2560 or frame.shape[0] != 720:
                self.get_logger().warn(
                    f'Unexpected stereo image size: {frame.shape[1]}x{frame.shape[0]}, expected 2560x720'
                )
            
            process_start = time.time()
            if USE_CUDA:
                self._process_stereo_frame_cuda(frame)
            else:
                self._process_stereo_frame_cpu(frame)
            process_time = (time.time() - process_start) * 1000  # ms
            
            total_time = (time.time() - callback_start) * 1000  # ms
            
            self._frame_count += 1
            self._log_stats(msg_age, decode_time, process_time, total_time)
            
            # Signal WebRTC that new frame is ready
            if frame_event and frame_loop:
                try:
                    frame_loop.call_soon_threadsafe(frame_event.set)
                except RuntimeError:
                    pass
                    
        except Exception as e:
            self.get_logger().error(f'Stereo decode error: {e}')
    
    def _process_stereo_frame_cpu(self, frame: np.ndarray):
        """CPU path: Convert BGR to RGB and copy to stereo buffer"""
        # Convert BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Copy to stereo buffer (already correct size)
        with self._stereo_lock:
            if frame_rgb.shape[0] == self.output_height and frame_rgb.shape[1] == self.output_width:
                # Direct copy if size matches
                np.copyto(self._stereo_frame, frame_rgb)
            else:
                # Resize if needed
                resized = cv2.resize(frame_rgb, (self.output_width, self.output_height), 
                                    interpolation=cv2.INTER_LINEAR)
                np.copyto(self._stereo_frame, resized)
    
    def _process_stereo_frame_cuda(self, frame: np.ndarray):
        """CUDA-accelerated processing on Jetson"""
        global _cuda_stream
        
        # Upload to GPU
        gpu_frame = cv2.cuda.GpuMat()
        gpu_frame.upload(frame, _cuda_stream)
        
        # BGR to RGB on GPU
        gpu_rgb = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2RGB, stream=_cuda_stream)
        
        # Resize if needed
        if frame.shape[0] != self.output_height or frame.shape[1] != self.output_width:
            gpu_rgb = cv2.cuda.resize(gpu_rgb, (self.output_width, self.output_height),
                                     interpolation=cv2.INTER_LINEAR, stream=_cuda_stream)
        
        # Wait and download
        _cuda_stream.waitForCompletion()
        result = gpu_rgb.download()
        
        # Copy to stereo buffer
        with self._stereo_lock:
            np.copyto(self._stereo_frame, result)
    
    def get_stereo_frame(self) -> Optional[np.ndarray]:
        """Get the latest stereo frame - returns reference, no copy for speed"""
        with self._stereo_lock:
            # Return reference - caller must not modify
            return self._stereo_frame
        
    def _log_stats(self, msg_age=0, decode_time=0, process_time=0, total_time=0):
        """Log statistics periodically"""
        self._total_msg_age += msg_age
        self._total_decode_time += decode_time
        self._total_process_time += process_time
        self._total_callback_time += total_time
        
        now = time.time()
        if now - self._last_stats_time >= 5.0:  # Log every 5 seconds
            elapsed = now - self._last_stats_time
            fps = self._frame_count / elapsed
            if self._frame_count > 0:
                avg_msg_age = self._total_msg_age / self._frame_count
                avg_decode = self._total_decode_time / self._frame_count
                avg_process = self._total_process_time / self._frame_count
                avg_total = self._total_callback_time / self._frame_count
                self.get_logger().info(
                    f'Stereo: {fps:.1f} FPS | '
                    f'Latency: msg_age={avg_msg_age*1000:.0f}ms decode={avg_decode:.1f}ms '
                    f'process={avg_process:.1f}ms total={avg_total:.1f}ms'
                )
            self._frame_count = 0
            self._total_msg_age = 0.0
            self._total_decode_time = 0.0
            self._total_process_time = 0.0
            self._total_callback_time = 0.0
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
        
        # Select codec - VP8 for best CPU encoding performance
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
    
    # Add bandwidth and latency optimizations to SDP
    sdp = pc.localDescription.sdp
    bitrate_kbps = TARGET_BITRATE // 1000
    sdp_lines = sdp.split('\n')
    new_lines = []
    for line in sdp_lines:
        new_lines.append(line)
        if line.startswith('m=video'):
            # Bandwidth constraints
            new_lines.append(f'b=AS:{bitrate_kbps}')
            new_lines.append(f'b=TIAS:{TARGET_BITRATE}')
        # Add low-latency attributes for VP8/H264
        if line.startswith('a=rtpmap') and ('VP8' in line or 'H264' in line):
            # Google-specific low latency settings
            new_lines.append(f'a=fmtp:{line.split()[0].split(":")[1]} x-google-min-bitrate={bitrate_kbps};x-google-max-bitrate={bitrate_kbps}')
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
    """Create SSL context for HTTPS with broad mobile browser compatibility"""
    import ssl
    
    ssl_keyfile = "key.pem"
    ssl_certfile = "cert.pem"
    
    if not (os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile)):
        module_dir = os.path.dirname(os.path.abspath(__file__))
        ssl_keyfile = os.path.join(module_dir, "key.pem")
        ssl_certfile = os.path.join(module_dir, "cert.pem")
    
    if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        
        # Set minimum TLS version for security but allow TLS 1.2 for compatibility
        ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
        ssl_context.maximum_version = ssl.TLSVersion.TLSv1_3
        
        # Use a broad set of ciphers for mobile browser compatibility
        # This includes ciphers supported by Android Chrome and Meta Quest browser
        ssl_context.set_ciphers(
            "ECDHE+AESGCM:ECDHE+CHACHA20:DHE+AESGCM:DHE+CHACHA20:"
            "ECDH+AESGCM:DH+AESGCM:ECDH+AES:DH+AES:RSA+AESGCM:RSA+AES:!aNULL:!eNULL:!MD5"
        )
        
        ssl_context.load_cert_chain(ssl_certfile, ssl_keyfile)
        print(f"Using SSL: {ssl_certfile} (TLS 1.2-1.3, mobile-compatible ciphers)")
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
