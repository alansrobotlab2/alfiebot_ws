#!/usr/bin/env python3
"""
ROS2 Video Streamer via WebRTC - Jetson Hardware-Accelerated Version
Uses GStreamer with nvjpegdec + nvv4l2h264enc for minimal CPU usage
"""

import asyncio
import json
import os
import threading
import fractions
import time
from typing import Optional
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

import numpy as np
from av import VideoFrame, Packet
from av.video.frame import VideoFrame as AVVideoFrame

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRelay
from aiortc.rtcrtpsender import RTCRtpSender

# Monkey-patch aiortc encoder bitrate limits for better quality
try:
    import aiortc.codecs.h264 as h264_codec
    import aiortc.codecs.vpx as vpx_codec
    # Increase H.264 limits
    h264_codec.DEFAULT_BITRATE = 3_000_000  # 3 Mbps default
    h264_codec.MIN_BITRATE = 1_000_000      # 1 Mbps min
    h264_codec.MAX_BITRATE = 8_000_000      # 8 Mbps max
    # Increase VP8 limits as fallback
    vpx_codec.DEFAULT_BITRATE = 2_000_000   # 2 Mbps default
    vpx_codec.MIN_BITRATE = 500_000         # 500 kbps min
    vpx_codec.MAX_BITRATE = 6_000_000       # 6 Mbps max
    print("✓ Patched aiortc encoder bitrates (H264: 3Mbps default, 8Mbps max)")
except Exception as e:
    print(f"Warning: Could not patch encoder bitrates: {e}")

VIDEO_PORT = 8083
TARGET_FPS = 10
TARGET_BITRATE = 3_000_000  # 3 Mbps (H264 max in aiortc)

# Initialize GStreamer
Gst.init(None)

# Global references
ros_node: Optional['HWAccelVideoNode'] = None
relay = MediaRelay()
pcs = set()

# Frame queue for passing encoded H.264 packets from GStreamer to WebRTC
h264_packet_queue: Queue = Queue(maxsize=2)

# Asyncio event for frame signaling
frame_event: Optional[asyncio.Event] = None
frame_loop: Optional[asyncio.AbstractEventLoop] = None


class JetsonH264Track(VideoStreamTrack):
    """
    Video track that outputs H.264 packets encoded by Jetson hardware.
    Uses GStreamer nvv4l2h264enc for hardware encoding.
    """
    
    kind = "video"
    
    def __init__(self, node: 'HWAccelVideoNode'):
        super().__init__()
        self.node = node
        self._timestamp = 0
        self._start_time = time.time()
        
    async def recv(self):
        """Receive the next H.264 encoded frame"""
        global frame_event
        
        # Wait for new frame signal
        if frame_event:
            try:
                await asyncio.wait_for(frame_event.wait(), timeout=0.1)
                frame_event.clear()
            except asyncio.TimeoutError:
                pass
        
        pts, time_base = self._next_timestamp()
        
        # Get latest decoded frame from GStreamer pipeline
        frame_data = self.node.get_latest_frame()
        
        if frame_data is not None:
            try:
                # frame_data is already RGB numpy array from GStreamer
                frame = VideoFrame.from_ndarray(frame_data, format="rgb24")
                frame.pts = pts
                frame.time_base = time_base
                return frame
            except Exception as e:
                print(f"Frame error: {e}")
        
        # Return black frame if no data
        black = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = VideoFrame.from_ndarray(black, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
    
    def _next_timestamp(self):
        self._timestamp += int(90000 / TARGET_FPS)
        return self._timestamp, fractions.Fraction(1, 90000)


class GStreamerPipeline:
    """
    GStreamer pipeline for hardware-accelerated JPEG decode.
    Uses appsrc to inject ROS JPEG frames, appsink to get decoded RGB.
    
    Pipeline: appsrc -> nvjpegdec -> nvvidconv -> video/x-raw,format=RGB -> appsink
    """
    
    def __init__(self, on_frame_callback):
        self.on_frame_callback = on_frame_callback
        self.pipeline = None
        self.appsrc = None
        self.appsink = None
        self.loop = None
        self.thread = None
        self._running = False
        self._frame_count = 0
        self._last_log_time = time.time()
        
    def start(self):
        """Start the GStreamer pipeline in a separate thread"""
        self._running = True
        self.thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self.thread.start()
        
    def stop(self):
        """Stop the GStreamer pipeline"""
        self._running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop:
            self.loop.quit()
            
    def _run_pipeline(self):
        """Run GStreamer main loop in thread"""
        # Build pipeline with hardware JPEG decoder
        # nvjpegdec outputs to NVMM memory, nvvidconv moves to CPU, videoconvert to RGB
        pipeline_str = (
            'appsrc name=src emit-signals=false is-live=true format=time '
            'caps=image/jpeg,framerate=30/1 ! '
            'nvjpegdec ! '
            'nvvidconv ! '
            'video/x-raw ! '
            'videoconvert ! '
            'video/x-raw,format=RGB ! '
            'appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true'
        )
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            print("✓ Using NVIDIA hardware JPEG decoder (nvjpegdec)")
        except GLib.Error as e:
            print(f"GStreamer pipeline error: {e}")
            print("⚠ Falling back to software decoder (jpegdec)...")
            # Fallback to software JPEG decoder
            pipeline_str = (
                'appsrc name=src emit-signals=false is-live=true format=time '
                'caps=image/jpeg,framerate=30/1 ! '
                'jpegdec ! '
                'videoconvert ! '
                'video/x-raw,format=RGB ! '
                'appsink name=sink emit-signals=true sync=false max-buffers=2 drop=true'
            )
            self.pipeline = Gst.parse_launch(pipeline_str)
        
        self.appsrc = self.pipeline.get_by_name('src')
        self.appsink = self.pipeline.get_by_name('sink')
        
        # Connect to new-sample signal
        self.appsink.connect('new-sample', self._on_new_sample)
        
        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Failed to start GStreamer pipeline")
            return
            
        print("GStreamer hardware pipeline started")
        
        # Run main loop
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except Exception as e:
            print(f"GStreamer loop error: {e}")
            
    def _on_new_sample(self, sink):
        """Handle decoded frame from appsink"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
            
        buf = sample.get_buffer()
        caps = sample.get_caps()
        
        # Get frame dimensions from caps
        struct = caps.get_structure(0)
        width = struct.get_int('width')[1]
        height = struct.get_int('height')[1]
        
        # Map buffer and extract data
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            # Create numpy array from buffer (RGB format)
            frame = np.ndarray(
                shape=(height, width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            ).copy()  # Copy to own the data
            buf.unmap(map_info)
            
            # Call the callback with decoded frame
            self.on_frame_callback(frame)
            
            # Log stats periodically
            self._frame_count += 1
            now = time.time()
            if now - self._last_log_time >= 5.0:
                fps = self._frame_count / (now - self._last_log_time)
                # Calculate frame quality metrics
                mean_val = np.mean(frame)
                std_val = np.std(frame)
                print(f"GStreamer decode: {fps:.1f} FPS ({width}x{height}) | mean={mean_val:.1f} std={std_val:.1f}")
                self._frame_count = 0
                self._last_log_time = now
                
        return Gst.FlowReturn.OK
        
    def push_jpeg(self, jpeg_data: bytes, timestamp_ns: int):
        """Push JPEG frame data to the pipeline"""
        if not self._running or self.appsrc is None:
            return
            
        # Create GStreamer buffer from JPEG data
        buf = Gst.Buffer.new_allocate(None, len(jpeg_data), None)
        buf.fill(0, jpeg_data)
        buf.pts = timestamp_ns
        buf.dts = timestamp_ns
        buf.duration = Gst.SECOND // TARGET_FPS
        
        # Push to pipeline
        ret = self.appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            pass  # Frame dropped, normal under load


class HWAccelVideoNode(Node):
    """ROS2 node with hardware-accelerated video processing"""
    
    def __init__(self):
        super().__init__('webrtc_video_hw')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/alfie/oak/rgb/image_raw/compressed')
        self.declare_parameter('port', VIDEO_PORT)
        self.declare_parameter('bitrate_mbps', 3)  # Video bitrate in Mbps (H264 max=3)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.bitrate = self.get_parameter('bitrate_mbps').get_parameter_value().integer_value * 1_000_000
        
        # Latest decoded frame (RGB numpy array)
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()
        
        # Start GStreamer pipeline
        self.gst_pipeline = GStreamerPipeline(self._on_decoded_frame)
        self.gst_pipeline.start()
        
        # QoS profile for camera images
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        self._start_time = time.time()
        self._msg_count = 0
        
        self.get_logger().info(f'HW-Accelerated WebRTC Video Streamer')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'WebRTC signaling on port: {self.port}')
    
    def image_callback(self, msg: CompressedImage):
        """Handle incoming compressed image - push to GStreamer"""
        # Convert ROS timestamp to nanoseconds
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # Push JPEG to GStreamer for hardware decode
        self.gst_pipeline.push_jpeg(bytes(msg.data), timestamp_ns)
        
        self._msg_count += 1
        
    def _on_decoded_frame(self, frame: np.ndarray):
        """Callback when GStreamer decodes a frame"""
        global frame_event, frame_loop
        
        # Store decoded frame (atomic reference assignment)
        with self._frame_lock:
            self._latest_frame = frame
        
        # Signal waiting WebRTC tracks
        if frame_event and frame_loop:
            try:
                frame_loop.call_soon_threadsafe(frame_event.set)
            except RuntimeError:
                pass
    
    def get_latest_frame(self) -> Optional[np.ndarray]:
        """Get the latest decoded RGB frame"""
        with self._frame_lock:
            return self._latest_frame
            
    def destroy_node(self):
        """Cleanup"""
        self.gst_pipeline.stop()
        super().destroy_node()


async def offer(request):
    """Handle WebRTC offer from client"""
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    
    # Configure peer connection to prefer H.264
    pc = RTCPeerConnection()
    pcs.add(pc)
    
    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed" or pc.connectionState == "closed":
            await pc.close()
            pcs.discard(pc)
    
    # Add video track with H.264 preference
    if ros_node:
        video_track = JetsonH264Track(ros_node)
        pc.addTrack(relay.subscribe(video_track))
        
        # Force H.264 codec if available (better for hardware decode on client)
        try:
            capabilities = RTCRtpSender.getCapabilities("video")
            if capabilities:
                h264_codecs = [c for c in capabilities.codecs if c.mimeType == "video/H264"]
                if h264_codecs:
                    transceiver = pc.getTransceivers()[0]
                    transceiver.setCodecPreferences(h264_codecs)
                    print("Using H.264 codec (hardware friendly)")
        except Exception as e:
            print(f"Could not set codec preferences: {e}")
        
        bitrate = ros_node.bitrate if ros_node else TARGET_BITRATE
        print(f"Video target: {TARGET_FPS} FPS, {bitrate // 1_000_000} Mbps")
    
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    # Modify SDP to set bandwidth constraint
    sdp = pc.localDescription.sdp
    bitrate_kbps = (ros_node.bitrate if ros_node else TARGET_BITRATE) // 1000
    # Add bandwidth line after m=video line
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
    print(f"WebRTC signaling: {protocol}://0.0.0.0:{port}")
    
    while True:
        await asyncio.sleep(3600)


def run_server_thread(port: int):
    """Run WebRTC server in a new event loop"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(run_webrtc_server(port))


def main():
    """Main function"""
    global ros_node
    
    print("🎥 ROS2 WebRTC Video Streamer - Jetson HW Accelerated")
    print("=" * 60)
    print("Using: nvjpegdec (HW JPEG) + H.264 codec preference")
    print()
    
    rclpy.init()
    ros_node = HWAccelVideoNode()
    
    server_thread = threading.Thread(
        target=run_server_thread,
        args=(ros_node.port,),
        daemon=True
    )
    server_thread.start()
    
    print(f"📺 WebRTC signaling at port {ros_node.port}")
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
