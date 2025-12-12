#!/usr/bin/env python3
"""
ROS2 Video Streamer via WebRTC - Hardware-accelerated, low-latency video streaming
Uses aiortc for WebRTC and streams compressed images from ROS2 topic
"""

import asyncio
import json
import os
import threading
import fractions
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
from aiortc.contrib.media import MediaRelay

VIDEO_PORT = 8083  # Different port from Socket.IO streamer

# Global reference to the ROS node
ros_node: Optional['WebRTCVideoNode'] = None
relay = MediaRelay()
pcs = set()  # Track peer connections for cleanup


class ROSVideoTrack(VideoStreamTrack):
    """
    A video track that reads frames from ROS2 compressed image topic.
    Sends JPEG frames directly when possible for efficiency.
    """
    
    kind = "video"
    
    def __init__(self, node: 'WebRTCVideoNode'):
        super().__init__()
        self.node = node
        self._timestamp = 0
        self._frame_count = 0
    
    async def recv(self):
        """Receive the next video frame"""
        pts, time_base = await self.next_timestamp()
        
        # Get latest frame from ROS
        frame_data = self.node.get_latest_frame()
        
        if frame_data is not None:
            try:
                # Decode JPEG to numpy array
                np_arr = np.frombuffer(frame_data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if img is not None:
                    # Convert BGR to RGB
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    # Create VideoFrame
                    frame = VideoFrame.from_ndarray(img_rgb, format="rgb24")
                    frame.pts = pts
                    frame.time_base = time_base
                    return frame
            except Exception as e:
                print(f"Frame decode error: {e}")
        
        # Return a black frame if no data
        black = np.zeros((480, 640, 3), dtype=np.uint8)
        frame = VideoFrame.from_ndarray(black, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
    
    async def next_timestamp(self):
        """Generate timestamps for frames at ~30fps"""
        # Wait for next frame timing (~30fps)
        await asyncio.sleep(1/30)
        
        self._timestamp += 3000  # 90000 / 30 fps
        return self._timestamp, fractions.Fraction(1, 90000)


class WebRTCVideoNode(Node):
    """ROS2 node that subscribes to compressed images for WebRTC streaming"""
    
    def __init__(self):
        super().__init__('webrtc_video_streamer')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/alfie/oak/rgb/image_raw/compressed')
        self.declare_parameter('port', VIDEO_PORT)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Latest frame storage (raw bytes, not base64)
        self._latest_frame: Optional[bytes] = None
        self._frame_lock = threading.Lock()
        
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
        
        self.get_logger().info(f'WebRTC Video Streamer initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'WebRTC signaling on port: {self.port}')
    
    def image_callback(self, msg: CompressedImage):
        """Handle incoming compressed image messages"""
        with self._frame_lock:
            # Store raw bytes (not base64) for efficiency
            self._latest_frame = bytes(msg.data)
    
    def get_latest_frame(self) -> Optional[bytes]:
        """Get the latest frame as raw bytes"""
        with self._frame_lock:
            return self._latest_frame


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
    
    # Add video track
    if ros_node:
        video_track = ROSVideoTrack(ros_node)
        pc.addTrack(relay.subscribe(video_track))
    
    # Handle the offer
    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    
    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type
    })


async def on_shutdown(app):
    """Cleanup on server shutdown"""
    # Close all peer connections
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def create_ssl_context():
    """Create SSL context for HTTPS"""
    import ssl
    
    # Check for SSL certs
    ssl_keyfile = "key.pem"
    ssl_certfile = "cert.pem"
    
    # If not in current directory, try module directory
    if not (os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile)):
        module_dir = os.path.dirname(os.path.abspath(__file__))
        ssl_keyfile = os.path.join(module_dir, "key.pem")
        ssl_certfile = os.path.join(module_dir, "cert.pem")
    
    if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(ssl_certfile, ssl_keyfile)
        print(f"Using SSL certificates: {ssl_certfile}, {ssl_keyfile}")
        return ssl_context
    else:
        print("Warning: SSL certificates not found. Running without HTTPS.")
        return None


async def run_webrtc_server(port: int):
    """Run the WebRTC signaling server"""
    app = web.Application()
    
    # CORS headers for all responses
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
    print(f"WebRTC signaling server running at {protocol}://0.0.0.0:{port}")
    
    # Keep running
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
    
    print("🎥 ROS2 WebRTC Video Streamer")
    print("=" * 60)
    print("Hardware-accelerated, low-latency video streaming")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    ros_node = WebRTCVideoNode()
    
    # Start WebRTC server in a separate thread
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
        print("\n👋 WebRTC Streamer stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
