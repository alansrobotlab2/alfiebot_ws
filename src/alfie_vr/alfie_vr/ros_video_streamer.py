#!/usr/bin/env python3
"""
ROS2 Video Streamer - Streams compressed images from ROS2 topic via Socket.IO
"""

import asyncio
import base64
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

import socketio
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

VIDEO_PORT = 8081

FRAME_RATE = 10  # Target frame rate for streaming
FRAME_INTERVAL = 1.0 / FRAME_RATE  # Interval between frames in seconds


# Initialize FastAPI and Socket.IO
app = FastAPI()
# Use polling=False to force WebSocket only (no long-polling fallback which adds latency)
sio = socketio.AsyncServer(
    async_mode='asgi', 
    cors_allowed_origins='*',
    ping_timeout=10,
    ping_interval=5,
    max_http_buffer_size=1024*1024  # 1MB max for frames
)
socket_app = socketio.ASGIApp(sio, app)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global reference to the ROS node for accessing latest frame
ros_node: Optional['VideoStreamerNode'] = None

# Video State
is_streaming = False
stream_task = None

# Frame sequence tracking to detect new frames
last_sent_frame_id = 0


async def video_stream_loop():
    """Stream video frames from ROS2 topic to Socket.IO clients"""
    global is_streaming, ros_node, last_sent_frame_id
    print("Starting ROS2 video stream loop...")
    
    if ros_node is None:
        print("Error: ROS node not initialized")
        is_streaming = False
        return

    frames_sent = 0
    frames_skipped = 0
    frames_no_new = 0
    last_stats_time = time.time()
    
    # Debug: Log first few frames
    debug_count = 0

    try:
        while is_streaming:
            loop_start = time.time()
            
            # Get the latest frame from the ROS node
            frame_data, frame_id, frame_timestamp = ros_node.get_latest_frame()
            
            # Debug logging for first few iterations
            if debug_count < 10:
                print(f"Loop iter {debug_count}: frame_data={'present' if frame_data else 'None'}, "
                      f"frame_id={frame_id}, last_sent={last_sent_frame_id}, "
                      f"timestamp={frame_timestamp:.3f}")
                debug_count += 1
            
            if frame_data is not None and frame_id > last_sent_frame_id:
                # Only send if this is a new frame we haven't sent yet
                last_sent_frame_id = frame_id
                
                # Calculate frame age (how old is this frame since we received it from ROS)
                frame_age_ms = (time.time() - frame_timestamp) * 1000
                
                # Skip frames that are too old (more than 500ms old - generous threshold)
                # This only happens during severe backlog situations
                if frame_age_ms > 500:
                    frames_skipped += 1
                    print(f"Skipping stale frame (age: {frame_age_ms:.0f}ms)")
                else:
                    # Send with timestamp so client can track latency
                    await sio.emit('video_frame', {
                        'frame': frame_data,
                        'timestamp': int(frame_timestamp * 1000),  # ms since epoch
                        'frame_id': frame_id
                    })
                    frames_sent += 1
            elif frame_data is None:
                frames_no_new += 1
            
            # Print stats every 5 seconds
            now = time.time()
            if now - last_stats_time >= 5.0:
                effective_fps = frames_sent / (now - last_stats_time)
                print(f"Video stats: sent={frames_sent} ({effective_fps:.1f} fps), skipped={frames_skipped}, no_new={frames_no_new}")
                frames_sent = 0
                frames_skipped = 0
                frames_no_new = 0
                last_stats_time = now
            
            # Control frame rate - account for processing time
            elapsed = time.time() - loop_start
            sleep_time = max(0, FRAME_INTERVAL - elapsed)
            await asyncio.sleep(sleep_time)
            
    except Exception as e:
        print(f"Stream error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Video stream loop stopped.")
        is_streaming = False


@sio.event
async def connect(sid, environ):
    print(f"Client connected: {sid}")


@sio.event
async def disconnect(sid):
    print(f"Client disconnected: {sid}")


@sio.event
async def start_video_stream(sid):
    global is_streaming, stream_task, last_sent_frame_id
    print(f"Client {sid} requested video stream")
    
    if not is_streaming:
        is_streaming = True
        last_sent_frame_id = 0  # Reset frame tracking for new stream
        stream_task = asyncio.create_task(video_stream_loop())


@sio.event
async def stop_video_stream(sid):
    print(f"Client {sid} requested stop video stream")


class VideoStreamerNode(Node):
    """ROS2 node that subscribes to compressed images and makes them available for streaming"""
    
    def __init__(self):
        super().__init__('video_streamer')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/alfie/oak/rgb/image_raw/compressed')
        self.declare_parameter('port', VIDEO_PORT)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        
        # Latest frame storage with metadata
        self._latest_frame: Optional[str] = None
        self._frame_id: int = 0
        self._frame_timestamp: float = 0.0
        self._frame_lock = threading.Lock()
        
        # Frame rate tracking for ROS topic
        self._ros_frame_count = 0
        self._ros_last_stats_time = time.time()
        
        # QoS profile for camera images - use BEST_EFFORT for lowest latency
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the latest frame
        )
        
        # Subscribe to compressed image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info(f'Video Streamer Node initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Streaming on port: {self.port}')
    
    def image_callback(self, msg: CompressedImage):
        """Handle incoming compressed image messages"""
        try:
            receive_time = time.time()
            
            # Convert the compressed image data to base64
            frame_b64 = base64.b64encode(msg.data).decode('utf-8')
            
            with self._frame_lock:
                self._latest_frame = frame_b64
                self._frame_id += 1
                self._frame_timestamp = receive_time
            
            # Track ROS frame rate
            self._ros_frame_count += 1
            now = time.time()
            if now - self._ros_last_stats_time >= 5.0:
                ros_fps = self._ros_frame_count / (now - self._ros_last_stats_time)
                self.get_logger().info(f'ROS topic rate: {ros_fps:.1f} fps')
                self._ros_frame_count = 0
                self._ros_last_stats_time = now
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def get_latest_frame(self) -> tuple[Optional[str], int, float]:
        """Get the latest frame as base64 encoded string with metadata"""
        with self._frame_lock:
            return self._latest_frame, self._frame_id, self._frame_timestamp


def run_uvicorn_server(port: int):
    """Run the uvicorn server in a separate thread"""
    print(f"Starting Video Server on port {port}...")
    print(f"Ensure no other service is using port {port}")
    
    # Check for SSL certs - first try current directory (like video.py), then module directory
    ssl_keyfile = "key.pem"
    ssl_certfile = "cert.pem"
    
    # If not in current directory, try module directory
    if not (os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile)):
        module_dir = os.path.dirname(os.path.abspath(__file__))
        ssl_keyfile = os.path.join(module_dir, "key.pem")
        ssl_certfile = os.path.join(module_dir, "cert.pem")
    
    if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
        print(f"Using SSL certificates: {ssl_certfile}, {ssl_keyfile}")
        uvicorn.run(socket_app, host="0.0.0.0", port=port, 
                    ssl_keyfile=ssl_keyfile, ssl_certfile=ssl_certfile)
    else:
        print("Warning: SSL certificates not found. Running in HTTP mode (might be blocked by browser if main page is HTTPS)")
        uvicorn.run(socket_app, host="0.0.0.0", port=port)


def main():
    """Main function"""
    global ros_node
    
    print("🎥 ROS2 Video Streamer")
    print("=" * 60)
    print("Streams /alfie/oak/rgb/image_raw/compressed via Socket.IO")
    print()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create node
    ros_node = VideoStreamerNode()
    
    # Start uvicorn server in a separate thread
    server_thread = threading.Thread(
        target=run_uvicorn_server, 
        args=(ros_node.port,),
        daemon=True
    )
    server_thread.start()
    
    print(f"📺 Video stream available at port {ros_node.port}")
    print("🎯 Press Ctrl+C to stop")
    print()
    
    try:
        # Spin the node
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        print("\n👋 Video Streamer stopped by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
