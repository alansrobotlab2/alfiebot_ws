#!/usr/bin/env python3
"""
ROS2 Video Streamer - Streams compressed images from ROS2 topic via Socket.IO
"""

import asyncio
import base64
import os
import threading
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


# Initialize FastAPI and Socket.IO
app = FastAPI()
sio = socketio.AsyncServer(async_mode='asgi', cors_allowed_origins='*')
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


async def video_stream_loop():
    """Stream video frames from ROS2 topic to Socket.IO clients"""
    global is_streaming, ros_node
    print("Starting ROS2 video stream loop...")
    
    if ros_node is None:
        print("Error: ROS node not initialized")
        is_streaming = False
        return

    try:
        while is_streaming:
            # Get the latest frame from the ROS node
            frame_data = ros_node.get_latest_frame()
            
            if frame_data is not None:
                # Frame is already base64 encoded
                await sio.emit('video_frame', {'frame': frame_data})
            
            # Control frame rate (~30 fps max)
            await asyncio.sleep(0.033)
            
    except Exception as e:
        print(f"Stream error: {e}")
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
    global is_streaming, stream_task
    print(f"Client {sid} requested video stream")
    
    if not is_streaming:
        is_streaming = True
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
        
        # Latest frame storage
        self._latest_frame: Optional[str] = None
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
        
        self.get_logger().info(f'Video Streamer Node initialized')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Streaming on port: {self.port}')
    
    def image_callback(self, msg: CompressedImage):
        """Handle incoming compressed image messages"""
        try:
            # Convert the compressed image data to base64
            frame_b64 = base64.b64encode(msg.data).decode('utf-8')
            
            with self._frame_lock:
                self._latest_frame = frame_b64
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def get_latest_frame(self) -> Optional[str]:
        """Get the latest frame as base64 encoded string"""
        with self._frame_lock:
            return self._latest_frame


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
