#!/usr/bin/env python3
"""
GStreamer-based low-latency camera node for ROS2
Uses GStreamer pipeline to capture MJPEG and publish directly to compressed topic
Much lower latency than usb_cam
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import threading


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('camera_frame_id', 'stereo_camera_link')
        
        # Get parameters
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_vertical = self.get_parameter('flip_vertical').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        
        # Publishers
        self.image_pub = self.create_publisher(
            CompressedImage,
            'image_raw/compressed',
            1  # Queue size 1 for minimal latency
        )
        
        self.info_pub = self.create_publisher(
            CameraInfo,
            'camera_info',
            1
        )
        
        # Frame counter
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Build GStreamer pipeline
        self.pipeline = self._create_pipeline()
        
        # Start pipeline in separate thread
        self.pipeline_thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self.pipeline_thread.start()
        
        self.get_logger().info(f'GStreamer camera started: {self.device} @ {self.width}x{self.height} {self.framerate}fps')
        if self.flip_vertical:
            self.get_logger().info('Vertical flip enabled')
    
    def _create_pipeline(self):
        """Create GStreamer pipeline for low-latency MJPEG capture"""
        
        # Pipeline elements:
        # v4l2src: Capture from camera with minimal buffering
        # image/jpeg: Force MJPEG format (no decode!)
        # appsink: Pull frames into our application
        
        # Note: Can't flip JPEG directly, would need to decode/re-encode
        # So we skip flip in pipeline and do it in OpenCV if needed
        
        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        
        self.get_logger().info(f'Pipeline: {pipeline_str}')
        
        pipeline = Gst.parse_launch(pipeline_str)
        
        # Get appsink and connect callback
        appsink = pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample)
        
        return pipeline
    
    def _on_new_sample(self, sink):
        """Callback when new frame is available"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        # Get buffer
        buf = sample.get_buffer()
        
        # Get current time FIRST for accurate latency measurement
        receive_time = self.get_clock().now()
        
        # Extract JPEG data
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        try:
            # Get JPEG bytes
            jpeg_data = map_info.data
            
            # Handle vertical flip if needed (decode, flip, re-encode)
            if self.flip_vertical:
                import cv2
                nparr = np.frombuffer(jpeg_data, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is not None:
                    img = cv2.flip(img, 0)  # Vertical flip
                    _, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    jpeg_data = encoded.tobytes()
            
            # Create ROS message - use receive_time (NOW) for freshest timestamp
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = receive_time.to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = bytes(jpeg_data)
            
            # Publish
            self.image_pub.publish(msg)
            
            # Publish camera info (basic)
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            self.info_pub.publish(info_msg)
            
            # Stats
            self.frame_count += 1
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed >= 5.0:
                fps = self.frame_count / elapsed
                self.get_logger().info(f'Publishing at {fps:.1f} FPS')
                self.frame_count = 0
                self.last_log_time = now
            
        finally:
            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def _run_pipeline(self):
        """Run GStreamer pipeline in separate thread"""
        # Set to playing
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Create main loop
        loop = GLib.MainLoop()
        
        # Handle bus messages
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message, loop)
        
        # Run loop
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        
        # Cleanup
        self.pipeline.set_state(Gst.State.NULL)
    
    def _on_bus_message(self, bus, message, loop):
        """Handle GStreamer bus messages"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f'GStreamer error: {err}, {debug}')
            loop.quit()
        elif t == Gst.MessageType.EOS:
            self.get_logger().info('End of stream')
            loop.quit()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.get_logger().warn(f'GStreamer warning: {err}')
        
        return True
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
