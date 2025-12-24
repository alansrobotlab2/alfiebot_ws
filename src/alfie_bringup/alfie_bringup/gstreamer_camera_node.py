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

import cv2


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 15)  # Reduced from 30 to save CPU
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('camera_frame_id', 'stereo_camera_link')
        self.declare_parameter('use_hardware_accel', True)  # Use Jetson hardware encoder/decoder
        self.declare_parameter('jpeg_quality', 85)  # JPEG quality (0-100, lower = smaller files, faster)
        
        # Get parameters
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_vertical = self.get_parameter('flip_vertical').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.use_hardware_accel = self.get_parameter('use_hardware_accel').get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # Publishers - separate left and right feeds
        self.left_image_pub = self.create_publisher(
            CompressedImage,
            'left/image_raw/compressed',
            1  # Queue size 1 for minimal latency
        )
        
        self.right_image_pub = self.create_publisher(
            CompressedImage,
            'right/image_raw/compressed',
            1  # Queue size 1 for minimal latency
        )
        
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            'left/camera_info',
            1
        )
        
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            'right/camera_info',
            1
        )
        
        # Pre-compute camera info (half-width for each eye)
        self._half_width = None  # Will be set after pipeline starts
        
        # Frame counter
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()
        
        # GLib main loop and bus (for cleanup)
        self.main_loop = None
        self.bus = None
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Check for hardware acceleration support
        self.get_logger().info(f'Hardware acceleration requested: {self.use_hardware_accel}')
        self.hardware_available = self._check_hardware_support()
        if self.use_hardware_accel and not self.hardware_available:
            self.get_logger().warning('⚠️  Hardware acceleration requested but not available, falling back to CPU')
            self.use_hardware_accel = False
        elif self.use_hardware_accel and self.hardware_available:
            self.get_logger().info('🚀 Using hardware acceleration')
        
        # Build GStreamer pipeline
        self.pipeline = self._create_pipeline()
        
        # Start pipeline in separate thread
        self.pipeline_thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self.pipeline_thread.start()
        
        half_width = self.width // 2
        self.get_logger().info(f'GStreamer stereo camera started: {self.device} @ {self.width}x{self.height} {self.framerate}fps')
        self.get_logger().info(f'Publishing left/right feeds at {half_width}x{self.height} each')
        if self.flip_vertical:
            self.get_logger().info('Vertical flip enabled')
    
    def _create_camera_info(self, header, frame_id):
        """Create a valid CameraInfo message with proper intrinsic parameters"""
        half_width = self.width // 2
        height = self.height
        
        info = CameraInfo()
        info.header = header
        info.header.frame_id = frame_id
        info.width = half_width
        info.height = height
        
        # Approximate focal length (can be calibrated later)
        # For a typical webcam, fx ~= width is a reasonable starting point
        fx = float(half_width)
        fy = float(half_width)
        cx = float(half_width) / 2.0
        cy = float(height) / 2.0
        
        # K matrix (3x3 intrinsic camera matrix)
        # [fx  0  cx]
        # [0  fy  cy]
        # [0   0   1]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        
        # Distortion coefficients (assume no distortion for now)
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (identity for monocular)
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        
        # Projection matrix (3x4)
        # [fx  0  cx  Tx]
        # [0  fy  cy   0]
        # [0   0   1   0]
        # Tx = 0 for left camera, Tx = -fx * baseline for right camera
        # We'll use Tx=0 for both since we don't have calibration
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        return info
    
    def _check_hardware_support(self):
        """Check if Nvidia hardware acceleration plugins are available"""
        required_elements = ['nvv4l2decoder', 'nvvidconv', 'nvjpegenc']
        
        for elem_name in required_elements:
            elem = Gst.ElementFactory.find(elem_name)
            if elem is None:
                self.get_logger().warning(f'Missing GStreamer element: {elem_name}')
                return False
        
        self.get_logger().info('✅ Hardware acceleration elements available')
        return True
    
    def _create_pipeline(self):
        """Create GStreamer pipeline for low-latency MJPEG capture"""
        
        if self.use_hardware_accel:
            return self._create_hardware_pipeline()
        else:
            return self._create_software_pipeline()
    
    def _create_software_pipeline(self):
        """Create CPU-based pipeline (fallback)"""
        # Pipeline elements:
        # v4l2src: Capture from camera with minimal buffering
        # image/jpeg: Force MJPEG format (no decode!)
        # appsink: Pull frames into our application
        
        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        
        self.get_logger().info(f'🖥️  CPU Pipeline: {pipeline_str}')
        
        pipeline = Gst.parse_launch(pipeline_str)
        appsink = pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample_software)
        
        return pipeline
    
    def _create_hardware_pipeline(self):
        """Create hardware-accelerated pipeline using Jetson NVDEC with GPU-based stereo split"""
        # Full hardware pipeline: decode → tee → crop left/right → flip → encode (all on GPU!)
        # Uses nvvidconv left/right/top/bottom properties or videocrop for cropping
        
        half_width = self.width // 2
        flip_method = 2 if self.flip_vertical else 0  # 2 = vertical flip, 0 = no flip
        
        # Try nvvidconv with left/right crop properties (Jetson L4T style)
        # Left image: crop right half away (right=half_width)
        # Right image: crop left half away (left=half_width)
        
        # Note: Camera left/right are swapped in the raw image
        # Left eye is in right half of image, right eye is in left half
        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
            f'nvv4l2decoder mjpeg=1 ! '
            f'tee name=t '
            # Left branch: left eye is in right half of image, flip if needed, encode
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left={half_width} right={self.width} top=0 bottom={self.height} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={half_width},height={self.height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=left_sink emit-signals=true max-buffers=1 drop=true sync=false '
            # Right branch: right eye is in left half of image, flip if needed, encode
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left=0 right={half_width} top=0 bottom={self.height} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={half_width},height={self.height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=right_sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        
        flip_str = "Flip→" if self.flip_vertical else ""
        self.get_logger().info(f'🚀 Hardware Pipeline (Full GPU: Decode→Split→{flip_str}Encode)')
        self.get_logger().debug(f'Pipeline: {pipeline_str}')
        
        try:
            pipeline = Gst.parse_launch(pipeline_str)
            
            # Connect left sink
            left_sink = pipeline.get_by_name('left_sink')
            left_sink.connect('new-sample', self._on_new_sample_hardware_left)
            
            # Connect right sink
            right_sink = pipeline.get_by_name('right_sink')
            right_sink.connect('new-sample', self._on_new_sample_hardware_right)
            
            return pipeline
            
        except Exception as e:
            self.get_logger().error(f'Failed to create hardware pipeline: {e}')
            self.get_logger().warning('Falling back to CPU pipeline')
            self.use_hardware_accel = False
            return self._create_software_pipeline()
    
    def _on_new_sample_software(self, sink):
        """Callback for CPU pipeline - decode, split, and process left/right"""
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
        
        left_jpeg = None
        right_jpeg = None
        try:
            # Decode JPEG to split
            nparr = np.frombuffer(map_info.data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if img is None:
                return Gst.FlowReturn.OK
            
            # Handle vertical flip if needed
            if self.flip_vertical:
                img = cv2.flip(img, 0)  # Vertical flip
            
            # Split into left and right halves
            # Note: Camera left/right are swapped in the raw image
            height, width = img.shape[:2]
            half_width = width // 2
            left_img = img[:, half_width:]   # Left eye is in right half of image
            right_img = img[:, :half_width]  # Right eye is in left half of image
            
            # Encode to JPEG
            _, left_encoded = cv2.imencode('.jpg', left_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            _, right_encoded = cv2.imencode('.jpg', right_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            left_jpeg = left_encoded.tobytes()
            right_jpeg = right_encoded.tobytes()
            
            # Cleanup
            del nparr, img, left_img, right_img, left_encoded, right_encoded
        finally:
            buf.unmap(map_info)
        
        if left_jpeg is None or right_jpeg is None:
            return Gst.FlowReturn.OK
        
        # Create header
        header = Header()
        header.stamp = receive_time.to_msg()
        
        # Publish left image
        left_msg = CompressedImage()
        left_msg.header = header
        left_msg.header.frame_id = 'left_camera_link'
        left_msg.format = 'jpeg'
        left_msg.data = left_jpeg
        self.left_image_pub.publish(left_msg)
        
        # Publish right image
        right_msg = CompressedImage()
        right_msg.header = header
        right_msg.header.frame_id = 'right_camera_link'
        right_msg.format = 'jpeg'
        right_msg.data = right_jpeg
        self.right_image_pub.publish(right_msg)
        
        # Publish camera info for both
        left_info = self._create_camera_info(left_msg.header, 'left_camera_link')
        self.left_info_pub.publish(left_info)
        
        right_info = self._create_camera_info(right_msg.header, 'right_camera_link')
        self.right_info_pub.publish(right_info)
        
        return Gst.FlowReturn.OK
    
    def _on_new_sample_hardware_left(self, sink):
        """Callback for hardware pipeline LEFT image - already cropped on GPU"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        receive_time = self.get_clock().now()
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        # Copy JPEG data (already cropped and encoded on GPU)
        jpeg_data = bytes(map_info.data)
        buf.unmap(map_info)
        
        # Create and publish left image
        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = receive_time.to_msg()
        msg.header.frame_id = 'left_camera_link'
        msg.format = 'jpeg'
        msg.data = jpeg_data
        self.left_image_pub.publish(msg)
        
        # Publish camera info
        info_msg = self._create_camera_info(msg.header, 'left_camera_link')
        self.left_info_pub.publish(info_msg)
        
        return Gst.FlowReturn.OK
    
    def _on_new_sample_hardware_right(self, sink):
        """Callback for hardware pipeline RIGHT image - already cropped on GPU"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        receive_time = self.get_clock().now()
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        # Copy JPEG data (already cropped and encoded on GPU)
        jpeg_data = bytes(map_info.data)
        buf.unmap(map_info)
        
        # Create and publish right image
        msg = CompressedImage()
        msg.header = Header()
        msg.header.stamp = receive_time.to_msg()
        msg.header.frame_id = 'right_camera_link'
        msg.format = 'jpeg'
        msg.data = jpeg_data
        self.right_image_pub.publish(msg)
        
        # Publish camera info
        info_msg = self._create_camera_info(msg.header, 'right_camera_link')
        self.right_info_pub.publish(info_msg)
        
        return Gst.FlowReturn.OK
    
    def _run_pipeline(self):
        """Run GStreamer pipeline in separate thread"""
        # Set to playing
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Create main loop (store as instance for cleanup)
        self.main_loop = GLib.MainLoop()
        
        # Handle bus messages
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message', self._on_bus_message, self.main_loop)
        
        # Run loop
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup bus watch
            if self.bus:
                self.bus.remove_signal_watch()
            # Set pipeline to NULL
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
        # Quit main loop first to stop the pipeline thread
        if self.main_loop and self.main_loop.is_running():
            self.main_loop.quit()
        
        # Remove bus signal watch
        if self.bus:
            try:
                self.bus.remove_signal_watch()
            except Exception:
                pass
            self.bus = None
        
        # Stop pipeline
        if hasattr(self, 'pipeline') and self.pipeline:
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
