#!/usr/bin/env python3
"""
GStreamer-based low-latency camera node for ROS2
Uses GStreamer pipeline to capture MJPEG and publish directly to compressed topic
Much lower latency than usb_cam
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
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
        self.declare_parameter('source_width', 3200)  # Capture at max resolution
        self.declare_parameter('source_height', 1200)  # Capture at max resolution
        self.declare_parameter('output_width', 640)  # Output resolution width
        self.declare_parameter('output_height', 480)  # Output resolution height
        self.declare_parameter('framerate', 15)  # Full framerate for dual-resolution
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('use_hardware_accel', True)  # Use Jetson hardware encoder/decoder
        self.declare_parameter('jpeg_quality', 70)  # JPEG quality (0-100, lower = smaller files, faster)
        self.declare_parameter('camera_frame_id', 'stereo_camera_link')

        # Get parameters
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.source_width = self.get_parameter('source_width').get_parameter_value().integer_value
        self.source_height = self.get_parameter('source_height').get_parameter_value().integer_value
        self.output_width = self.get_parameter('output_width').get_parameter_value().integer_value
        self.output_height = self.get_parameter('output_height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_vertical = self.get_parameter('flip_vertical').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.use_hardware_accel = self.get_parameter('use_hardware_accel').get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # Publishers - dual resolution: wide angle and center crop for each eye
        # All topics published under stereo_camera namespace for clear organization
        # Wide angle publishers (full FOV downsampled to output)
        self.left_wide_image_pub = self.create_publisher(
            CompressedImage,
            'stereo_camera/left_wide/image_raw/compressed',
            1  # Queue size 1 for minimal latency
        )

        self.right_wide_image_pub = self.create_publisher(
            CompressedImage,
            'stereo_camera/right_wide/image_raw/compressed',
            1
        )

        self.left_wide_info_pub = self.create_publisher(
            CameraInfo,
            'stereo_camera/left_wide/camera_info',
            1
        )

        self.right_wide_info_pub = self.create_publisher(
            CameraInfo,
            'stereo_camera/right_wide/camera_info',
            1
        )

        # Center crop publishers (center output_width x output_height region)
        self.left_center_image_pub = self.create_publisher(
            CompressedImage,
            'stereo_camera/left_center/image_raw/compressed',
            1
        )

        self.right_center_image_pub = self.create_publisher(
            CompressedImage,
            'stereo_camera/right_center/image_raw/compressed',
            1
        )

        self.left_center_info_pub = self.create_publisher(
            CameraInfo,
            'stereo_camera/left_center/camera_info',
            1
        )

        self.right_center_info_pub = self.create_publisher(
            CameraInfo,
            'stereo_camera/right_center/camera_info',
            1
        )
        
        # GLib main loop and bus (for cleanup)
        self.main_loop = None
        self.bus = None
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Pre-allocate JPEG encode params (avoid list creation every frame)
        self._jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        
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

        self.get_logger().info(f'GStreamer stereo camera started: {self.device} @ {self.source_width}x{self.source_height} {self.framerate}fps')
        self.get_logger().info(f'Publishing 4 streams: wide ({self.output_width}x{self.output_height}, full FOV) + center ({self.output_width}x{self.output_height}, cropped FOV)')
        if self.flip_vertical:
            self.get_logger().info('Vertical flip enabled')

        # Pre-create camera info templates (only header.stamp changes per frame)
        # Wide: downsampled 2x, so focal_length_factor=0.5
        # Center: 1:1 crop, so focal_length_factor=1.0
        self._left_wide_camera_info_template = self._create_camera_info_template(
            'left_wide_camera_link', self.output_width, self.output_height, focal_length_factor=0.5)
        self._right_wide_camera_info_template = self._create_camera_info_template(
            'right_wide_camera_link', self.output_width, self.output_height, focal_length_factor=0.5)
        self._left_center_camera_info_template = self._create_camera_info_template(
            'left_center_camera_link', self.output_width, self.output_height, focal_length_factor=1.0)
        self._right_center_camera_info_template = self._create_camera_info_template(
            'right_center_camera_link', self.output_width, self.output_height, focal_length_factor=1.0)
    
    def _create_camera_info_template(self, frame_id, width, height, focal_length_factor=1.0):
        """Create a reusable CameraInfo template (everything except timestamp)

        Args:
            frame_id: Camera frame ID
            width: Image width in pixels
            height: Image height in pixels
            focal_length_factor: Multiplier for focal length (0.5 for wide/downsampled, 1.0 for center/cropped)
        """
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = width
        info.height = height

        # Approximate focal length (can be calibrated later)
        # For wide (downsampled): fx = width * focal_length_factor
        # For center (cropped): fx = width * focal_length_factor
        base_fx = float(width)
        fx = base_fx * focal_length_factor
        fy = fx  # Assume square pixels
        cx = float(width) / 2.0
        cy = float(height) / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return info
    
    def _publish_camera_info(self, publisher, template, stamp):
        """Publish camera info with updated timestamp"""
        template.header.stamp = stamp
        publisher.publish(template)
    
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
        """Create CPU-based pipeline with 4 outputs (fallback)"""
        # Pipeline elements:
        # v4l2src: Capture from camera at max resolution
        # image/jpeg: Force MJPEG format (no decode!)
        # appsink: Pull frames into our application

        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.source_width},height={self.source_height},framerate={self.framerate}/1 ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )

        self.get_logger().info(f'🖥️  CPU Pipeline: {pipeline_str}')
        self.get_logger().info(f'   Will decode once and produce 4 streams in software')

        pipeline = Gst.parse_launch(pipeline_str)
        appsink = pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample_software)

        return pipeline
    
    def _create_hardware_pipeline(self):
        """Create hardware-accelerated pipeline using Jetson NVDEC with GPU-based dual-resolution stereo split"""
        # Full hardware pipeline: decode → tee → 4 branches (2 wide + 2 center) → flip → encode (all on GPU!)
        # Uses nvvidconv left/right/top/bottom properties for cropping and scaling

        half_width = self.source_width // 2  # Each eye gets half the width
        flip_method = 2 if self.flip_vertical else 0  # 2 = vertical flip, 0 = no flip

        # Calculate center crop coordinates (output_width x output_height from center of each eye)
        # Center horizontally within each half: (half_width - output_width) / 2
        # Center vertically: (source_height - output_height) / 2
        center_crop_width = self.output_width
        center_crop_height = self.output_height

        # Horizontal offset from start of each half
        h_offset = (half_width - center_crop_width) // 2
        # Vertical offset
        v_offset = (self.source_height - center_crop_height) // 2

        # Left center: starts at half_width (right half start) + horizontal offset
        left_center_x_start = half_width + h_offset
        left_center_x_end = left_center_x_start + center_crop_width

        # Right center: starts at 0 (left half start) + horizontal offset
        right_center_x_start = h_offset
        right_center_x_end = right_center_x_start + center_crop_width

        # Vertical crop (same for both)
        center_y_start = v_offset
        center_y_end = v_offset + center_crop_height

        # Note: Camera left/right are swapped in the raw image
        # Left eye is in right half of image, right eye is in left half

        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.source_width},height={self.source_height},framerate={self.framerate}/1 ! '
            f'nvv4l2decoder mjpeg=1 ! '
            f'tee name=t '

            # Branch 1: Left Wide - crop right half and scale to output size
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left={half_width} right={self.source_width} top=0 bottom={self.source_height} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={self.output_width},height={self.output_height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=left_wide_sink emit-signals=true max-buffers=1 drop=true sync=false '

            # Branch 2: Right Wide - crop left half and scale to output size
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left=0 right={half_width} top=0 bottom={self.source_height} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={self.output_width},height={self.output_height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=right_wide_sink emit-signals=true max-buffers=1 drop=true sync=false '

            # Branch 3: Left Center - crop center region from right half
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left={left_center_x_start} right={left_center_x_end} top={center_y_start} bottom={center_y_end} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={self.output_width},height={self.output_height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=left_center_sink emit-signals=true max-buffers=1 drop=true sync=false '

            # Branch 4: Right Center - crop center region from left half
            f't. ! queue max-size-buffers=1 leaky=downstream ! '
            f'nvvidconv left={right_center_x_start} right={right_center_x_end} top={center_y_start} bottom={center_y_end} flip-method={flip_method} ! '
            f'video/x-raw(memory:NVMM),format=I420,width={self.output_width},height={self.output_height} ! '
            f'nvjpegenc quality={self.jpeg_quality} ! '
            f'appsink name=right_center_sink emit-signals=true max-buffers=1 drop=true sync=false'
        )

        flip_str = "Flip→" if self.flip_vertical else ""
        self.get_logger().info(f'🚀 Hardware Pipeline: {self.source_width}x{self.source_height}@{self.framerate}fps → 4 streams (2×wide + 2×center) all {self.output_width}x{self.output_height}')
        self.get_logger().info(f'   Full GPU: Decode→Tee→{flip_str}Crop/Scale→Encode')
        self.get_logger().info(f'   Center crop coordinates: Left[{left_center_x_start}:{left_center_x_end}, {center_y_start}:{center_y_end}], Right[{right_center_x_start}:{right_center_x_end}, {center_y_start}:{center_y_end}]')
        self.get_logger().debug(f'Pipeline: {pipeline_str}')

        try:
            pipeline = Gst.parse_launch(pipeline_str)

            # Connect all 4 sinks
            left_wide_sink = pipeline.get_by_name('left_wide_sink')
            left_wide_sink.connect('new-sample', self._on_new_sample_hardware_left_wide)

            right_wide_sink = pipeline.get_by_name('right_wide_sink')
            right_wide_sink.connect('new-sample', self._on_new_sample_hardware_right_wide)

            left_center_sink = pipeline.get_by_name('left_center_sink')
            left_center_sink.connect('new-sample', self._on_new_sample_hardware_left_center)

            right_center_sink = pipeline.get_by_name('right_center_sink')
            right_center_sink.connect('new-sample', self._on_new_sample_hardware_right_center)

            return pipeline

        except Exception as e:
            self.get_logger().error(f'Failed to create hardware pipeline: {e}')
            self.get_logger().warning('Falling back to CPU pipeline')
            self.use_hardware_accel = False
            return self._create_software_pipeline()
    
    def _on_new_sample_software(self, sink):
        """Callback for CPU pipeline - decode once, produce 4 outputs"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        receive_time = self.get_clock().now()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            # Decode JPEG once - use frombuffer directly
            img = cv2.imdecode(np.frombuffer(map_info.data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return Gst.FlowReturn.OK

            if self.flip_vertical:
                img = cv2.flip(img, 0)

            stamp = receive_time.to_msg()

            # Extract left and right halves
            half_width = img.shape[1] // 2
            left_half = img[:, half_width:]   # Left eye is in right half
            right_half = img[:, :half_width]  # Right eye is in left half

            # Wide angle: downsample full halves to output size
            left_wide = cv2.resize(left_half, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)
            right_wide = cv2.resize(right_half, (self.output_width, self.output_height), interpolation=cv2.INTER_AREA)

            # Center crop: extract center region from each half (dynamically calculated)
            # Center horizontally within each half: (half_width - output_width) / 2
            # Center vertically: (source_height - output_height) / 2
            h_offset = (half_width - self.output_width) // 2
            v_offset = (img.shape[0] - self.output_height) // 2

            left_center = left_half[v_offset:v_offset+self.output_height, h_offset:h_offset+self.output_width]
            right_center = right_half[v_offset:v_offset+self.output_height, h_offset:h_offset+self.output_width]

            # Publish all 4 streams
            self._publish_compressed_image(left_wide, self.left_wide_image_pub,
                                          'left_wide_camera_link', stamp)
            self._publish_compressed_image(right_wide, self.right_wide_image_pub,
                                          'right_wide_camera_link', stamp)
            self._publish_compressed_image(left_center, self.left_center_image_pub,
                                          'left_center_camera_link', stamp)
            self._publish_compressed_image(right_center, self.right_center_image_pub,
                                          'right_center_camera_link', stamp)

            # Publish camera info
            self._publish_camera_info(self.left_wide_info_pub,
                                     self._left_wide_camera_info_template, stamp)
            self._publish_camera_info(self.right_wide_info_pub,
                                     self._right_wide_camera_info_template, stamp)
            self._publish_camera_info(self.left_center_info_pub,
                                     self._left_center_camera_info_template, stamp)
            self._publish_camera_info(self.right_center_info_pub,
                                     self._right_center_camera_info_template, stamp)

        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _publish_compressed_image(self, img, publisher, frame_id, stamp):
        """Helper to publish a compressed image"""
        _, buf = cv2.imencode('.jpg', img, self._jpeg_params)
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        publisher.publish(msg)
    
    def _on_new_sample_hardware_left_wide(self, sink):
        """Callback for hardware pipeline LEFT WIDE image - full FOV downsampled"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        receive_time = self.get_clock().now()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            stamp = receive_time.to_msg()

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'left_wide_camera_link'
            msg.format = 'jpeg'
            msg.data = bytes(map_info.data)
            self.left_wide_image_pub.publish(msg)

            self._publish_camera_info(self.left_wide_info_pub, self._left_wide_camera_info_template, stamp)
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_new_sample_hardware_right_wide(self, sink):
        """Callback for hardware pipeline RIGHT WIDE image - full FOV downsampled"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        receive_time = self.get_clock().now()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            stamp = receive_time.to_msg()

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'right_wide_camera_link'
            msg.format = 'jpeg'
            msg.data = bytes(map_info.data)
            self.right_wide_image_pub.publish(msg)

            self._publish_camera_info(self.right_wide_info_pub, self._right_wide_camera_info_template, stamp)
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_new_sample_hardware_left_center(self, sink):
        """Callback for hardware pipeline LEFT CENTER image - center crop"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        receive_time = self.get_clock().now()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            stamp = receive_time.to_msg()

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'left_center_camera_link'
            msg.format = 'jpeg'
            msg.data = bytes(map_info.data)
            self.left_center_image_pub.publish(msg)

            self._publish_camera_info(self.left_center_info_pub, self._left_center_camera_info_template, stamp)
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_new_sample_hardware_right_center(self, sink):
        """Callback for hardware pipeline RIGHT CENTER image - center crop"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        receive_time = self.get_clock().now()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            stamp = receive_time.to_msg()

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'right_center_camera_link'
            msg.format = 'jpeg'
            msg.data = bytes(map_info.data)
            self.right_center_image_pub.publish(msg)

            self._publish_camera_info(self.right_center_info_pub, self._right_center_camera_info_template, stamp)
        finally:
            buf.unmap(map_info)

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
