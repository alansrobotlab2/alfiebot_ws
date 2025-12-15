#!/usr/bin/env python3
"""
Stereo Colorizer Node

Colorizes mono left/right stereo images using the RGB camera.
Uses depth information and camera calibration to project RGB colors
onto the mono stereo views for colored 3D stereo vision.

Subscribes to:
- RGB image (color source)
- Left mono image (rectified)
- Right mono image (rectified)
- Depth image (aligned to RGB)
- Camera info for all cameras

Publishes:
- Colorized left image
- Colorized right image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading


class StereoColorizerNode(Node):
    """
    Node that colorizes mono stereo images using RGB camera data.
    
    The approach:
    1. Use depth to get 3D points from RGB camera view
    2. Project those 3D points into left/right camera views
    3. Sample RGB colors and apply to left/right images
    """
    
    def __init__(self):
        super().__init__('stereo_colorizer')
        
        # Declare parameters
        self.declare_parameter('rgb_image_topic', '/alfie/oak/rgb/image_raw')
        self.declare_parameter('rgb_info_topic', '/alfie/oak/rgb/camera_info')
        self.declare_parameter('left_image_topic', '/alfie/oak/left/image_rect')
        self.declare_parameter('left_info_topic', '/alfie/oak/left/camera_info')
        self.declare_parameter('right_image_topic', '/alfie/oak/right/image_rect')
        self.declare_parameter('right_info_topic', '/alfie/oak/right/camera_info')
        self.declare_parameter('depth_topic', '/alfie/oak/stereo/image_raw')
        self.declare_parameter('output_left_topic', '/alfie/oak/left/image_color')
        self.declare_parameter('output_right_topic', '/alfie/oak/right/image_color')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        self.rgb_image_topic = self.get_parameter('rgb_image_topic').value
        self.rgb_info_topic = self.get_parameter('rgb_info_topic').value
        self.left_image_topic = self.get_parameter('left_image_topic').value
        self.left_info_topic = self.get_parameter('left_info_topic').value
        self.right_image_topic = self.get_parameter('right_image_topic').value
        self.right_info_topic = self.get_parameter('right_info_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.output_left_topic = self.get_parameter('output_left_topic').value
        self.output_right_topic = self.get_parameter('output_right_topic').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.bridge = CvBridge()
        
        # Camera calibration data
        self.rgb_K = None  # RGB intrinsics
        self.left_K = None  # Left intrinsics
        self.right_K = None  # Right intrinsics
        self.rgb_D = None  # RGB distortion
        self.left_D = None
        self.right_D = None
        
        # Extrinsics - will be computed from TF or estimated
        # OAK-D camera baseline is approximately 75mm
        self.baseline = 0.075  # meters
        self.rgb_to_left_R = np.eye(3)
        self.rgb_to_left_t = np.array([self.baseline / 2, 0, 0])  # RGB is center, left is offset
        self.rgb_to_right_R = np.eye(3)
        self.rgb_to_right_t = np.array([-self.baseline / 2, 0, 0])  # Right is opposite offset
        
        self.calibration_received = False
        self.lock = threading.Lock()
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to camera info (one-shot style, just need calibration)
        self.rgb_info_sub = self.create_subscription(
            CameraInfo, self.rgb_info_topic, self.rgb_info_callback, qos)
        self.left_info_sub = self.create_subscription(
            CameraInfo, self.left_info_topic, self.left_info_callback, qos)
        self.right_info_sub = self.create_subscription(
            CameraInfo, self.right_info_topic, self.right_info_callback, qos)
        
        # Use message_filters to sync RGB, depth, left, right images
        self.rgb_sub = Subscriber(self, Image, self.rgb_image_topic, qos_profile=qos)
        self.depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos)
        self.left_sub = Subscriber(self, Image, self.left_image_topic, qos_profile=qos)
        self.right_sub = Subscriber(self, Image, self.right_image_topic, qos_profile=qos)
        
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.left_sub, self.right_sub],
            queue_size=5,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synced_callback)
        
        # Publishers
        if self.publish_compressed:
            self.left_pub = self.create_publisher(
                CompressedImage, self.output_left_topic + '/compressed', 10)
            self.right_pub = self.create_publisher(
                CompressedImage, self.output_right_topic + '/compressed', 10)
        else:
            self.left_pub = self.create_publisher(
                Image, self.output_left_topic, 10)
            self.right_pub = self.create_publisher(
                Image, self.output_right_topic, 10)
        
        # Also publish raw for compatibility
        self.left_raw_pub = self.create_publisher(Image, self.output_left_topic, 10)
        self.right_raw_pub = self.create_publisher(Image, self.output_right_topic, 10)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Stereo Colorizer Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'RGB: {self.rgb_image_topic}')
        self.get_logger().info(f'Left: {self.left_image_topic}')
        self.get_logger().info(f'Right: {self.right_image_topic}')
        self.get_logger().info(f'Depth: {self.depth_topic}')
        self.get_logger().info(f'Output left: {self.output_left_topic}')
        self.get_logger().info(f'Output right: {self.output_right_topic}')
        
    def rgb_info_callback(self, msg: CameraInfo):
        if self.rgb_K is None:
            self.rgb_K = np.array(msg.k).reshape(3, 3)
            self.rgb_D = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            self.get_logger().info(f'RGB camera info received: {msg.width}x{msg.height}')
            self.check_calibration()
    
    def left_info_callback(self, msg: CameraInfo):
        if self.left_K is None:
            self.left_K = np.array(msg.k).reshape(3, 3)
            self.left_D = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            # Extract baseline from P matrix if available
            if len(msg.p) == 12:
                P = np.array(msg.p).reshape(3, 4)
                # P[0,3] = -fx * baseline for right camera
                # For left camera, P[0,3] should be 0
            self.get_logger().info(f'Left camera info received: {msg.width}x{msg.height}')
            self.check_calibration()
    
    def right_info_callback(self, msg: CameraInfo):
        if self.right_K is None:
            self.right_K = np.array(msg.k).reshape(3, 3)
            self.right_D = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            # Extract baseline from P matrix
            if len(msg.p) == 12:
                P = np.array(msg.p).reshape(3, 4)
                fx = P[0, 0]
                if fx > 0 and P[0, 3] != 0:
                    self.baseline = abs(P[0, 3] / fx)
                    self.get_logger().info(f'Extracted baseline: {self.baseline*1000:.1f}mm')
            self.get_logger().info(f'Right camera info received: {msg.width}x{msg.height}')
            self.check_calibration()
    
    def check_calibration(self):
        if self.rgb_K is not None and self.left_K is not None and self.right_K is not None:
            self.calibration_received = True
            self.get_logger().info('All camera calibrations received!')
    
    def synced_callback(self, rgb_msg: Image, depth_msg: Image, 
                        left_msg: Image, right_msg: Image):
        """Process synchronized images and colorize stereo pair."""
        if not self.calibration_received:
            return
        
        try:
            # Convert images
            rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            left_img = self.bridge.imgmsg_to_cv2(left_msg, 'mono8')
            right_img = self.bridge.imgmsg_to_cv2(right_msg, 'mono8')
            
            # Colorize left and right images
            left_color = self.colorize_from_rgb(
                rgb_img, depth_img, left_img, 
                self.rgb_K, self.left_K, 
                self.rgb_to_left_R, self.rgb_to_left_t,
                is_left=True
            )
            
            right_color = self.colorize_from_rgb(
                rgb_img, depth_img, right_img,
                self.rgb_K, self.right_K,
                self.rgb_to_right_R, self.rgb_to_right_t,
                is_left=False
            )
            
            # Publish
            self.publish_images(left_color, right_color, rgb_msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error in synced_callback: {e}')
    
    def colorize_from_rgb(self, rgb_img, depth_img, mono_img, 
                          rgb_K, target_K, R, t, is_left=True):
        """
        Colorize a mono image using RGB and depth.
        
        Simple approach: For each pixel in the mono image, find the corresponding
        pixel in RGB using the depth and camera geometry.
        """
        h, w = mono_img.shape[:2]
        rgb_h, rgb_w = rgb_img.shape[:2]
        
        # Create output color image
        color_img = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Resize depth to match RGB if needed
        if depth_img.shape[:2] != rgb_img.shape[:2]:
            depth_img = cv2.resize(depth_img, (rgb_w, rgb_h), interpolation=cv2.INTER_NEAREST)
        
        # Simple stereo colorization approach:
        # Use horizontal disparity based on depth and baseline
        
        # Convert depth to float meters
        if depth_img.dtype == np.uint16:
            depth_m = depth_img.astype(np.float32) / 1000.0  # mm to m
        else:
            depth_m = depth_img.astype(np.float32)
        
        # Get focal length from RGB camera
        fx = rgb_K[0, 0]
        cx = rgb_K[0, 2]
        cy = rgb_K[1, 2]
        
        # Target camera params
        fx_t = target_K[0, 0]
        cx_t = target_K[0, 2]
        cy_t = target_K[1, 2]
        
        # Scale factors between cameras
        scale_x = rgb_w / w
        scale_y = rgb_h / h
        
        # For each pixel in target (mono) image, compute where it maps in RGB
        # Using simple horizontal disparity model
        
        # Create coordinate grids
        y_coords, x_coords = np.mgrid[0:h, 0:w]
        
        # Scale to RGB image coordinates
        x_rgb = (x_coords * scale_x).astype(np.int32)
        y_rgb = (y_coords * scale_y).astype(np.int32)
        
        # Clip to valid range
        x_rgb = np.clip(x_rgb, 0, rgb_w - 1)
        y_rgb = np.clip(y_rgb, 0, rgb_h - 1)
        
        # Get depth at each point
        depth_at_points = depth_m[y_rgb, x_rgb]
        
        # Compute disparity shift based on depth and baseline
        # disparity = fx * baseline / depth
        valid_depth = depth_at_points > 0.1  # Filter out invalid depth
        disparity = np.zeros_like(depth_at_points)
        disparity[valid_depth] = (fx * self.baseline / 2) / depth_at_points[valid_depth]
        
        # Shift x coordinates based on camera (left or right)
        if is_left:
            x_rgb_shifted = x_rgb + (disparity * scale_x).astype(np.int32)
        else:
            x_rgb_shifted = x_rgb - (disparity * scale_x).astype(np.int32)
        
        # Clip to valid range
        x_rgb_shifted = np.clip(x_rgb_shifted, 0, rgb_w - 1)
        
        # Sample RGB colors
        color_img = rgb_img[y_rgb, x_rgb_shifted]
        
        # Blend with mono for areas with invalid depth
        mono_color = cv2.cvtColor(mono_img, cv2.COLOR_GRAY2BGR)
        invalid_mask = ~valid_depth
        color_img[invalid_mask] = mono_color[invalid_mask]
        
        return color_img
    
    def publish_images(self, left_color, right_color, header):
        """Publish colorized images."""
        
        if self.publish_compressed:
            # Publish compressed
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            
            _, left_encoded = cv2.imencode('.jpg', left_color, encode_param)
            left_msg = CompressedImage()
            left_msg.header = header
            left_msg.format = 'jpeg'
            left_msg.data = left_encoded.tobytes()
            self.left_pub.publish(left_msg)
            
            _, right_encoded = cv2.imencode('.jpg', right_color, encode_param)
            right_msg = CompressedImage()
            right_msg.header = header
            right_msg.format = 'jpeg'
            right_msg.data = right_encoded.tobytes()
            self.right_pub.publish(right_msg)
        
        # Also publish raw
        left_msg = self.bridge.cv2_to_imgmsg(left_color, 'bgr8')
        left_msg.header = header
        self.left_raw_pub.publish(left_msg)
        
        right_msg = self.bridge.cv2_to_imgmsg(right_color, 'bgr8')
        right_msg.header = header
        self.right_raw_pub.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoColorizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
