#!/usr/bin/env python3
"""
Data Recorder Node for GR00T Training Dataset Collection

Records synchronized sensor data and robot actions during teleoperation
for imitation learning. Saves to ROS2 bag format for later conversion
to GR00T training format.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import os
import subprocess
import datetime
from pathlib import Path

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist


class DataRecorder(Node):
    """Records demonstration data to ROS2 bags with metadata"""

    def __init__(self):
        super().__init__('data_recorder')

        # Parameters
        self.declare_parameter('output_dir', '~/alfiebot_ws/data/demonstrations')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_duration', 300.0)  # 5 minutes max per demo

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value
        )
        self.auto_start = self.get_parameter('auto_start').value
        self.max_duration = self.get_parameter('max_duration').value

        # Create output directory
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        # Recording state
        self.recording = False
        self.bag_process = None
        self.current_bag_path = None
        self.recording_start_time = None

        # Topics to record
        self.topics_to_record = [
            # Camera feeds (4 stereo streams - compressed)
            '/alfie/stereo_camera/left_wide/image_raw/compressed',
            '/alfie/stereo_camera/left_wide/camera_info',
            '/alfie/stereo_camera/left_center/image_raw/compressed',
            '/alfie/stereo_camera/left_center/camera_info',
            '/alfie/stereo_camera/right_center/image_raw/compressed',
            '/alfie/stereo_camera/right_center/camera_info',
            '/alfie/stereo_camera/right_wide/image_raw/compressed',
            '/alfie/stereo_camera/right_wide/camera_info',

            # Robot state
            '/alfie/joint_states',
            '/alfie/robotlowstate',  # Low-level robot state
            '/alfie/low/gdb0state',  # Gripper 0 state
            '/alfie/low/gdb1state',  # Gripper 1 state

            # Robot commands (what we want the policy to learn)
            '/alfie/robotlowcmd',    # Low-level robot commands
            '/alfie/low/gdb0cmd',    # Gripper 0 commands
            '/alfie/low/gdb1cmd',    # Gripper 1 commands
            '/alfie/low/backcmd',    # Base movement commands

            # TF for camera transforms
            '/alfie/tf',
            '/alfie/tf_static',

            # Metadata
            '/recording/metadata',
        ]

        # QoS profile for real-time data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Control subscribers (relative to namespace, use BEST_EFFORT to match publisher)
        self.create_subscription(
            Bool,
            'recording/start',
            self.start_recording_callback,
            qos_profile
        )

        self.create_subscription(
            Bool,
            'recording/stop',
            self.stop_recording_callback,
            qos_profile
        )

        # Metadata publisher for annotations (relative to namespace)
        self.metadata_pub = self.create_publisher(
            String,
            'recording/metadata',
            10
        )

        # Status publisher (relative to namespace)
        self.status_pub = self.create_publisher(
            String,
            'recording/status',
            10
        )

        # Timer to check recording duration
        self.create_timer(1.0, self.check_recording_duration)

        self.get_logger().info(f'Data recorder initialized. Output dir: {self.output_dir}')
        self.get_logger().info(f'Recording {len(self.topics_to_record)} topics')

        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning recording...')
            self.start_recording()

    def start_recording_callback(self, msg):
        """Handle start recording request"""
        if msg.data and not self.recording:
            self.start_recording()

    def stop_recording_callback(self, msg):
        """Handle stop recording request"""
        if msg.data and self.recording:
            self.stop_recording()

    def start_recording(self):
        """Start ROS2 bag recording"""
        if self.recording:
            self.get_logger().warn('Already recording!')
            return

        # Generate bag file name with timestamp
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f'demo_{timestamp}'
        self.current_bag_path = os.path.join(self.output_dir, bag_name)

        # Build rosbag2 record command
        cmd = ['ros2', 'bag', 'record']
        cmd.extend(['-o', self.current_bag_path])

        # Add all topics
        for topic in self.topics_to_record:
            cmd.extend([topic])

        # Use MCAP storage format (better for large datasets)
        cmd.extend(['-s', 'mcap'])

        # Compression
        cmd.extend(['--compression-mode', 'file'])
        cmd.extend(['--compression-format', 'zstd'])

        try:
            self.get_logger().info(f'Starting recording: {bag_name}')
            self.get_logger().info(f'Command: {" ".join(cmd)}')

            # Start recording process
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.recording = True
            self.recording_start_time = self.get_clock().now()

            # Publish initial metadata
            metadata = String()
            metadata.data = f'timestamp:{timestamp},type:demonstration,status:started'
            self.metadata_pub.publish(metadata)

            status = String()
            status.data = f'RECORDING:{bag_name}'
            self.status_pub.publish(status)

            self.get_logger().info(f'✓ Recording started: {bag_name}')

        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')
            self.recording = False

    def stop_recording(self, reason='user_request'):
        """Stop ROS2 bag recording"""
        if not self.recording:
            self.get_logger().warn('Not currently recording!')
            return

        try:
            if self.bag_process:
                self.get_logger().info(f'Stopping recording (reason: {reason})...')

                # Send SIGINT to rosbag2 process
                self.bag_process.terminate()
                self.bag_process.wait(timeout=5.0)

                # Publish final metadata
                metadata = String()
                metadata.data = f'status:stopped,reason:{reason}'
                self.metadata_pub.publish(metadata)

                status = String()
                status.data = 'STOPPED'
                self.status_pub.publish(status)

                self.get_logger().info(f'✓ Recording saved: {self.current_bag_path}')
                self.get_logger().info(f'  To play back: ros2 bag play {self.current_bag_path}')

                self.bag_process = None
                self.recording = False
                self.current_bag_path = None
                self.recording_start_time = None

        except Exception as e:
            self.get_logger().error(f'Error stopping recording: {e}')

    def check_recording_duration(self):
        """Check if recording has exceeded max duration"""
        if not self.recording or not self.recording_start_time:
            return

        duration = (self.get_clock().now() - self.recording_start_time).nanoseconds / 1e9

        if duration > self.max_duration:
            self.get_logger().warn(
                f'Recording exceeded max duration ({self.max_duration}s), stopping...'
            )
            self.stop_recording(reason='max_duration_exceeded')

    def destroy_node(self):
        """Cleanup on node shutdown"""
        if self.recording:
            self.get_logger().info('Node shutting down, stopping recording...')
            self.stop_recording(reason='node_shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
