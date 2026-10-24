#!/usr/bin/env python3
"""
Data Recorder Node for GR00T Training Dataset Collection

Records synchronized sensor data and robot state during teleoperation
for imitation learning. Saves ROS2 bags (MCAP) for later conversion
to the GR00T LeRobot format by scripts/rosbag_to_groot.py.

Bags land in <data_root>/<dataset>/demonstrations/demo_YYYYMMDD_HHMMSS.
The dataset is a ROS parameter and can be switched between episodes:

    ros2 param set /alfie/data_recorder dataset can_do_challenge
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from rcl_interfaces.msg import SetParametersResult
import os
import subprocess
import datetime
from pathlib import Path

from std_msgs.msg import Bool, String

from alfie_gr00t.dataset_paths import (
    DEFAULT_DATA_ROOT, DEFAULT_DATASET, demos_dir,
)


class DataRecorder(Node):
    """Records demonstration data to ROS2 bags with metadata"""

    def __init__(self):
        super().__init__('data_recorder')

        # Parameters
        self.declare_parameter('data_root', DEFAULT_DATA_ROOT)
        self.declare_parameter('dataset', DEFAULT_DATASET)
        # Explicit bag directory; overrides data_root/dataset when non-empty.
        self.declare_parameter('output_dir', '')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('max_duration', 300.0)  # 5 minutes max per demo

        self.data_root = self.get_parameter('data_root').value
        self.dataset = self.get_parameter('dataset').value
        self.output_dir_override = self.get_parameter('output_dir').value
        self.auto_start = self.get_parameter('auto_start').value
        self.max_duration = self.get_parameter('max_duration').value

        self.output_dir = self._resolve_output_dir()
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

            # Robot state. State AND action both come from this message:
            # servo current_location / target_location, back current /
            # command position, current_cmd_vel / command_cmd_vel.
            '/alfie/robotlowstate',

            '/alfie/robot_description',  # URDF
            '/alfie/tf',
            '/alfie/tf_static',

            '/rosout',

            # Metadata
            '/alfie/recording/metadata',
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

        # Metadata publisher for annotations (relative to namespace).
        # Transient-local so `ros2 bag record`, which subscribes a moment
        # after we publish the start annotation, still receives it.
        self.metadata_pub = self.create_publisher(
            String,
            'recording/metadata',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            )
        )

        # Status publisher (relative to namespace)
        self.status_pub = self.create_publisher(
            String,
            'recording/status',
            10
        )

        # Allow switching dataset / data_root between episodes
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Timer to check recording duration
        self.create_timer(1.0, self.check_recording_duration)

        self.get_logger().info(
            f'Data recorder initialized. Dataset: {self.dataset}  Output dir: {self.output_dir}')
        self.get_logger().info(f'Recording {len(self.topics_to_record)} topics')

        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning recording...')
            self.start_recording()

    def _resolve_output_dir(self) -> str:
        if self.output_dir_override:
            return os.path.expanduser(self.output_dir_override)
        return str(demos_dir(self.dataset, self.data_root))

    def _on_set_parameters(self, params):
        """Apply dataset/data_root/output_dir changes; refuse mid-recording."""
        path_params = {p.name: p.value for p in params
                       if p.name in ('dataset', 'data_root', 'output_dir')}
        if not path_params:
            return SetParametersResult(successful=True)
        if self.recording:
            return SetParametersResult(
                successful=False,
                reason='Cannot change dataset while recording; stop first')
        if 'dataset' in path_params:
            name = str(path_params['dataset']).strip()
            if not name or '/' in name or name in ('.', '..'):
                return SetParametersResult(
                    successful=False,
                    reason="dataset must be a plain folder name (no '/')")
            self.dataset = name
        if 'data_root' in path_params:
            self.data_root = str(path_params['data_root'])
        if 'output_dir' in path_params:
            self.output_dir_override = str(path_params['output_dir'])

        self.output_dir = self._resolve_output_dir()
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f'Dataset switched to: {self.dataset}  Output dir: {self.output_dir}')
        return SetParametersResult(successful=True)

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
            self.get_logger().info(f'Starting recording: {bag_name} (dataset: {self.dataset})')
            self.get_logger().info(f'Command: {" ".join(cmd)}')

            # Start recording process. Let its output flow to our console:
            # a PIPE nobody drains would eventually block rosbag2.
            self.bag_process = subprocess.Popen(cmd)

            self.recording = True
            self.recording_start_time = self.get_clock().now()

            # Publish initial metadata
            metadata = String()
            metadata.data = (
                f'timestamp:{timestamp},type:demonstration,'
                f'dataset:{self.dataset},status:started'
            )
            self.metadata_pub.publish(metadata)

            status = String()
            status.data = f'RECORDING:{self.dataset}/{bag_name}'
            self.status_pub.publish(status)

            self.get_logger().info(f'✓ Recording started: {bag_name}')

        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')
            self.bag_process = None
            self.recording = False

    def stop_recording(self, reason='user_request'):
        """Stop ROS2 bag recording"""
        if not self.recording:
            self.get_logger().warn('Not currently recording!')
            return

        bag_path = self.current_bag_path
        try:
            if self.bag_process:
                self.get_logger().info(f'Stopping recording (reason: {reason})...')

                # Final annotation goes out before the bag closes so it is
                # captured in the recording.
                metadata = String()
                metadata.data = f'status:stopped,reason:{reason}'
                self.metadata_pub.publish(metadata)

                # SIGTERM: ros2 bag record installs a handler that closes the
                # bag cleanly (writes metadata.yaml, finishes compression).
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().error(
                        'rosbag2 did not exit after SIGTERM, killing it; '
                        'the bag may be incomplete')
                    self.bag_process.kill()
                    self.bag_process.wait(timeout=5.0)

                status = String()
                status.data = 'STOPPED'
                self.status_pub.publish(status)

                self.get_logger().info(f'✓ Recording saved: {bag_path}')
                self.get_logger().info(f'  To play back: ros2 bag play {bag_path}')

        except Exception as e:
            self.get_logger().error(f'Error stopping recording: {e}')
        finally:
            # Always leave a clean state so the next start works.
            self.bag_process = None
            self.recording = False
            self.current_bag_path = None
            self.recording_start_time = None

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
