#!/usr/bin/env python3
"""
Data Recorder Node for GR00T Training Dataset Collection

Records synchronized sensor data and robot state during teleoperation
for imitation learning. Saves ROS2 bags (MCAP) for later conversion
to the GR00T LeRobot format by scripts/rosbag_to_groot.py.

Bags land in <data_root>/<dataset>/demonstrations/demo_YYYYMMDD_HHMMSS.
The dataset is a ROS parameter and can be switched between episodes:

    ros2 param set /alfie/data_recorder dataset can_do_challenge

By default it logs only errors/warnings, "● RECORDING" once rosbag2 is really
recording and "■ SAVED" when the episode ends. For the step-by-step detail
(and rosbag2's own output) while debugging:

    ros2 param set /alfie/data_recorder verbose true
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
import time
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
        # Log the step-by-step detail (and rosbag2's info output); off = only
        # errors, warnings, recording started and recording saved.
        self.declare_parameter('verbose', False)

        self.data_root = self.get_parameter('data_root').value
        self.dataset = self.get_parameter('dataset').value
        self.output_dir_override = self.get_parameter('output_dir').value
        self.auto_start = self.get_parameter('auto_start').value
        self.max_duration = self.get_parameter('max_duration').value
        self.verbose = bool(self.get_parameter('verbose').value)

        self.output_dir = self._resolve_output_dir()
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        # Recording state
        self.recording = False
        self.bag_process = None
        self.current_bag_path = None
        self.recording_start_time = None
        # Set while `ros2 bag record` is starting up; cleared once it has
        # subscribed to everything (see _check_recorder_ready).
        self.ready_pending = False
        self.launch_mono = 0.0
        self.ready_mono = 0.0      # when rosbag2 finished subscribing
        self.last_wait_log = 0.0
        self.warned_slow = False

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

        # Status publisher (relative to namespace): STOPPED, STARTING:<ds>/<bag>
        # while rosbag2 spins up, RECORDING:<ds>/<bag> once it is subscribed.
        # Re-sent every second so late joiners (VR HUD, episode manager) see it.
        self.status_pub = self.create_publisher(
            String,
            'recording/status',
            10
        )
        self.status_text = 'STOPPED'

        # Allow switching dataset / data_root between episodes
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Timer to check recording duration
        self.create_timer(1.0, self.check_recording_duration)
        # Watches the recorder start up so we can say when it is really recording
        self.create_timer(0.1, self._check_recorder_ready)

        self.get_logger().info(
            f'Data recorder initialized. Dataset: {self.dataset}  Output dir: {self.output_dir}')
        self._detail(f'Recording {len(self.topics_to_record)} topics')

        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning recording...')
            self.start_recording()

    def _resolve_output_dir(self) -> str:
        if self.output_dir_override:
            return os.path.expanduser(self.output_dir_override)
        return str(demos_dir(self.dataset, self.data_root))

    def _detail(self, msg: str):
        """Step-by-step messages: INFO when verbose, DEBUG otherwise."""
        if self.verbose:
            self.get_logger().info(msg)
        else:
            self.get_logger().debug(msg)

    def _on_set_parameters(self, params):
        """Apply dataset/data_root/output_dir changes; refuse mid-recording."""
        for p in params:
            if p.name == 'verbose':
                self.verbose = bool(p.value)
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
            self._detail('Already recording, ignoring start')
            return

        # Generate bag file name with timestamp
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f'demo_{timestamp}'
        self.current_bag_path = os.path.join(self.output_dir, bag_name)

        # Build rosbag2 record command
        cmd = ['ros2', 'bag', 'record']
        cmd.extend(['-o', self.current_bag_path])

        # Add all topics (positional topics are deprecated in Jazzy)
        cmd.append('--topics')
        cmd.extend(self.topics_to_record)

        # Recording is driven by recording/start|stop, not the terminal; also
        # avoids the "stdin is not a terminal" notice under launch.
        cmd.append('--disable-keyboard-controls')

        # Use MCAP storage format (better for large datasets)
        cmd.extend(['-s', 'mcap'])

        # Compression
        cmd.extend(['--compression-mode', 'file'])
        cmd.extend(['--compression-format', 'zstd'])

        # rosbag2 prints a line per subscribed topic plus storage chatter
        cmd.extend(['--log-level', 'info' if self.verbose else 'warn'])

        try:
            self._detail(f'Starting recording: {bag_name} (dataset: {self.dataset})')
            self._detail(f'Command: {" ".join(cmd)}')

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

            self._publish_status(f'STARTING:{self.dataset}/{bag_name}')

            self.ready_pending = True
            self.launch_mono = time.monotonic()
            self.last_wait_log = self.launch_mono
            self.warned_slow = False
            self._detail(f'Recorder launched for {bag_name}; waiting for it to subscribe...')

        except Exception as e:
            self.get_logger().error(f'Failed to start recording: {e}')
            self.bag_process = None
            self.recording = False

    RECORDER_NODE_PREFIX = 'rosbag2'
    SLOW_START_S = 15.0

    def _recorder_subscribed(self, topic: str) -> bool:
        return any(info.node_name.startswith(self.RECORDER_NODE_PREFIX)
                   for info in self.get_subscriptions_info_by_topic(topic))

    def _check_recorder_ready(self):
        """Log once `ros2 bag record` has subscribed to every live topic.

        Popen returns long before the recorder is up (CLI start, discovery,
        subscriptions take a few seconds), so the episode only really starts
        here. Topics nobody publishes are reported rather than waited for:
        rosbag2 cannot subscribe to them until they appear.
        """
        if not self.ready_pending or not self.recording:
            return
        now = time.monotonic()
        elapsed = now - self.launch_mono
        bag_name = os.path.basename(self.current_bag_path or '')

        code = self.bag_process.poll() if self.bag_process else None
        if code is not None:
            self.ready_pending = False
            self.get_logger().error(
                f'✗ rosbag2 exited (code {code}) {elapsed:.1f}s after launch, before it '
                f'started recording {bag_name}; see its output above')
            self.bag_process = None
            self.recording = False
            self.current_bag_path = None
            self.recording_start_time = None
            self._publish_status('STOPPED')
            return

        live = [t for t in self.topics_to_record if self.count_publishers(t) > 0]
        waiting = [t for t in live if not self._recorder_subscribed(t)]
        if not waiting:
            self.ready_pending = False
            self.ready_mono = now
            self._publish_status(f'RECORDING:{self.dataset}/{bag_name}')
            self.get_logger().info(
                f'● RECORDING {bag_name}: all {len(live)} live topics subscribed, '
                f'{elapsed:.1f}s after start')
            unpublished = [t for t in self.topics_to_record if t not in live]
            if unpublished:
                self.get_logger().warn(
                    f'  not being recorded (no publisher): {", ".join(unpublished)}')
            return

        if elapsed > self.SLOW_START_S and not self.warned_slow:
            self.warned_slow = True
            self.get_logger().warn(
                f'Recorder still not subscribed after {elapsed:.0f}s; missing: '
                f'{", ".join(waiting)}')
        elif now - self.last_wait_log >= 2.0:
            self.last_wait_log = now
            self._detail(
                f'  recorder starting ({elapsed:.1f}s): '
                f'{len(live) - len(waiting)}/{len(live)} topics subscribed')

    def stop_recording(self, reason='user_request'):
        """Stop ROS2 bag recording"""
        if not self.recording:
            self._detail('Not currently recording, ignoring stop')
            return

        bag_path = self.current_bag_path
        try:
            if self.bag_process:
                self._detail(f'Stopping recording (reason: {reason})...')

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

                self._publish_status('STOPPED')

                if self.ready_pending:
                    length = 'stopped before rosbag2 finished subscribing'
                else:
                    length = f'{time.monotonic() - self.ready_mono:.1f}s'
                why = '' if reason == 'user_request' else f' ({reason})'
                self.get_logger().info(
                    f'■ SAVED {os.path.basename(bag_path)}: {length}{why}')
                self._detail(f'  {bag_path}; play back: ros2 bag play {bag_path}')

        except Exception as e:
            self.get_logger().error(f'Error stopping recording: {e}')
        finally:
            # Always leave a clean state so the next start works.
            self.bag_process = None
            self.recording = False
            self.current_bag_path = None
            self.recording_start_time = None
            self.ready_pending = False

    def _publish_status(self, text: str):
        self.status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def check_recording_duration(self):
        """Re-send the status, and stop if recording exceeded max duration"""
        self._publish_status(self.status_text)
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
            self._detail('Node shutting down, stopping recording...')
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
