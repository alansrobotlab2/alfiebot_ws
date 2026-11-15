#!/usr/bin/env python3
"""
Data Recorder Node for GR00T Training Dataset Collection

Records synchronized sensor data and robot state during teleoperation
for imitation learning. Saves ROS2 bags (MCAP) for later conversion
to the GR00T LeRobot format by scripts/rosbag_to_groot.py.

Bags land in <data_root>/<session>/demonstrations/demo_YYYYMMDD_HHMMSS, where
<session> is a recording session folder YYYY-MM-DD_sNN_hwN (dataset_paths.py).
With the `dataset` parameter empty (the default) the recorder uses today's
latest open session and starts a new one when there is none or it was closed.
Naming a dataset pins the folder; it can be switched between episodes:

    ros2 param set /alfie/data_recorder dataset 2026-11-20_s02_hw2

Each bag folder also gets an episode.json with the facts only known at record
time: task ID, source (demo / correction / ...), correction round, the policy
checkpoint and run a correction came from, operator, hardware version. The
operator-set part is the recording context (recording_context.py), changed
from the episode manager (JSON on recording/set_context) and republished
latched on recording/context. A VR takeover of a GR00T run overrides it: the
episode is a correction of that run's task.

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
import json
import os
import subprocess
import datetime
import time
from pathlib import Path

from std_msgs.msg import Bool, String

from alfie_gr00t import task_registry
from alfie_gr00t.dataset_paths import (
    DEFAULT_DATA_ROOT, EPISODE_FILE, create_session, data_root,
    demos_dir, is_closed, open_session,
)
from alfie_gr00t.recording_context import (
    CONTEXT_FILE, DEFAULT_CONTEXT, merge_context,
)

TAKEOVER_TOPIC = '/alfie/groot/takeover'
# A takeover event stamped this close before a recording start belongs to it
# (the event is latched, so older ones are replayed to every new subscriber).
TAKEOVER_WINDOW_S = 15.0


class DataRecorder(Node):
    """Records demonstration data to ROS2 bags with metadata"""

    def __init__(self):
        super().__init__('data_recorder')

        # Parameters
        self.declare_parameter('data_root', DEFAULT_DATA_ROOT)
        # '' = today's open recording session, created when needed.
        self.declare_parameter('dataset', '')
        # Task registry file; '' = the installed config/tasks.yaml.
        self.declare_parameter('tasks_yaml', '')
        # alfiebot git checkout; its commit is stored with every episode.
        self.declare_parameter('workspace_dir', '~/alfiebot_ws')
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

        self.tasks_yaml = self.get_parameter('tasks_yaml').value or None
        self.workspace_dir = os.path.expanduser(self.get_parameter('workspace_dir').value)
        self.output_dir = self._resolve_output_dir()

        # Recording context (operator-set episode facts), kept across restarts.
        self.rec_context = dict(DEFAULT_CONTEXT)
        self._load_context()
        self.last_takeover = None      # latest parsed takeover event
        self.episode_record = None     # facts of the episode being recorded

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

            # VR takeover of a GR00T run (latched JSON {stamp_ns, task}); a bag
            # started by the takeover gets it on subscribe. The converter
            # tags such episodes origin=takeover and takes their task from it.
            '/alfie/groot/takeover',
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

        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        # Recording context: JSON updates in, the full context out (latched).
        self.context_pub = self.create_publisher(String, 'recording/context', latched)
        self.create_subscription(String, 'recording/set_context', self._set_context_callback, 10)
        # VR takeover of a GR00T run: the next episode is a correction of it.
        self.create_subscription(String, TAKEOVER_TOPIC, self._takeover_callback, latched)

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

        self._publish_context()
        self.get_logger().info(
            f'Data recorder initialized. Session: {self.dataset or "(today\'s, on first recording)"}'
            f'  Data root: {data_root(self.data_root)}  Task: {self.rec_context["task_id"] or "(not set)"}')
        self._detail(f'Recording {len(self.topics_to_record)} topics')

        if self.auto_start:
            self.get_logger().info('Auto-start enabled, beginning recording...')
            self.start_recording()

    def _resolve_output_dir(self) -> str:
        """Bag directory for the current settings ('' until a session is chosen)."""
        if self.output_dir_override:
            return os.path.expanduser(self.output_dir_override)
        if not self.dataset:
            return ''
        return str(demos_dir(self.dataset, self.data_root))

    def _hw(self) -> str:
        try:
            return task_registry.current_hw(self.tasks_yaml)
        except Exception as e:   # noqa: BLE001 - recording must not depend on the registry
            self.get_logger().error(f'Cannot read hardware.yaml ({e}); tagging episodes hw0')
            return 'hw0'

    def _registry(self):
        try:
            return task_registry.load(self.tasks_yaml)
        except Exception as e:   # noqa: BLE001
            self.get_logger().error(f'Cannot read the task registry: {e}')
            return None

    def _choose_session(self) -> bool:
        """Settle the folder the next bag goes into. False = cannot record."""
        if self.output_dir_override:
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)
            return True
        root = data_root(self.data_root)
        hw = self._hw()
        if self.dataset and is_closed(root / self.dataset):
            self.get_logger().warn(
                f'Session {self.dataset} is closed; recording into a new session')
            self.dataset = ''
        if not self.dataset:
            try:
                self.dataset = open_session(root, hw) or create_session(root, hw)
            except OSError as e:
                self.get_logger().error(f'Cannot create a session under {root}: {e}')
                return False
            self.get_logger().info(f'Recording session: {self.dataset}')
        self.output_dir = self._resolve_output_dir()
        Path(self.output_dir).mkdir(parents=True, exist_ok=True)
        self._publish_context()
        return True

    # -- recording context -------------------------------------------------

    def _context_path(self) -> Path:
        return data_root(self.data_root) / CONTEXT_FILE

    def _load_context(self):
        try:
            with open(self._context_path()) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            return
        reg = self._registry()
        self.rec_context, error = merge_context(None, saved, reg.tasks if reg else {})
        if error:
            self.get_logger().warn(f'Saved recording context ignored: {error}')

    def _publish_context(self):
        msg = String()
        msg.data = json.dumps({**self.rec_context, 'session': self.dataset, 'hw': self._hw()})
        self.context_pub.publish(msg)

    def _set_context_callback(self, msg: String):
        try:
            update = json.loads(msg.data)
        except ValueError:
            update = None
        reg = self._registry()
        new, error = merge_context(self.rec_context, update, reg.tasks if reg else {})
        if error:
            self.get_logger().warn(f'Recording context update refused: {error}')
        elif isinstance(update, dict) and 'session' in update and self.recording:
            self.get_logger().warn('Cannot switch session while recording; stop first')
        else:
            if isinstance(update, dict) and 'session' in update:
                name = str(update['session'] or '')
                if name and not (data_root(self.data_root) / name).is_dir():
                    self.get_logger().warn(f'No session "{name}"; keeping {self.dataset}')
                else:
                    self.dataset = name
                    self.output_dir = self._resolve_output_dir()
            if new != self.rec_context:
                self.rec_context = new
                try:
                    path = self._context_path()
                    path.parent.mkdir(parents=True, exist_ok=True)
                    with open(path, 'w') as f:
                        json.dump(new, f, indent=2)
                except OSError as e:
                    self.get_logger().warn(f'Recording context not saved: {e}')
            self.get_logger().info(
                f'Recording context: task {new["task_id"] or "(not set)"}, {new["source"]}, '
                f'round {new["round"]}, operator {new["operator"] or "(not set)"}, '
                f'session {self.dataset or "(today\'s)"}')
        self._publish_context()

    def _takeover_callback(self, msg: String):
        try:
            event = json.loads(msg.data)
            event['stamp_ns'] = int(event['stamp_ns'])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'Unreadable takeover event: {msg.data!r}')
            return
        self.last_takeover = event
        self._apply_takeover()

    def _apply_takeover(self):
        """Turn the episode being recorded into a correction if a takeover started it."""
        rec, event = self.episode_record, self.last_takeover
        if rec is None or event is None or rec.get('takeover_stamp_ns') == event['stamp_ns']:
            return
        if not (-TAKEOVER_WINDOW_S <= (event['stamp_ns'] - rec['start_ns']) / 1e9 <= self.max_duration):
            return
        text = str(event.get('task') or '')
        reg = self._registry()
        task = reg.by_text(text) if reg else None
        task_id = str(event.get('task_id') or '') or (task['id'] if task else '')
        rec.update(
            source='correction',
            round=self.rec_context['round'],
            takeover_stamp_ns=event['stamp_ns'],
            instruction_sent=text,          # exact string the policy was running on
            takeover_s=event.get('takeover_s'),   # seconds into the policy run
            policy_ckpt=event.get('policy_ckpt') or None,
            run_id=event.get('run_id') or None,
        )
        if task_id:
            rec['task_id'] = task_id
        else:
            self.get_logger().warn(
                f'Takeover task "{text}" is not in the task registry; '
                f'episode keeps task {rec["task_id"] or "(not set)"}')
        if not self.rec_context['objects']:
            rec['objects'] = self._task_objects(rec['task_id'], reg)

    def _workspace_commit(self):
        """HEAD of the alfiebot checkout (the source tree, which the running
        build normally matches), or None."""
        try:
            out = subprocess.run(['git', '-C', self.workspace_dir, 'rev-parse', 'HEAD'],
                                 capture_output=True, text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return out.stdout.strip() if out.returncode == 0 else None

    @staticmethod
    def _task_objects(task_id, reg):
        task = reg.by_id(task_id) if reg and task_id else None
        if task is None:
            return []
        return [o for o in (task.get('object'), task.get('destination')) if o]

    def _new_episode_record(self, bag_name: str) -> dict:
        ctx = self.rec_context
        demo = ctx['source'] == 'demo'
        return {
            'bag': bag_name,
            'session': self.dataset,
            'hw': self._hw(),
            'recorded_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
            'start_ns': self.get_clock().now().nanoseconds,
            'task_id': ctx['task_id'],
            'source': ctx['source'],
            'round': 0 if demo else ctx['round'],
            'policy_ckpt': None,
            'run_id': None,
            'instruction_sent': None,
            'takeover_s': None,
            'failure_mode': None,      # corrections: tagged during review
            'operator': ctx['operator'],
            'objects': list(ctx['objects']) or self._task_objects(ctx['task_id'], self._registry()),
            'surface': ctx['surface'],
            'quality': 'ok',
            'notes': '',
            'alfiebot_commit': self._workspace_commit(),
        }

    def _write_episode_record(self, bag_path: str, reason: str):
        """episode.json in the bag folder (rosbag2 creates the folder itself)."""
        rec = self.episode_record
        self.episode_record = None
        if rec is None or not os.path.isdir(bag_path):
            return
        rec['stop_reason'] = reason
        try:
            with open(os.path.join(bag_path, EPISODE_FILE), 'w') as f:
                json.dump(rec, f, indent=2)
                f.write('\n')
        except OSError as e:
            self.get_logger().error(f'Could not write {EPISODE_FILE} for {bag_path}: {e}')

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
            if '/' in name or name in ('.', '..'):
                return SetParametersResult(
                    successful=False,
                    reason="dataset must be a plain folder name (no '/'), or '' "
                           "for today's session")
            self.dataset = name
        if 'data_root' in path_params:
            self.data_root = str(path_params['data_root'])
        if 'output_dir' in path_params:
            self.output_dir_override = str(path_params['output_dir'])

        self.output_dir = self._resolve_output_dir()
        self.get_logger().info(
            f'Session switched to: {self.dataset or "(today\'s)"}  '
            f'Output dir: {self.output_dir or "(chosen on first recording)"}')
        self._publish_context()
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

        if not self._choose_session():
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
            self.episode_record = self._new_episode_record(bag_name)
            self._apply_takeover()
            if not self.episode_record['task_id']:
                self.get_logger().warn(
                    'No task chosen for this episode; pick one in the episode manager '
                    '(it can be labelled after conversion)')

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
            self.episode_record = None

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
            self.episode_record = None
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

                self._write_episode_record(bag_path, reason)
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
            self.episode_record = None

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
