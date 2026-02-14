#!/usr/bin/env python3
"""GR00T N1.6 inference client node for Alfiebot.

This node connects to a remote GR00T inference server via ZeroMQ,
sends synchronized observations, and publishes action predictions
to control the robot.

Key timing:
- Inference runs continuously on a background thread (as fast as the server allows)
- The server returns a 16-step action horizon; the command loop steps through
  these actions at the training data rate (15 FPS = 67ms per action)
- Command publishing runs at exactly 100 Hz on the ROS2 executor
- When a new action chunk arrives, it immediately replaces the current one
- EMA smoothing at 100 Hz bridges transitions between chunks
"""

from enum import Enum
import threading
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from alfie_msgs.msg import BackCmd, RobotLowCmd, RobotLowState
from alfie_msgs.srv import BackRequestCalibration

from ..core.action_publisher import ActionPublisher
from ..core.observation_bridge import Observation, ObservationBridge
from ..core.zmq_client import ZMQClient, build_server_address, DEFAULT_IPC_PATH
from ..utils.safety import SafetyMonitor


# Command publishing rate (Hz) - must match robot expectations
COMMAND_RATE_HZ = 100

# Training data collection rate (Hz) - action horizon step period
TRAINING_FPS = 15
ACTION_STEP_PERIOD = 1.0 / TRAINING_FPS  # ~67ms per action step


class ClientState(Enum):
    """GR00T client state machine states."""

    IDLE = 0        # Waiting for activationsrc/alfie_asr/alfie_asr
    CONNECTING = 1  # Attempting to connect to server
    ACTIVE = 2      # Running inference loop
    E_STOP = 3      # Emergency stop triggered
    ERROR = 4       # Error state (connection lost, etc.)


class GrootClientNode(Node):
    """Main GR00T inference client ROS2 node.

    Orchestrates:
    - Observation collection from cameras and robot state
    - ZeroMQ communication with inference server (background thread)
    - Action chunk execution at training rate (15 FPS)
    - Action publishing to robot (100 Hz on ROS2 executor)
    - Safety monitoring and state management

    Normalization contract:
    - Raw state is sent to the server; Gr00tPolicy normalizes internally
    - Server returns raw/unnormalized actions; no denormalization needed

    Action chunking:
    - Server returns 16 actions per inference call (the action horizon)
    - The 100 Hz command timer steps through them at 67ms each (15 FPS training rate)
    - Inference runs continuously; each new chunk immediately replaces the old one
    - With ~300ms inference, typically ~4-5 actions execute before a fresh chunk arrives
    - Actions 5-15 serve as a buffer when inference is occasionally slow

    Threading model:
    - Inference runs continuously on a dedicated background thread.
      The blocking ZMQ call (~300ms) never stalls the ROS2 executor.
    - Command publishing runs at 100 Hz on the single-threaded ROS2 executor.
    - _action_lock protects the shared action chunk between threads.
    """

    def __init__(self):
        super().__init__('groot_client')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        self.transport = self.get_parameter('transport').value
        self.server_host = self.get_parameter('server_host').value
        self.server_port = self.get_parameter('server_port').value
        self.ipc_path = self.get_parameter('ipc_path').value
        self.target_fps = self.get_parameter('target_fps').value
        self.inference_timeout_ms = self.get_parameter('inference_timeout_ms').value
        self.task_description = self.get_parameter('task_description').value
        self.enable_safety_limits = self.get_parameter('enable_safety_limits').value
        self.action_smoothing_alpha = self.get_parameter('action_smoothing_alpha').value
        self.action_chunk_enabled = self.get_parameter('action_chunk_enabled').value
        self.action_chunk_size = self.get_parameter('action_chunk_size').value
        self.csv_log_path = self.get_parameter('csv_log_path').value
        self.base_velocity_decay = self.get_parameter('base_velocity_decay').value
        self.max_joint_delta = self.get_parameter('max_joint_delta').value
        self.debug_save_images = self.get_parameter('debug_save_images').value
        self.h264_conditioning = self.get_parameter('h264_conditioning').value
        self.latency_skip_actions = self.get_parameter('latency_skip_actions').value
        self.chunk_blend_actions = self.get_parameter('chunk_blend_actions').value
        self.interpolate_actions = self.get_parameter('interpolate_actions').value
        self.back_init_height = self.get_parameter('back_init_height').value

        # Build server address from transport parameters
        self.server_address = build_server_address(
            transport=self.transport,
            host=self.server_host,
            port=self.server_port,
            ipc_path=self.ipc_path,
        )

        # Initialize state
        self._state = ClientState.IDLE
        self._active = False
        self._running = True  # Controls inference thread lifetime

        # Action chunk state (protected by lock for thread safety)
        # The inference thread writes a full chunk; the command timer reads it.
        # _pending_chunk buffers the next chunk until the current one is
        # fully consumed, matching open-loop eval behavior where each
        # 16-action horizon plays to completion before the next inference.
        self._action_chunk: Optional[np.ndarray] = None  # shape (N, 22)
        self._pending_chunk: Optional[np.ndarray] = None # next chunk, waiting to be promoted
        self._chunk_state: Optional[np.ndarray] = None   # state when chunk arrived
        self._chunk_timestamp: float = 0.0               # time.monotonic() when chunk arrived
        self._prev_chunk_tail: Optional[np.ndarray] = None  # tail of old chunk for blending
        self._action_lock = threading.Lock()
        self._total_chunks = 0                            # for diagnostic logging

        # Initialize safety monitor
        # Watchdog timeout must exceed the chunk duration (16 actions * 67ms ≈ 1.07s)
        # plus inference latency (~130ms), since inference only runs once per chunk.
        self.safety = SafetyMonitor(
            watchdog_timeout=2.0,
            max_consecutive_failures=5,
        )

        # Initialize ZMQ client
        self.zmq_client = ZMQClient(
            server_address=self.server_address,
            timeout_ms=self.inference_timeout_ms,
            logger=lambda msg: self.get_logger().info(msg),
        )

        # Initialize observation bridge
        self.observation_bridge = ObservationBridge(
            node=self,
            debug_save_images=self.debug_save_images,
            h264_conditioning=self.h264_conditioning,
        )

        # Initialize action publisher
        self.action_publisher = ActionPublisher(
            node=self,
            safety=self.safety if self.enable_safety_limits else None,
            smoothing_alpha=self.action_smoothing_alpha,
            csv_log_path=self.csv_log_path,
        )

        # QoS for control topics
        qos_control = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Activation subscription
        self.activate_sub = self.create_subscription(
            Bool,
            '~/activate',
            self._activate_callback,
            qos_control,
        )

        # E-stop subscription
        self.estop_sub = self.create_subscription(
            Bool,
            '~/estop',
            self._estop_callback,
            qos_control,
        )

        # Task description subscription (dynamic updates)
        self.task_sub = self.create_subscription(
            String,
            '~/task_description',
            self._task_callback,
            qos_control,
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '~/status', qos_control)

        # Command publishing timer (100 Hz - sends commands to robot)
        # Runs on the ROS2 executor, never blocked by inference
        self.command_timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ,
            self._command_callback,
        )

        # Status publishing timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # Print parameter values on startup
        self._log_parameters()

        # Test server connection
        self._test_server_connection()

        # Initialize back (calibrate if needed, move to target height)
        if self.back_init_height >= 0.0:
            self._initialize_back()

        # Start inference on a background thread (after all setup is complete)
        self._inference_thread = threading.Thread(
            target=self._inference_loop,
            name='groot_inference',
            daemon=True,
        )
        self._inference_thread.start()

    def _log_parameters(self):
        """Log all parameter values on startup."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('GR00T Client Parameters:')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Transport:            {self.transport}')
        self.get_logger().info(f'  Server Address:       {self.server_address}')
        if self.transport == 'tcp':
            self.get_logger().info(f'    Host:               {self.server_host}')
            self.get_logger().info(f'    Port:               {self.server_port}')
        else:
            self.get_logger().info(f'    IPC Path:           {self.ipc_path}')
        self.get_logger().info(f'  Target FPS:           {self.target_fps}')
        self.get_logger().info(f'  Command Rate:         {COMMAND_RATE_HZ} Hz')
        self.get_logger().info(f'  Inference Timeout:    {self.inference_timeout_ms} ms')
        self.get_logger().info(f'  Task Description:     "{self.task_description}"')
        self.get_logger().info(f'  Safety Limits:        {self.enable_safety_limits}')
        self.get_logger().info(f'  Action Smoothing:     {self.action_smoothing_alpha}')
        self.get_logger().info(f'  Action Chunking:      {self.action_chunk_enabled}')
        self.get_logger().info(f'  Action Chunk Size:    {self.action_chunk_size}')
        self.get_logger().info(f'  Action Step Period:   {ACTION_STEP_PERIOD * 1000:.1f} ms ({TRAINING_FPS} FPS)')
        self.get_logger().info(f'  CSV Log Path:         {self.csv_log_path or "(disabled)"}')
        self.get_logger().info(f'  Base Vel Decay:       {self.base_velocity_decay}')
        self.get_logger().info(f'  Max Joint Delta:      {self.max_joint_delta} rad')
        self.get_logger().info(f'  Debug Save Images:    {self.debug_save_images}')
        self.get_logger().info(f'  H.264 Conditioning:   {self.h264_conditioning}')
        self.get_logger().info(f'  Back Init Height:     {self.back_init_height} m')
        self.get_logger().info(f'  Latency Skip:         {self.latency_skip_actions} actions ({self.latency_skip_actions * ACTION_STEP_PERIOD * 1000:.0f} ms)')
        self.get_logger().info(f'  Chunk Blend:          {self.chunk_blend_actions} actions ({self.chunk_blend_actions * ACTION_STEP_PERIOD * 1000:.0f} ms)')
        self.get_logger().info(f'  Interpolate Actions:  {self.interpolate_actions}')
        self.get_logger().info('=' * 60)

    def _test_server_connection(self):
        """Test connection to the inference server on startup."""
        self.get_logger().info(f'Testing connection to server at {self.server_address}...')

        if self.zmq_client.connect():
            self.get_logger().info('Server connection successful!')
            # Disconnect after test - will reconnect when activated
            self.zmq_client.close()
        else:
            self.get_logger().warn(
                f'Could not connect to server at {self.server_address}. '
                'Server may not be running. Will retry on activation.'
            )

    def _initialize_back(self):
        """Calibrate the back if needed and move it to back_init_height.

        Continuously publishes back commands at 100Hz until the back reaches
        the target height (within tolerance), or a timeout expires.

        Uses spin_once to pump the executor since this runs during __init__
        before the main spin loop starts.
        """
        POSITION_TOLERANCE = 0.005  # 5mm
        MOVE_TIMEOUT = 15.0  # seconds to reach target height

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Shared state updated by subscription callback
        latest_state = [None]

        def _state_cb(msg):
            latest_state[0] = msg

        sub = self.create_subscription(RobotLowState, '/alfie/robotlowstate', _state_cb, qos)

        # Wait for first RobotLowState message
        self.get_logger().info('Waiting for robot state to check back calibration...')
        deadline = time.monotonic() + 10.0
        while latest_state[0] is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if latest_state[0] is None:
            self.get_logger().warn('Timed out waiting for robot state — skipping back init')
            self.destroy_subscription(sub)
            return

        robot_state = latest_state[0]

        # Calibrate if needed
        if not robot_state.back_state.is_calibrated:
            self.get_logger().info('Back not calibrated, calling calibration service...')
            calibrate_client = self.create_client(
                BackRequestCalibration, '/alfie/low/calibrate_back'
            )
            if calibrate_client.wait_for_service(timeout_sec=5.0):
                request = BackRequestCalibration.Request()
                future = calibrate_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                if future.result() is not None:
                    if future.result().success:
                        self.get_logger().info('Back calibration successful')
                    else:
                        self.get_logger().warn('Back calibration returned failure')
                        self.destroy_subscription(sub)
                        return
                else:
                    self.get_logger().warn('Back calibration service call failed')
                    self.destroy_subscription(sub)
                    return
            else:
                self.get_logger().warn('Back calibration service not available')
                self.destroy_subscription(sub)
                return
        else:
            self.get_logger().info('Back already calibrated')

        # Move to target height — publish at 100Hz until position is reached
        target = self.back_init_height
        self.get_logger().info(f'Moving back to {target:.3f} m ...')

        cmd = RobotLowCmd()
        cmd.back_cmd = BackCmd()
        cmd.back_cmd.position = target
        cmd.back_cmd.velocity = 0.2
        cmd.back_cmd.acceleration = 0.1

        move_deadline = time.monotonic() + MOVE_TIMEOUT
        reached = False

        while time.monotonic() < move_deadline:
            # Pump executor to receive fresh state messages
            rclpy.spin_once(self, timeout_sec=0.0)

            # Publish command at ~100Hz
            self.action_publisher.cmd_pub.publish(cmd)

            # Check current position
            if latest_state[0] is not None:
                current_pos = latest_state[0].back_state.current_position
                if abs(current_pos - target) <= POSITION_TOLERANCE:
                    reached = True
                    break

            time.sleep(0.01)  # 100Hz

        self.destroy_subscription(sub)

        if reached:
            self.get_logger().info(
                f'Back reached target height {target:.3f} m '
                f'(current: {latest_state[0].back_state.current_position:.3f} m)'
            )
        else:
            current_pos = latest_state[0].back_state.current_position if latest_state[0] else float('nan')
            self.get_logger().warn(
                f'Back did not reach target {target:.3f} m within {MOVE_TIMEOUT:.0f}s '
                f'(current: {current_pos:.3f} m)'
            )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        # Transport configuration
        # Use "ipc" for on-device inference (faster), "tcp" for remote server
        self.declare_parameter('transport', 'tcp')
        self.declare_parameter('server_host', '192.168.50.108')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('ipc_path', DEFAULT_IPC_PATH)

        # Inference settings
        self.declare_parameter('target_fps', 15)
        self.declare_parameter('inference_timeout_ms', 100)
        self.declare_parameter('task_description', 'find the can and pick it up')
        self.declare_parameter('enable_safety_limits', True)
        self.declare_parameter('action_smoothing_alpha', 0.5)
        self.declare_parameter('action_chunk_enabled', True)
        self.declare_parameter('action_chunk_size', 16)
        self.declare_parameter('csv_log_path', '')

        # Base velocity drift correction.
        # Decay factor (0-1) multiplied into base velocity each command cycle.
        # Can help dampen small prediction noise in base velocity output.
        # 0.0 = no correction (default), 0.95 = gentle decay toward zero.
        self.declare_parameter('base_velocity_decay', 0.0)

        # Maximum allowed joint position change per command cycle (radians).
        # Safety backstop for delta limiting — servos also self-limit via
        # target_speed/acceleration, so this is a secondary safeguard.
        self.declare_parameter('max_joint_delta', 0.5)

        # Debug: save first few observation images to disk for visual comparison
        # with training data. Images saved to /tmp/groot_debug_images/
        self.declare_parameter('debug_save_images', False)

        # H.264 conditioning: apply libx264 yuv420p CRF=23 encode/decode to
        # live camera frames so they match the training data pipeline
        # (rosbag_to_groot.py encodes to MP4 with these exact settings).
        self.declare_parameter('h264_conditioning', False)

        # Latency compensation: skip first N actions from new chunks to
        # account for observation-to-action delay (~160ms at 67ms/action).
        # 2 = skip 134ms (good starting point). 0 = disabled.
        self.declare_parameter('latency_skip_actions', 2)

        # Chunk blending: crossfade window (in actions) at chunk boundaries.
        # Prevents discontinuities that cause hand/gripper twitching.
        # 4 actions = ~268ms blend. 0 = disabled (hard switch).
        self.declare_parameter('chunk_blend_actions', 4)

        # Inter-action interpolation: linearly interpolate between
        # consecutive actions in the chunk for smooth 100Hz output.
        self.declare_parameter('interpolate_actions', True)

        # Back initialization height (meters). On startup, the back is
        # calibrated if needed and moved to this position before inference.
        # Set to -1.0 to disable back initialization entirely.
        self.declare_parameter('back_init_height', 0.1)

    def _activate_callback(self, msg: Bool):
        """Handle activation/deactivation requests."""
        if msg.data:
            self._activate()
        else:
            self._deactivate()

    def _estop_callback(self, msg: Bool):
        """Handle E-stop signals."""
        if msg.data:
            self._trigger_estop()
        else:
            self._reset_estop()

    def _task_callback(self, msg: String):
        """Handle dynamic task description updates."""
        self.task_description = msg.data
        self.get_logger().info(f'Task updated: "{self.task_description}"')

    def _activate(self):
        """Activate inference loop."""
        if self._state == ClientState.E_STOP:
            self.get_logger().warn('Cannot activate while E-stop is active')
            return

        if self._state == ClientState.ACTIVE:
            return

        self.get_logger().info('Activating GR00T inference...')
        self._state = ClientState.CONNECTING

        # Attempt connection
        if self.zmq_client.connect():
            self._state = ClientState.ACTIVE
            self._active = True
            self.get_logger().info('GR00T inference active')

            # Check starting pose against training data distribution
            self._check_starting_pose()
        else:
            self._state = ClientState.ERROR
            self.get_logger().error('Failed to connect to inference server')

    def _deactivate(self):
        """Deactivate inference loop."""
        if self._state == ClientState.IDLE:
            return

        self.get_logger().info('Deactivating GR00T inference...')
        self._active = False
        self._state = ClientState.IDLE

        # Clear action chunk
        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None
            self._chunk_state = None
            self._prev_chunk_tail = None

        # Send stop command
        self.action_publisher.publish_stop()

        # Reset smoothing
        self.action_publisher.reset_smoothing()

    def _trigger_estop(self):
        """Trigger emergency stop."""
        self.get_logger().warn('E-STOP triggered!')
        self._state = ClientState.E_STOP
        self._active = False
        self.safety.e_stop_active = True

        # Clear action chunk
        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None
            self._chunk_state = None
            self._prev_chunk_tail = None

        # Send stop command
        self.action_publisher.publish_stop()

    def _reset_estop(self):
        """Reset emergency stop."""
        if self._state != ClientState.E_STOP:
            return

        self.get_logger().info('E-STOP reset')
        self.safety.e_stop_active = False
        self.safety.reset_failures()
        self._state = ClientState.IDLE

        # Clear action chunk
        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None
            self._chunk_state = None
            self._prev_chunk_tail = None

        # Reset smoothing
        self.action_publisher.reset_smoothing()

    def _check_starting_pose(self):
        """Check if robot starting pose is within training data distribution.

        Logs warnings for joints significantly out of range.
        """
        # Expected STARTING values from training episode t=0 analysis
        # (not mid-task averages — these are the poses when demos begin)
        # Format: state_dim -> (name, expected_value, tolerance)
        EXPECTED_START = {
            6:  ('back_joint', 0.098, 0.02),
            9:  ('left_elbow_pitch', -1.477, 0.15),
            19: ('head_yaw', 0.0, 0.5),
            20: ('head_pitch', -0.04, 0.3),
            21: ('head_roll', 0.06, 0.15),
        }

        obs = self.observation_bridge.get_latest_observation()
        if obs is None or not obs.valid:
            self.get_logger().warn('Pose check: no valid observation available')
            return

        mismatches = []
        for idx, (name, expected, tol) in EXPECTED_START.items():
            actual = obs.state[idx]
            if abs(actual - expected) > tol:
                mismatches.append(
                    f'{name}={actual:.3f} (expected ~{expected:.3f}, off by {actual - expected:+.3f})'
                )

        if mismatches:
            self.get_logger().warn(
                f'Starting pose mismatches ({len(mismatches)} joints): '
                + ', '.join(mismatches)
            )
        else:
            self.get_logger().info('Starting pose check: all joints within training range')

    def _inference_loop(self):
        """Background inference loop (runs on dedicated thread).

        Waits until the current action chunk is nearly consumed before
        requesting a new one. This ensures each 16-step action horizon
        actually plays out — matching how open-loop eval steps through
        the full horizon before calling inference again.

        Without this, ~130ms inference latency means chunks get replaced
        after only ~2 actions execute, causing stuttering/wiggling.
        """
        # How long a chunk takes to execute at the training rate.
        # With latency skip, the effective trajectory is shorter because
        # we start reading from action[skip] — the last `skip` actions
        # worth of time would just hold the final position.
        effective_actions = self.action_chunk_size - self.latency_skip_actions
        chunk_duration = effective_actions * ACTION_STEP_PERIOD  # (16-2) * 67ms ≈ 0.93s

        while self._running:
            # Idle-poll when not active
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            # Wait until the current chunk is nearly consumed before
            # requesting a new one. This lets the model's full trajectory
            # play out instead of replacing it after 1-2 actions.
            with self._action_lock:
                chunk_ts = self._chunk_timestamp
                has_chunk = self._action_chunk is not None

            if has_chunk:
                elapsed = time.monotonic() - chunk_ts
                remaining = chunk_duration - elapsed
                # Wait until the chunk has FULLY played out before capturing
                # a new observation. This ensures the model sees the RESULT
                # of its 16-action plan, not an in-progress state. Without
                # this, the model sees a mid-trajectory state and re-plans
                # from there, creating overlapping/oscillating trajectories.
                # The ~130ms inference gap is covered by the velocity-zeroing
                # and joint-holding in _command_callback.
                if remaining > 0:
                    time.sleep(min(remaining, 0.05))
                    continue

            # Get latest observation
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            # Send raw state — Gr00tPolicy normalizes internally
            # This is the blocking call (~130ms) that motivated the thread
            response = self.zmq_client.send_observation(
                images=obs.images,
                state=obs.state,
                language=self.task_description,
            )

            if response is None:
                # Inference failed
                self.safety.record_failure()

                if self.safety.in_safe_mode:
                    self.get_logger().error('Too many failures, entering safe mode')
                    self._state = ClientState.ERROR
                    self._active = False
                    with self._action_lock:
                        self._action_chunk = None
                        self._prev_chunk_tail = None
                continue

            # Update safety watchdog
            self.safety.update_inference_time()

            # Extract action chunk from response
            chunk = self._extract_action_chunk(response)
            if chunk is None:
                self.get_logger().warn('Invalid action response from server')
                continue

            # Log chunk timing and diagnostics
            with self._action_lock:
                old_ts = self._chunk_timestamp
            gap = time.monotonic() - old_ts if has_chunk else 0.0
            self.get_logger().info(
                f'[chunk] new chunk after {gap:.3f}s '
                f'({gap/ACTION_STEP_PERIOD:.1f} actions consumed), '
                f'state_base={np.array2string(obs.state[0:6], precision=4, suppress_small=True)}, '
                f'action_base={np.array2string(chunk[0, 0:6], precision=4, suppress_small=True)}'
            )

            # Dump full chunk trajectory for first 3 chunks to diagnose
            # whether the model produces temporal structure within a chunk
            if self._total_chunks < 3:
                self.get_logger().info(f'[chunk_dump] state: {np.array2string(obs.state, precision=3, suppress_small=True)}')
                for i in range(len(chunk)):
                    a = chunk[i]
                    self.get_logger().info(
                        f'[chunk_dump] action[{i:2d}]: '
                        f'base=({a[0]:+.4f},{a[1]:+.4f},{a[5]:+.4f}) '
                        f'back={a[6]:.3f} '
                        f'r_arm=({a[13]:+.3f},{a[14]:+.3f},{a[15]:+.3f}) '
                        f'r_grip={a[18]:.3f} '
                        f'head=({a[19]:+.3f},{a[20]:+.3f},{a[21]:+.3f})'
                    )
            self._total_chunks += 1

            # Buffer the new chunk — the command callback will promote it
            # once the current chunk is fully consumed, matching open-loop
            # eval behavior (full 16-action horizon before next inference).
            with self._action_lock:
                if self._action_chunk is None:
                    # No current chunk — install immediately (first chunk)
                    self._action_chunk = chunk
                    self._chunk_state = obs.state.copy()
                    self._chunk_timestamp = time.monotonic()
                else:
                    # Buffer until current chunk finishes
                    self._pending_chunk = chunk

    def _command_callback(self):
        """Command publishing callback (runs at exactly 100 Hz).

        Steps through the action chunk at the training data rate (67ms per
        action) with three smoothing mechanisms:

        1. **Latency skip**: When a new chunk is promoted, skip the first N
           actions to compensate for the ~160ms observation-to-action delay.
           This prevents overshoot during approach.

        2. **Chunk blending**: Crossfade between the tail of the old chunk
           and the head of the new chunk over a configurable window. This
           prevents twitching at chunk boundaries.

        3. **Inter-action interpolation**: Linearly interpolate between
           consecutive actions for smooth 100Hz output instead of holding
           each 67ms action constant.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        now = time.monotonic()

        # Check if current chunk is exhausted and a pending chunk is ready
        with self._action_lock:
            chunk = self._action_chunk
            timestamp = self._chunk_timestamp
            pending = self._pending_chunk

            if chunk is not None and pending is not None:
                elapsed = now - timestamp
                effective_len = len(chunk) - self.latency_skip_actions
                chunk_duration = effective_len * ACTION_STEP_PERIOD
                if elapsed >= chunk_duration:
                    # Save tail of old chunk for blending
                    blend_n = self.chunk_blend_actions
                    if blend_n > 0:
                        self._prev_chunk_tail = chunk[-blend_n:].copy()
                    else:
                        self._prev_chunk_tail = None

                    # Store full chunk — latency skip is applied per-body-part
                    # in the action selection below (base velocity is offset
                    # by skip actions to compensate for observation delay,
                    # but joints use the full trajectory from action[0]).
                    self._action_chunk = pending
                    self._pending_chunk = None
                    self._chunk_timestamp = now
                    # Re-read after promotion
                    chunk = self._action_chunk
                    timestamp = self._chunk_timestamp

        if chunk is None:
            return

        # Compute fractional position within the chunk.
        # With latency skip, the effective playable actions are (chunk_size - skip),
        # so the chunk is exhausted earlier.
        if self.action_chunk_enabled:
            elapsed = now - timestamp
            skip = self.latency_skip_actions
            effective_len = len(chunk) - skip
            t_frac = elapsed / ACTION_STEP_PERIOD  # fractional action index
            chunk_exhausted = t_frac >= effective_len
            idx = min(int(t_frac), effective_len - 1)
        else:
            t_frac = 0.0
            chunk_exhausted = False
            idx = 0

        # Latency skip: offset the read index into the chunk to compensate
        # for the ~160ms observation-to-action delay. The model planned its
        # trajectory from where the robot was 160ms ago — skipping 2 actions
        # (134ms) starts execution from roughly where the robot actually is.
        # Applied to ALL dimensions uniformly (base velocity + joint positions).
        # This prevents the grab-release-grab pattern where the new chunk's
        # action[0] targets a more-open gripper position than the robot has
        # already reached during the 130ms inference gap.
        skip = self.latency_skip_actions
        skipped_frac = t_frac + skip
        skipped_idx = min(int(skipped_frac), len(chunk) - 1)

        # Inter-action interpolation: lerp between consecutive actions
        if self.interpolate_actions and skipped_idx < len(chunk) - 1:
            alpha = skipped_frac - int(skipped_frac)  # fractional part [0, 1)
            action = (1.0 - alpha) * chunk[skipped_idx] + alpha * chunk[skipped_idx + 1]
        else:
            action = chunk[skipped_idx].copy()

        # When the chunk is exhausted (waiting for next inference), zero
        # base velocity but hold joint positions. Velocity commands persist
        # at 100Hz — holding the last velocity means the robot keeps driving
        # indefinitely, causing overshoot. Joint positions are ABSOLUTE so
        # holding them just keeps the servos in place.
        if chunk_exhausted:
            action[0:6] = 0.0

        # Chunk boundary blending: crossfade from old chunk tail to new
        blend_n = self.chunk_blend_actions
        if blend_n > 0 and self._prev_chunk_tail is not None and t_frac < blend_n:
            # Weight ramps from ~0 (old chunk) to 1 (new chunk)
            w_new = min((t_frac + 0.5) / blend_n, 1.0)
            old_idx = min(idx, len(self._prev_chunk_tail) - 1)
            old_action = self._prev_chunk_tail[old_idx]
            action = w_new * action + (1.0 - w_new) * old_action

        # Clear blend state once past the blend window
        if self._prev_chunk_tail is not None and t_frac >= blend_n:
            self._prev_chunk_tail = None

        # Apply base velocity drift correction.
        if self.base_velocity_decay > 0.0:
            action[0:6] *= (1.0 - self.base_velocity_decay)

        # Publish action to robot at 100 Hz
        self.action_publisher.publish_action(
            action=action,
            apply_smoothing=True,
            apply_safety=self.enable_safety_limits,
        )

    def _extract_action_chunk(self, response: dict) -> Optional[np.ndarray]:
        """Extract action chunk from server response.

        Args:
            response: Server response dictionary with 'actions' key.

        Returns:
            Action chunk as (N, 22) ndarray, or None on error.
            N is min(action_chunk_size, available actions).
        """
        if 'status' in response and response['status'] != 'ok':
            self.get_logger().warn(f"Server error: {response.get('error_message', 'unknown')}")
            return None

        if 'actions' not in response:
            return None

        actions = response['actions']

        # actions is a list of 16 action vectors (action horizon)
        if not isinstance(actions, list) or len(actions) == 0:
            return None

        # Take first N actions based on chunk size
        n = min(self.action_chunk_size, len(actions))
        return np.array(actions[:n], dtype=np.float32)

    def _publish_status(self):
        """Publish current status."""
        zmq_stats = self.zmq_client.get_stats()
        obs_stats = self.observation_bridge.get_stats()
        action_stats = self.action_publisher.get_stats()
        safety_stats = self.safety.get_status()

        status = {
            'state': self._state.name,
            'active': self._active,
            'task': self.task_description,
            'zmq': zmq_stats,
            'observation': obs_stats,
            'action': action_stats,
            'safety': safety_stats,
        }

        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources on shutdown."""
        self.get_logger().info('Shutting down GR00T client...')

        # Stop inference thread
        self._running = False
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=2.0)

        # Send stop command
        self.action_publisher.publish_stop()

        # Close CSV log
        self.action_publisher.close_csv()

        # Close ZMQ connection
        self.zmq_client.close()

        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GrootClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
