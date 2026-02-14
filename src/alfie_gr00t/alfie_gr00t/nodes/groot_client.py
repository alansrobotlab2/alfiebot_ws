#!/usr/bin/env python3
"""GR00T N1.6 inference client node for Alfiebot.

This node connects to a remote GR00T inference server via ZeroMQ,
sends synchronized observations, and publishes action predictions
to control the robot.

Key timing:
- Server returns a 16-step action horizon; the command loop steps through
  the first `n_action_steps` at the training data rate (15 FPS = 67ms/action)
- After n_action_steps are consumed, a fresh observation is captured and
  a new inference request is sent
- Command publishing runs at exactly 100 Hz on the ROS2 executor
- Inter-action interpolation smooths 15 FPS actions to 100 Hz output

Threading model:
- Inference runs on a dedicated background thread (blocking ZMQ ~130ms)
- Command publishing runs at 100 Hz on the single-threaded ROS2 executor
- _action_lock protects the shared action chunk between threads
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

    IDLE = 0        # Waiting for activation
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

    Action chunking (n_action_steps):
    - Server returns 16 actions per inference call (the action_horizon)
    - n_action_steps controls how many are EXECUTED before re-querying
    - n_action_steps=16: execute full chunk (current behavior, least responsive)
    - n_action_steps=8: execute half, re-query with fresh observation (more responsive)
    - The 100 Hz command timer steps through actions at 67ms each (training rate)
    - After n_action_steps consumed, zero base velocity, hold joints, request new chunk
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
        self.inference_timeout_ms = self.get_parameter('inference_timeout_ms').value
        self.task_description = self.get_parameter('task_description').value
        self.enable_safety_limits = self.get_parameter('enable_safety_limits').value
        self.action_smoothing_alpha = self.get_parameter('action_smoothing_alpha').value
        self.action_chunk_size = self.get_parameter('action_chunk_size').value
        self.n_action_steps = self.get_parameter('n_action_steps').value
        self.csv_log_path = self.get_parameter('csv_log_path').value
        self.base_velocity_decay = self.get_parameter('base_velocity_decay').value
        self.latency_skip_base = self.get_parameter('latency_skip_base').value
        self.max_joint_delta = self.get_parameter('max_joint_delta').value
        self.debug_save_images = self.get_parameter('debug_save_images').value
        self.h264_conditioning = self.get_parameter('h264_conditioning').value
        self.interpolate_actions = self.get_parameter('interpolate_actions').value
        self.back_init_height = self.get_parameter('back_init_height').value

        # Validate n_action_steps
        if self.n_action_steps < 1 or self.n_action_steps > self.action_chunk_size:
            self.get_logger().warn(
                f'n_action_steps={self.n_action_steps} out of range [1, {self.action_chunk_size}], '
                f'clamping to {self.action_chunk_size}'
            )
            self.n_action_steps = min(max(self.n_action_steps, 1), self.action_chunk_size)

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
        # fully consumed (n_action_steps executed).
        self._action_chunk: Optional[np.ndarray] = None  # shape (N, 22)
        self._pending_chunk: Optional[np.ndarray] = None  # next chunk, waiting
        self._chunk_timestamp: float = 0.0                # time.monotonic() when chunk started
        self._action_lock = threading.Lock()
        self._total_chunks = 0                            # for diagnostic logging

        # Chunk transition blending: when a new chunk starts, its action[0]
        # may not match where the joints ended up from the previous chunk.
        # We blend joints (not base) from the old chunk's last action to the
        # new chunk's trajectory over BLEND_STEPS actions to avoid snapping.
        BLEND_STEPS = 3  # blend over ~200ms (3 actions at 67ms each)
        self._blend_steps = BLEND_STEPS
        self._blend_from: Optional[np.ndarray] = None  # last action of previous chunk (22D)

        # Initialize safety monitor
        # Watchdog timeout must exceed the chunk execution time plus inference latency
        execution_time = self.n_action_steps * ACTION_STEP_PERIOD
        self.safety = SafetyMonitor(
            watchdog_timeout=execution_time + 1.0,
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
        execution_time_ms = self.n_action_steps * ACTION_STEP_PERIOD * 1000
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
        self.get_logger().info(f'  Command Rate:         {COMMAND_RATE_HZ} Hz')
        self.get_logger().info(f'  Inference Timeout:    {self.inference_timeout_ms} ms')
        self.get_logger().info(f'  Task Description:     "{self.task_description}"')
        self.get_logger().info(f'  Safety Limits:        {self.enable_safety_limits}')
        self.get_logger().info(f'  Action Smoothing:     {self.action_smoothing_alpha}')
        self.get_logger().info(f'  Action Chunk Size:    {self.action_chunk_size}')
        self.get_logger().info(f'  n_action_steps:       {self.n_action_steps} ({execution_time_ms:.0f} ms)')
        self.get_logger().info(f'  Action Step Period:   {ACTION_STEP_PERIOD * 1000:.1f} ms ({TRAINING_FPS} FPS)')
        self.get_logger().info(f'  CSV Log Path:         {self.csv_log_path or "(disabled)"}')
        self.get_logger().info(f'  Base Vel Decay:       {self.base_velocity_decay}')
        self.get_logger().info(f'  Latency Skip (base):  {self.latency_skip_base} actions')
        self.get_logger().info(f'  Max Joint Delta:      {self.max_joint_delta} rad')
        self.get_logger().info(f'  Debug Save Images:    {self.debug_save_images}')
        self.get_logger().info(f'  H.264 Conditioning:   {self.h264_conditioning}')
        self.get_logger().info(f'  Interpolate Actions:  {self.interpolate_actions}')
        self.get_logger().info(f'  Back Init Height:     {self.back_init_height} m')
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
        """Calibrate the back if needed and move it to back_init_height."""
        POSITION_TOLERANCE = 0.005  # 5mm
        MOVE_TIMEOUT = 15.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

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

        # Move to target height
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
            rclpy.spin_once(self, timeout_sec=0.0)
            self.action_publisher.cmd_pub.publish(cmd)

            if latest_state[0] is not None:
                current_pos = latest_state[0].back_state.current_position
                if abs(current_pos - target) <= POSITION_TOLERANCE:
                    reached = True
                    break

            time.sleep(0.01)

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
        self.declare_parameter('transport', 'tcp')
        self.declare_parameter('server_host', '192.168.50.108')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('ipc_path', DEFAULT_IPC_PATH)

        # Inference settings
        self.declare_parameter('inference_timeout_ms', 5000)
        self.declare_parameter('task_description', 'find the can and pick it up')
        self.declare_parameter('enable_safety_limits', True)
        self.declare_parameter('action_smoothing_alpha', 0.95)
        self.declare_parameter('action_chunk_size', 16)

        # n_action_steps: how many of the 16 predicted actions to execute
        # before re-querying with a fresh observation.
        # 16 = execute full chunk (least responsive, 1.07s between queries)
        # 8 = execute half (NVIDIA sim default, 0.53s between queries)
        self.declare_parameter('n_action_steps', 16)

        self.declare_parameter('csv_log_path', '')

        # Base velocity drift correction (0-1). Applied as v *= (1 - decay).
        self.declare_parameter('base_velocity_decay', 0.15)

        # Per-body-part latency skip: base velocity reads ahead by this many
        # actions to compensate for ~200ms observation-to-action delay.
        # Joints play from action[0] — skipping joints causes trajectory
        # repetition during manipulation. Only base benefits from look-ahead.
        self.declare_parameter('latency_skip_base', 3)

        # Maximum allowed joint position change per command cycle (radians).
        self.declare_parameter('max_joint_delta', 2.0)

        # Debug: save observation images to /tmp/groot_debug_images/
        self.declare_parameter('debug_save_images', False)

        # H.264 conditioning to match training data pipeline
        self.declare_parameter('h264_conditioning', False)

        # Inter-action interpolation for smooth 100Hz output
        self.declare_parameter('interpolate_actions', True)

        # Back initialization height (meters). Set to -1.0 to disable.
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

        if self.zmq_client.connect():
            self._state = ClientState.ACTIVE
            self._active = True
            self.get_logger().info('GR00T inference active')
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

        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None

        self.action_publisher.publish_stop()
        self.action_publisher.reset_smoothing()

    def _trigger_estop(self):
        """Trigger emergency stop."""
        self.get_logger().warn('E-STOP triggered!')
        self._state = ClientState.E_STOP
        self._active = False
        self.safety.e_stop_active = True

        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None

        self.action_publisher.publish_stop()

    def _reset_estop(self):
        """Reset emergency stop."""
        if self._state != ClientState.E_STOP:
            return

        self.get_logger().info('E-STOP reset')
        self.safety.e_stop_active = False
        self.safety.reset_failures()
        self._state = ClientState.IDLE

        with self._action_lock:
            self._action_chunk = None
            self._pending_chunk = None

        self.action_publisher.reset_smoothing()

    def _check_starting_pose(self):
        """Check if robot starting pose is within training data distribution."""
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

        Waits until the FULL chunk (all n_action_steps) is consumed, then
        captures a fresh observation and requests a new chunk. The model must
        see the RESULT of the entire joint trajectory, not mid-trajectory
        state — otherwise it re-plans from a stale arm position and causes
        repeating "praying mantis" motions.

        The latency_skip_base only affects when the BASE zeros out in
        _command_callback, not the observation/re-query timing.
        """
        chunk_duration = self.n_action_steps * ACTION_STEP_PERIOD

        while self._running:
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            # Wait until current chunk's n_action_steps are consumed
            with self._action_lock:
                chunk_ts = self._chunk_timestamp
                has_chunk = self._action_chunk is not None

            if has_chunk:
                elapsed = time.monotonic() - chunk_ts
                remaining = chunk_duration - elapsed
                if remaining > 0:
                    time.sleep(min(remaining, 0.05))
                    continue

            # Get latest observation AFTER chunk is consumed
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            # Send observation to server (blocking ~130ms)
            response = self.zmq_client.send_observation(
                images=obs.images,
                state=obs.state,
                language=self.task_description,
            )

            if response is None:
                self.safety.record_failure()
                if self.safety.in_safe_mode:
                    self.get_logger().error('Too many failures, entering safe mode')
                    self._state = ClientState.ERROR
                    self._active = False
                    with self._action_lock:
                        self._action_chunk = None
                continue

            self.safety.update_inference_time()

            chunk = self._extract_action_chunk(response)
            if chunk is None:
                self.get_logger().warn('Invalid action response from server')
                continue

            # Log chunk diagnostics
            with self._action_lock:
                old_ts = self._chunk_timestamp
            gap = time.monotonic() - old_ts if has_chunk else 0.0
            self.get_logger().info(
                f'[chunk] new chunk after {gap:.3f}s '
                f'({gap/ACTION_STEP_PERIOD:.1f} actions consumed), '
                f'state_base={np.array2string(obs.state[0:6], precision=4, suppress_small=True)}, '
                f'action_base={np.array2string(chunk[0, 0:6], precision=4, suppress_small=True)}'
            )

            # Dump full trajectory for first 3 chunks
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

            # Install or buffer the new chunk
            with self._action_lock:
                if self._action_chunk is None:
                    self._action_chunk = chunk
                    self._chunk_timestamp = time.monotonic()
                else:
                    self._pending_chunk = chunk

    def _command_callback(self):
        """Command publishing callback (runs at exactly 100 Hz).

        Steps through the action chunk at the training data rate (67ms per
        action). When n_action_steps are consumed, promotes pending chunk or
        zeros base velocity while holding joint positions.

        Per-body-part latency skip: base velocity reads from action[idx+skip]
        (3 actions ahead by default) to compensate for ~200ms observation-to-
        action delay. Joints play from action[idx] — skipping joints causes
        trajectory repetition during manipulation.

        Inter-action interpolation: linearly interpolates between consecutive
        actions for smooth 100Hz output.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        now = time.monotonic()

        # Check if current chunk's n_action_steps are consumed
        with self._action_lock:
            chunk = self._action_chunk
            timestamp = self._chunk_timestamp
            pending = self._pending_chunk

            if chunk is not None and pending is not None:
                elapsed = now - timestamp
                chunk_duration = self.n_action_steps * ACTION_STEP_PERIOD
                if elapsed >= chunk_duration:
                    # Save last action of old chunk for transition blending
                    self._blend_from = chunk[min(self.n_action_steps - 1, len(chunk) - 1)].copy()
                    # Promote pending chunk
                    self._action_chunk = pending
                    self._pending_chunk = None
                    self._chunk_timestamp = now
                    chunk = self._action_chunk
                    timestamp = self._chunk_timestamp

        if chunk is None:
            return

        # Compute position within the chunk
        elapsed = now - timestamp
        t_frac = elapsed / ACTION_STEP_PERIOD  # fractional action index
        chunk_exhausted = t_frac >= self.n_action_steps
        idx = min(int(t_frac), self.n_action_steps - 1)

        # Clamp to valid chunk range
        idx = min(idx, len(chunk) - 1)

        # --- Joint action (no latency skip) ---
        if self.interpolate_actions and idx < len(chunk) - 1:
            alpha = t_frac - int(t_frac)
            action = (1.0 - alpha) * chunk[idx] + alpha * chunk[idx + 1]
        else:
            action = chunk[idx].copy()

        # --- Chunk transition blending (joints only) ---
        # When a new chunk starts, action[0] may jump from where the arm was.
        # Blend joints from the old chunk's last action to the new chunk over
        # a few steps to prevent snapping. Base velocity is handled separately.
        if self._blend_from is not None and idx < self._blend_steps:
            blend_alpha = (t_frac + 1.0) / (self._blend_steps + 1.0)  # ramp 0→1
            # Blend joints only (indices 6:22), not base velocity (0:6)
            action[6:] = (1.0 - blend_alpha) * self._blend_from[6:] + blend_alpha * action[6:]
        elif self._blend_from is not None and idx >= self._blend_steps:
            self._blend_from = None  # Done blending

        # --- Base velocity (with latency skip) ---
        # Base reads ahead in the chunk to compensate for observation-to-action
        # delay (~200ms). This makes the base more responsive to corrections
        # while joints play the full smooth trajectory.
        base_idx = idx + self.latency_skip_base
        base_exhausted = base_idx >= len(chunk)
        base_idx = min(base_idx, len(chunk) - 1)
        if not base_exhausted and self.interpolate_actions and base_idx < len(chunk) - 1:
            alpha = t_frac - int(t_frac)
            base_action = (1.0 - alpha) * chunk[base_idx] + alpha * chunk[base_idx + 1]
        elif not base_exhausted:
            base_action = chunk[base_idx]
        else:
            base_action = None
        if base_action is not None:
            action[0:6] = base_action[0:6]

        # Zero base velocity when either the base range or the full chunk is
        # exhausted. With latency_skip_base=3, the base runs out of unique
        # actions 3 steps before the chunk ends — zeroing here prevents the
        # robot from coasting on the last velocity for ~200ms extra.
        if chunk_exhausted or base_exhausted:
            action[0:6] = 0.0

        # Apply base velocity decay
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
        """
        if 'status' in response and response['status'] != 'ok':
            self.get_logger().warn(f"Server error: {response.get('error_message', 'unknown')}")
            return None

        if 'actions' not in response:
            return None

        actions = response['actions']

        if not isinstance(actions, list) or len(actions) == 0:
            return None

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

        self._running = False
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=2.0)

        self.action_publisher.publish_stop()
        self.action_publisher.close_csv()
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
