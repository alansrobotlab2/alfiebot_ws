#!/usr/bin/env python3
"""GR00T N1.6 inference client node for Alfiebot.

This node connects to a remote GR00T inference server via ZeroMQ,
sends synchronized observations, and publishes action predictions
to control the robot.

Key timing (exhaust mode, n_exec=12, trigger@12, skip=4):
- Server returns a 16-step action horizon
- Universal latency skip: execution window is actions[skip : skip + n_exec]
- Inference fires at chunk exhaust (804ms), observation captures trajectory result
- Overflow actions [12:16] bridge ~268ms of ~350ms inference RTT
- Fixed latency_skip=4 on every promotion (no overshoot compensation)
- ~0.87 Hz re-planning rate, <10% hold-and-wait per cycle
- Command publishing runs at exactly 100 Hz on the ROS2 executor
- Rate-limited interpolation smooths 15 FPS actions to 100 Hz output

Threading model:
- Inference runs on a dedicated background thread (blocking ZMQ ~280ms RTT)
- Command publishing runs at 100 Hz on the single-threaded ROS2 executor
- _action_lock protects the shared action chunk between threads
"""

import csv
from enum import Enum
from pathlib import Path
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
from ..core.action_smoother import ActionSmoother
from ..core.action_interpolator import ActionInterpolator
from ..core.chunk_buffer import ChunkBuffer, TimestampedChunk
from ..core.observation_bridge import Observation, ObservationBridge
from ..core.rate_limited_interpolator import RateLimitedInterpolator
from ..core.zmq_async_client import ZMQAsyncClient
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
    - Overlapped inference: fires mid-chunk for near-zero dead time
    - Action chunk execution at training rate (15 FPS)
    - Action publishing to robot (100 Hz on ROS2 executor)
    - Safety monitoring and state management

    Overlapped execution (n_action_steps, latency_skip, inference_trigger_step):
    - Server returns 16 actions per inference call (the action_horizon)
    - latency_skip accounts for observation staleness: exec starts at action[skip]
    - n_action_steps controls how many actions are EXECUTED per chunk
    - inference_trigger_step fires inference mid-chunk (overlapped) or at exhaust
    - n_action_steps=8, skip=4: execution window is actions[4:12] over 536ms
    - trigger_step=4: inference fires at 268ms, result arrives as pending ~548ms
    - After n_action_steps consumed, promote pending or zero base + hold joints
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
        self.base_smoothing_alpha = self.get_parameter('base_smoothing_alpha').value
        self.joint_smoothing_alpha = self.get_parameter('joint_smoothing_alpha').value
        self.action_chunk_size = self.get_parameter('action_chunk_size').value
        self.n_action_steps = self.get_parameter('n_action_steps').value
        self.csv_log_path = self.get_parameter('csv_log_path').value

        self.latency_skip = self.get_parameter('latency_skip').value
        self.inference_trigger_step = self.get_parameter('inference_trigger_step').value
        self.chunk_blend_steps = self.get_parameter('chunk_blend_steps').value
        self.debug_save_images = self.get_parameter('debug_save_images').value
        self.h264_conditioning = self.get_parameter('h264_conditioning').value
        self.interpolate_actions = self.get_parameter('interpolate_actions').value
        self.back_init_height = self.get_parameter('back_init_height').value

        # Continuous inference parameters
        self.continuous_inference = self.get_parameter('continuous_inference').value
        self.smoothing_strategy = self.get_parameter('smoothing_strategy').value
        self.smoothing_decay_m = self.get_parameter('smoothing_decay_m').value
        self.max_buffer_chunks = self.get_parameter('max_buffer_chunks').value

        # Post-ensembling filter and interpolation
        self._smoothing_method = self.get_parameter('smoothing_method').value
        self._interpolation_method = self.get_parameter('interpolation_method').value

        # Rate-limited interpolation
        self.rate_limit_enabled = self.get_parameter('rate_limit_enabled').value

        # Validate overlapped execution parameters
        max_n_steps = self.action_chunk_size - self.latency_skip
        if self.n_action_steps < 1 or self.n_action_steps > max_n_steps:
            self.get_logger().warn(
                f'n_action_steps={self.n_action_steps} out of range [1, {max_n_steps}] '
                f'(chunk_size={self.action_chunk_size}, skip={self.latency_skip}), '
                f'clamping to {max_n_steps}'
            )
            self.n_action_steps = min(max(self.n_action_steps, 1), max_n_steps)

        if self.inference_trigger_step < 1 or self.inference_trigger_step > self.n_action_steps:
            self.get_logger().warn(
                f'inference_trigger_step={self.inference_trigger_step} out of range '
                f'[1, {self.n_action_steps}], clamping to {self.n_action_steps}'
            )
            self.inference_trigger_step = min(max(self.inference_trigger_step, 1), self.n_action_steps)

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
        # _pending_chunk buffers the next chunk until the current one is consumed.
        self._action_chunk: Optional[np.ndarray] = None   # shape (N, 22)
        self._pending_chunk: Optional[np.ndarray] = None  # next chunk, waiting
        self._chunk_timestamp: float = 0.0                # time.monotonic() when chunk started
        self._effective_skip: int = self.latency_skip     # fixed skip (latency + blend offset)
        self._blend_from: Optional[np.ndarray] = None     # last action from old chunk for blend

        self._action_lock = threading.Lock()
        self._total_chunks = 0                            # for diagnostic logging

        # Continuous inference: ChunkBuffer for temporal ensembling
        self._chunk_buffer: Optional[ChunkBuffer] = None
        self._activation_time: Optional[float] = None  # monotonic ref for frame counting
        if self.continuous_inference:
            self._chunk_buffer = ChunkBuffer(
                max_chunks=self.max_buffer_chunks,
                strategy=self.smoothing_strategy,
                decay_m=self.smoothing_decay_m,
                latency_skip=self.latency_skip,
                chunk_size=self.action_chunk_size,
                ema_alpha_base=self.base_smoothing_alpha,
                ema_alpha_joints=self.joint_smoothing_alpha,
            )

        # Post-ensembling filter (Savitzky-Golay or Butterworth)
        self._action_smoother = ActionSmoother(method='none')
        if self._smoothing_method != 'none':
            try:
                self._action_smoother = ActionSmoother(
                    method=self._smoothing_method,
                    savgol_window=self.get_parameter('savgol_window').value,
                    savgol_polyorder=self.get_parameter('savgol_polyorder').value,
                    butter_order=self.get_parameter('butterworth_order').value,
                    butter_cutoff_hz=self.get_parameter('butterworth_cutoff_hz').value,
                )
                self.get_logger().info(
                    f'Action smoother: {self._smoothing_method}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to init smoother: {e}')

        # Interpolation method (linear or cubic_spline) — legacy fallback
        self._action_interpolator = ActionInterpolator(method='linear')
        if self._interpolation_method != 'linear':
            try:
                self._action_interpolator = ActionInterpolator(
                    method=self._interpolation_method,
                    spline_window=self.get_parameter('spline_window').value,
                )
                self.get_logger().info(
                    f'Action interpolator: {self._interpolation_method}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to init interpolator: {e}')

        # Rate-limited interpolator (G1-style velocity-capped joint transitions)
        self._rate_limiter: Optional[RateLimitedInterpolator] = None
        if self.rate_limit_enabled:
            max_speeds = {
                'back': self.get_parameter('rate_limit_back').value,
                'left_arm': self.get_parameter('rate_limit_left_arm').value,
                'left_gripper': self.get_parameter('rate_limit_left_gripper').value,
                'right_arm': self.get_parameter('rate_limit_right_arm').value,
                'right_gripper': self.get_parameter('rate_limit_right_gripper').value,
                'head': self.get_parameter('rate_limit_head').value,
            }
            self._rate_limiter = RateLimitedInterpolator(
                max_speeds=max_speeds,
                dt=1.0 / COMMAND_RATE_HZ,
                target_ema_alpha=self.joint_smoothing_alpha,
            )
            self.get_logger().info(
                f'Rate-limited interpolator enabled: {max_speeds}, '
                f'target_ema_alpha={self.joint_smoothing_alpha}'
            )

        # Per-action CSV logger (one row per action step, not per 100Hz tick)
        self._action_csv_file = None
        self._action_csv_writer = None
        self._last_logged_chunk_id = -1
        self._last_logged_abs_idx = -1
        if self.csv_log_path:
            self._init_action_csv(self.csv_log_path)

        # Initialize safety monitor
        # Watchdog timeout must exceed the chunk execution time plus inference latency
        execution_time = self.n_action_steps * ACTION_STEP_PERIOD
        self.safety = SafetyMonitor(
            watchdog_timeout=execution_time + 1.0,
            max_consecutive_failures=5,
        )

        # Async ZMQ mode
        self.use_async_zmq = self.get_parameter('use_async_zmq').value
        self.async_push_port = self.get_parameter('async_push_port').value
        self.async_pull_port = self.get_parameter('async_pull_port').value

        # Initialize ZMQ client (REQ/REP — always created for ping/health checks)
        self.zmq_client = ZMQClient(
            server_address=self.server_address,
            timeout_ms=self.inference_timeout_ms,
            logger=lambda msg: self.get_logger().info(msg),
        )

        # Initialize async ZMQ client (PUSH/PULL — for live inference when enabled)
        self.async_client: Optional[ZMQAsyncClient] = None
        if self.use_async_zmq:
            push_addr = f'tcp://{self.server_host}:{self.async_push_port}'
            pull_addr = f'tcp://{self.server_host}:{self.async_pull_port}'
            self.async_client = ZMQAsyncClient(
                push_address=push_addr,
                pull_address=pull_addr,
                req_address=self.server_address,
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
            base_smoothing_alpha=self.base_smoothing_alpha,
            joint_smoothing_alpha=self.joint_smoothing_alpha,
            csv_log_path=self.csv_log_path,
            # Base velocity limits
            max_base_linear_x=self.get_parameter('max_base_linear_x').value,
            max_base_linear_y=self.get_parameter('max_base_linear_y').value,
            max_base_angular_z=self.get_parameter('max_base_angular_z').value,
            max_base_linear_accel=self.get_parameter('max_base_linear_accel').value,
            max_base_angular_accel=self.get_parameter('max_base_angular_accel').value,
            # Per-group servo config
            servo_speed_arms=self.get_parameter('servo_speed_arms').value,
            servo_speed_grippers=self.get_parameter('servo_speed_grippers').value,
            servo_speed_head=self.get_parameter('servo_speed_head').value,
            servo_accel_arms=self.get_parameter('servo_accel_arms').value,
            servo_accel_grippers=self.get_parameter('servo_accel_grippers').value,
            servo_accel_head=self.get_parameter('servo_accel_head').value,
            servo_torque_arms=self.get_parameter('servo_torque_arms').value,
            servo_torque_grippers=self.get_parameter('servo_torque_grippers').value,
            servo_torque_head=self.get_parameter('servo_torque_head').value,
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
        if self.use_async_zmq and self.continuous_inference:
            inference_target = self._async_continuous_inference_loop
        elif self.use_async_zmq:
            inference_target = self._async_inference_loop
        elif self.continuous_inference:
            inference_target = self._continuous_inference_loop
        else:
            inference_target = self._inference_loop
        self._inference_thread = threading.Thread(
            target=inference_target,
            name='groot_inference',
            daemon=True,
        )
        self._inference_thread.start()

    def _log_parameters(self):
        """Log all parameter values on startup."""
        exec_ms = self.n_action_steps * ACTION_STEP_PERIOD * 1000
        trigger_ms = self.inference_trigger_step * ACTION_STEP_PERIOD * 1000
        skip_start = self.latency_skip
        skip_end = self.latency_skip + self.n_action_steps - 1
        overlapped = self.inference_trigger_step < self.n_action_steps
        mode = 'overlapped' if overlapped else 'sequential'
        replan_hz = 1.0 / (self.n_action_steps * ACTION_STEP_PERIOD) if self.n_action_steps > 0 else 0

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
        self.get_logger().info(f'  Base Smoothing:       {self.base_smoothing_alpha}')
        self.get_logger().info(f'  Joint Smoothing:      {self.joint_smoothing_alpha}')
        self.get_logger().info(f'  --- Execution Strategy: {mode} ---')
        self.get_logger().info(f'  Action Chunk Size:    {self.action_chunk_size}')
        self.get_logger().info(f'  n_action_steps:       {self.n_action_steps} ({exec_ms:.0f} ms)')
        self.get_logger().info(f'  Latency Skip:         {self.latency_skip} (exec window: [{skip_start}:{skip_end+1}])')
        self.get_logger().info(f'  Inference Trigger:    step {self.inference_trigger_step} ({trigger_ms:.0f} ms into chunk)')
        self.get_logger().info(f'  Re-planning Rate:     {replan_hz:.2f} Hz')

        self.get_logger().info(f'  Chunk Blend Steps:    {self.chunk_blend_steps}')
        self.get_logger().info(f'  Action Step Period:   {ACTION_STEP_PERIOD * 1000:.1f} ms ({TRAINING_FPS} FPS)')
        if self.continuous_inference:
            self.get_logger().info(f'  --- Continuous Inference (ACTIVE) ---')
            self.get_logger().info(f'  Smoothing Strategy:   {self.smoothing_strategy}')
            self.get_logger().info(f'  Decay M:              {self.smoothing_decay_m}')
            self.get_logger().info(f'  Max Buffer Chunks:    {self.max_buffer_chunks}')
        self.get_logger().info(f'  --- Smoothing (G1-style) ---')
        self.get_logger().info(f'  Rate Limiter:         {self.rate_limit_enabled}')
        if self.rate_limit_enabled:
            self.get_logger().info(f'    back:               {self.get_parameter("rate_limit_back").value} m/s')
            self.get_logger().info(f'    arms:               {self.get_parameter("rate_limit_left_arm").value} rad/s')
            self.get_logger().info(f'    grippers:           {self.get_parameter("rate_limit_left_gripper").value} rad/s')
            self.get_logger().info(f'    head:               {self.get_parameter("rate_limit_head").value} rad/s')
        self.get_logger().info(f'  Base Vel Limits:      lx={self.get_parameter("max_base_linear_x").value} ly={self.get_parameter("max_base_linear_y").value} az={self.get_parameter("max_base_angular_z").value}')
        self.get_logger().info(f'  Base Accel Limits:    lin={self.get_parameter("max_base_linear_accel").value} ang={self.get_parameter("max_base_angular_accel").value}')
        self.get_logger().info(f'  Servo Speed:          arms={self.get_parameter("servo_speed_arms").value} grippers={self.get_parameter("servo_speed_grippers").value} head={self.get_parameter("servo_speed_head").value}')
        self.get_logger().info(f'  --- Other ---')
        self.get_logger().info(f'  CSV Log Path:         {self.csv_log_path or "(disabled)"}')
        self.get_logger().info(f'  Debug Save Images:    {self.debug_save_images}')
        self.get_logger().info(f'  H.264 Conditioning:   {self.h264_conditioning}')
        self.get_logger().info(f'  Interpolate Actions:  {self.interpolate_actions} {"(superseded by rate limiter)" if self.rate_limit_enabled else ""}')
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

        # Async ZMQ (PUSH/PULL) for decoupled observation/action flow
        self.declare_parameter('use_async_zmq', False)
        self.declare_parameter('async_push_port', 5556)  # client → server observations
        self.declare_parameter('async_pull_port', 5557)  # server → client actions

        # Inference settings
        self.declare_parameter('inference_timeout_ms', 5000)
        self.declare_parameter('task_description', 'find the can and pick it up')
        self.declare_parameter('enable_safety_limits', True)
        self.declare_parameter('base_smoothing_alpha', 1.0)
        self.declare_parameter('joint_smoothing_alpha', 0.95)
        self.declare_parameter('action_chunk_size', 16)

        # n_action_steps: how many actions to execute per chunk.
        # 8 = overlapped (536ms, ~1.87 Hz re-planning, near-zero dead time)
        # 16 = sequential (1.07s, 0.75 Hz, 270ms dead time)
        self.declare_parameter('n_action_steps', 8)

        self.declare_parameter('csv_log_path', '')


        # Universal latency skip: all body parts read from action[skip + idx]
        # to compensate for observation-to-action delay (~280ms = 4 steps).
        # Phase 0 validated r>0.91 for all body parts at k=4.
        self.declare_parameter('latency_skip', 4)

        # Fire inference at this step within the execution window (overlapped).
        # Set equal to n_action_steps for sequential mode.
        self.declare_parameter('inference_trigger_step', 4)

        # Blend joints (not base) over N steps at chunk transitions.
        self.declare_parameter('chunk_blend_steps', 2)

        # Debug: save observation images to /tmp/groot_debug_images/
        self.declare_parameter('debug_save_images', False)

        # H.264 conditioning to match training data pipeline
        self.declare_parameter('h264_conditioning', False)

        # Inter-action interpolation for smooth 100Hz output
        self.declare_parameter('interpolate_actions', True)

        # Back initialization height (meters). Set to -1.0 to disable.
        self.declare_parameter('back_init_height', 0.1)

        # Continuous inference mode: always-churning inference with temporal
        # ensembling via ChunkBuffer. When false, uses the existing
        # trigger-step / pending-chunk / promote execution model.
        self.declare_parameter('continuous_inference', False)
        self.declare_parameter('smoothing_strategy', 'latest')
        self.declare_parameter('smoothing_decay_m', 0.01)
        self.declare_parameter('max_buffer_chunks', 8)

        # Post-ensembling filter: 'none', 'savgol', 'butterworth'
        self.declare_parameter('smoothing_method', 'none')
        self.declare_parameter('savgol_window', 5)
        self.declare_parameter('savgol_polyorder', 2)
        self.declare_parameter('butterworth_order', 2)
        self.declare_parameter('butterworth_cutoff_hz', 5.0)

        # Interpolation method: 'linear', 'cubic_spline'
        self.declare_parameter('interpolation_method', 'linear')
        self.declare_parameter('spline_window', 6)

        # Rate-limited interpolation (G1-style): velocity-capped joint transitions
        self.declare_parameter('rate_limit_enabled', True)
        self.declare_parameter('rate_limit_back', 0.3)
        self.declare_parameter('rate_limit_left_arm', 2.0)
        self.declare_parameter('rate_limit_left_gripper', 3.0)
        self.declare_parameter('rate_limit_right_arm', 2.0)
        self.declare_parameter('rate_limit_right_gripper', 3.0)
        self.declare_parameter('rate_limit_head', 1.0)

        # Base velocity limits (BEHAVIOR R1Pro-inspired)
        self.declare_parameter('max_base_linear_x', 0.15)
        self.declare_parameter('max_base_linear_y', 0.15)
        self.declare_parameter('max_base_angular_z', 0.8)
        self.declare_parameter('max_base_linear_accel', 0.3)
        self.declare_parameter('max_base_angular_accel', 1.5)

        # Per-group servo configuration
        self.declare_parameter('servo_speed_arms', 2.0)
        self.declare_parameter('servo_speed_grippers', 3.0)
        self.declare_parameter('servo_speed_head', 1.0)
        self.declare_parameter('servo_accel_arms', 5.0)
        self.declare_parameter('servo_accel_grippers', 8.0)
        self.declare_parameter('servo_accel_head', 3.0)
        self.declare_parameter('servo_torque_arms', 0.5)
        self.declare_parameter('servo_torque_grippers', 0.4)
        self.declare_parameter('servo_torque_head', 0.3)

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

        # Connect async client if enabled, otherwise REQ/REP
        if self.async_client is not None:
            if not self.async_client.connect():
                self._state = ClientState.ERROR
                self.get_logger().error('Failed to connect async ZMQ client')
                return

        if self.zmq_client.connect():
            self._state = ClientState.ACTIVE
            self._active = True
            self._activation_time = time.monotonic()
            if self._chunk_buffer is not None:
                self._chunk_buffer.reset()
            self.get_logger().info(
                f'GR00T inference active '
                f'({"async PUSH/PULL" if self.async_client else "REQ/REP"})'
            )
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

        if self._chunk_buffer is not None:
            self._chunk_buffer.reset()
        if self._rate_limiter is not None:
            self._rate_limiter.reset()

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

        if self._chunk_buffer is not None:
            self._chunk_buffer.reset()
        if self._rate_limiter is not None:
            self._rate_limiter.reset()

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

        if self._rate_limiter is not None:
            self._rate_limiter.reset()
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
        """Background inference loop with overlapped execution.

        Fires inference mid-chunk (at inference_trigger_step) instead of
        waiting for full chunk exhaust. This eliminates dead time and
        approximately doubles the re-planning rate.

        First chunk: no prior chunk → fire inference immediately.
        Subsequent chunks: fire at trigger_step into the execution window,
        while the robot is still executing the current chunk's actions.
        The result arrives as _pending_chunk before the current chunk exhausts.

        Timing (n_exec=8, trigger@4, skip=4, ~280ms inference):
          t=0      start executing chunk actions[4:12]
          t=268ms  capture observation, fire inference (trigger step 4)
          t=536ms  chunk exhausts → promote pending → start next chunk
          t=548ms  inference result arrives (buffered or installed)
        """
        trigger_time = self.inference_trigger_step * ACTION_STEP_PERIOD

        while self._running:
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            # Decide whether to fire inference now:
            # 1. No chunk at all → fire immediately (first chunk)
            # 2. Has chunk, no pending, trigger time reached → fire (overlapped)
            # 3. Has pending → wait (don't overwrite buffered chunk)
            with self._action_lock:
                has_chunk = self._action_chunk is not None
                has_pending = self._pending_chunk is not None
                chunk_ts = self._chunk_timestamp

            if has_pending:
                # Next chunk already buffered — nothing to do
                time.sleep(0.01)
                continue

            if has_chunk:
                # Wait until trigger point within execution window
                elapsed = time.monotonic() - chunk_ts
                if elapsed < trigger_time:
                    time.sleep(min(trigger_time - elapsed, 0.05))
                    continue

            # Capture observation (mid-chunk for overlapped, fresh for first chunk)
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            # Send observation to server (blocking ~280ms RTT)
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
            skip = self.latency_skip
            skip_idx = min(skip, len(chunk) - 1)
            self.get_logger().info(
                f'[chunk] {"pending" if has_chunk else "first"} chunk, '
                f'skip={skip}, n_exec={self.n_action_steps}, '
                f'exec_window=[{skip}:{skip + self.n_action_steps}], '
                f'action_base[{skip}]='
                f'{np.array2string(chunk[skip_idx, 0:6], precision=4, suppress_small=True)}'
            )

            # Dump full trajectory for first 3 chunks
            if self._total_chunks < 3:
                self.get_logger().info(
                    f'[chunk_dump] state: '
                    f'{np.array2string(obs.state, precision=3, suppress_small=True)}'
                )
                for i in range(len(chunk)):
                    a = chunk[i]
                    marker = ''
                    if i == skip:
                        marker = ' ◄ exec start'
                    elif i == skip + self.n_action_steps - 1:
                        marker = ' ◄ exec end'
                    self.get_logger().info(
                        f'[chunk_dump] action[{i:2d}]: '
                        f'base=({a[0]:+.4f},{a[1]:+.4f},{a[5]:+.4f}) '
                        f'back={a[6]:.3f} '
                        f'r_arm=({a[13]:+.3f},{a[14]:+.3f},{a[15]:+.3f}) '
                        f'r_grip={a[18]:.3f} '
                        f'head=({a[19]:+.3f},{a[20]:+.3f},{a[21]:+.3f})'
                        f'{marker}'
                    )
            self._total_chunks += 1

            # Install or buffer the new chunk
            with self._action_lock:
                if self._action_chunk is None:
                    # First chunk — no latency skip. The observation was
                    # captured immediately before inference (no mid-chunk
                    # trigger), so there's no staleness to compensate for.
                    # Start executing from action[0].
                    now = time.monotonic()
                    self._action_chunk = chunk
                    self._chunk_timestamp = now
                    self._effective_skip = 0
                else:
                    # Buffer as pending (promoted when current chunk exhausts)
                    self._pending_chunk = chunk

    def _async_inference_loop(self):
        """Async PUSH/PULL inference loop.

        Observations are pushed non-blocking to the server. Action results
        arrive asynchronously on the async_client's receiver thread.
        This loop polls for results and decides when to push new observations,
        using the same trigger-step timing as the synchronous _inference_loop.

        All existing overlap/skip/blend behavior is preserved:
        - latency_skip compensates for observation staleness
        - inference_trigger_step controls when observations are captured
        - _pending_chunk / promote pattern handles chunk transitions
        - chunk_blend_steps blends at transitions
        """
        trigger_time = self.inference_trigger_step * ACTION_STEP_PERIOD
        awaiting_result = False  # True after push, until result received

        while self._running:
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            # Check for new action result from server (non-blocking)
            result = self.async_client.pop_result()
            if result is not None:
                awaiting_result = False
                self.safety.update_inference_time()

                chunk = self._extract_action_chunk(result)
                if chunk is not None:
                    obs_id = result.get('obs_id', -1)
                    latency_ms = result.get('latency_ms', 0)

                    with self._action_lock:
                        has_chunk = self._action_chunk is not None

                    # Log chunk diagnostics
                    skip = self.latency_skip
                    skip_idx = min(skip, len(chunk) - 1)
                    self.get_logger().info(
                        f'[async-chunk] {"pending" if has_chunk else "first"} '
                        f'obs_id={obs_id} latency={latency_ms:.0f}ms '
                        f'skip={skip} exec_window=[{skip}:{skip + self.n_action_steps}] '
                        f'action_base[{skip}]='
                        f'{np.array2string(chunk[skip_idx, 0:6], precision=4, suppress_small=True)}'
                    )

                    self._total_chunks += 1

                    # Install or buffer chunk (same logic as sync loop)
                    with self._action_lock:
                        if self._action_chunk is None:
                            self._action_chunk = chunk
                            self._chunk_timestamp = time.monotonic()
                            self._effective_skip = 0
                        else:
                            self._pending_chunk = chunk

            # Decide whether to push a new observation
            with self._action_lock:
                has_chunk = self._action_chunk is not None
                has_pending = self._pending_chunk is not None
                chunk_ts = self._chunk_timestamp

            # Don't push if we already have a pending chunk or are awaiting a result
            if has_pending or awaiting_result:
                time.sleep(0.005)
                continue

            # Wait for trigger point within execution window
            if has_chunk:
                elapsed = time.monotonic() - chunk_ts
                if elapsed < trigger_time:
                    time.sleep(min(trigger_time - elapsed, 0.01))
                    continue

            # Capture observation and push (non-blocking)
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            obs_id = self.async_client.push_observation(
                images=obs.images,
                state=obs.state,
                language=self.task_description,
            )

            if obs_id >= 0:
                awaiting_result = True
            else:
                self.get_logger().warn('[async] Failed to push observation')
                time.sleep(0.01)

    def _async_continuous_inference_loop(self):
        """Async continuous inference: PUSH/PULL with temporal ensembling.

        Combines async ZMQ transport (non-blocking PUSH/PULL) with
        continuous inference (always-churning, ChunkBuffer temporal
        ensembling). Observations are pushed as fast as possible;
        results are popped non-blocking and added to the ChunkBuffer.

        Unlike _async_inference_loop, this does NOT use trigger-step
        timing or _action_chunk/_pending_chunk. Instead it matches
        _continuous_inference_loop: fire continuously, write to
        ChunkBuffer, let _continuous_command_callback read from it.

        Unlike _continuous_inference_loop, this does NOT block on
        send_observation(). PUSH/PULL sockets decouple observation
        delivery from inference latency. With HWM=1, pushing a new
        observation while one is in-flight drops the stale one.
        """
        # Track obs_id → (push_time, push_frame) for accurate frame association
        obs_push_info: dict[int, tuple[float, int]] = {}
        awaiting_first_chunk = True

        while self._running:
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                awaiting_first_chunk = True
                continue

            # ── Check for results (non-blocking) ──────────────────
            result = self.async_client.pop_result()
            if result is not None:
                self.safety.update_inference_time()

                chunk = self._extract_action_chunk(result)
                if chunk is not None:
                    arrival_time = time.monotonic()
                    arrival_frame = self._time_to_frame(arrival_time)

                    obs_id = result.get('obs_id', -1)
                    latency_ms = result.get('latency_ms', 0)

                    # Look up the push time/frame for this obs_id
                    push_time, push_frame = obs_push_info.pop(obs_id, (arrival_time, arrival_frame))

                    # Add to ChunkBuffer for temporal ensembling
                    self._chunk_buffer.add_chunk(TimestampedChunk(
                        actions=chunk,
                        obs_frame=push_frame,
                        arrival_frame=arrival_frame,
                        chunk_id=self._total_chunks,
                        obs_timestamp=push_time,
                        arrival_timestamp=arrival_time,
                    ))

                    if self._total_chunks % 10 == 0:
                        self.get_logger().info(
                            f'[async-continuous] chunk {self._total_chunks}: '
                            f'obs_id={obs_id} obs_frame={push_frame} '
                            f'arrival_frame={arrival_frame} '
                            f'buffer={self._chunk_buffer.num_chunks} '
                            f'latency={latency_ms:.0f}ms'
                        )

                    self._total_chunks += 1
                    awaiting_first_chunk = False

            # ── Push new observation (non-blocking) ───────────────
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.005)
                continue

            push_time = time.monotonic()
            push_frame = self._time_to_frame(push_time)

            obs_id = self.async_client.push_observation(
                images=obs.images,
                state=obs.state,
                language=self.task_description,
            )

            if obs_id >= 0:
                obs_push_info[obs_id] = (push_time, push_frame)
                # Prune old entries (keep last 50)
                if len(obs_push_info) > 50:
                    oldest_key = min(obs_push_info.keys())
                    del obs_push_info[oldest_key]

            # Brief sleep to avoid busy-spinning
            if awaiting_first_chunk:
                time.sleep(0.005)
            else:
                time.sleep(0.01)

    def _continuous_inference_loop(self):
        """Continuous inference: always-churning loop.

        Captures observation and fires inference immediately after receiving
        each chunk result. No trigger-step timing — just keep the pipeline
        full. Yields ~3.3Hz at 280ms RTT, ~5Hz at 200ms RTT.

        Chunks are added to the ChunkBuffer for temporal ensembling.
        The command callback queries the buffer at 100Hz.
        """
        while self._running:
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            # Capture observation NOW
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            obs_time = time.monotonic()
            obs_frame = self._time_to_frame(obs_time)

            # Blocking inference call (~280ms RTT)
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
                continue

            self.safety.update_inference_time()

            chunk = self._extract_action_chunk(response)
            if chunk is None:
                self.get_logger().warn('Invalid action response from server')
                continue

            arrival_time = time.monotonic()
            arrival_frame = self._time_to_frame(arrival_time)

            # Add to rolling buffer
            self._chunk_buffer.add_chunk(TimestampedChunk(
                actions=chunk,
                obs_frame=obs_frame,
                arrival_frame=arrival_frame,
                chunk_id=self._total_chunks,
                obs_timestamp=obs_time,
                arrival_timestamp=arrival_time,
            ))

            if self._total_chunks % 10 == 0:
                self.get_logger().info(
                    f'[continuous] chunk {self._total_chunks}: '
                    f'obs_frame={obs_frame}, arrival_frame={arrival_frame}, '
                    f'buffer={self._chunk_buffer.num_chunks}, '
                    f'latency={((arrival_time - obs_time) * 1000):.0f}ms'
                )

            self._total_chunks += 1
            # Immediately loop back — no waiting

    def _continuous_command_callback(self):
        """100Hz command callback for continuous inference mode.

        Pipeline: ChunkBuffer → ActionSmoother → ActionInterpolator → publish.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        now = time.monotonic()
        current_frame = self._time_to_frame(now)

        # Get ensembled action from ChunkBuffer (15 FPS)
        action = self._chunk_buffer.get_action(current_frame)
        if action is None:
            return

        # Apply post-ensembling filter (Savitzky-Golay or Butterworth)
        action = self._action_smoother.smooth(action)

        # Feed waypoint to interpolator and evaluate at sub-frame time
        if self.interpolate_actions:
            self._action_interpolator.update_waypoint(current_frame, action)
            # Also feed the next frame's action if available
            next_raw = self._chunk_buffer.get_action(current_frame + 1)
            if next_raw is not None:
                next_smoothed = self._action_smoother.smooth(next_raw)
                self._action_interpolator.update_waypoint(current_frame + 1, next_smoothed)

            frac_time = (now - self._activation_time) / ACTION_STEP_PERIOD
            interp = self._action_interpolator.evaluate(frac_time)
            if interp is not None:
                action = interp

        obs = self.observation_bridge.get_latest_observation()
        current_state = obs.state if obs is not None else None

        # Log per-action step
        self._log_action_step(
            self._chunk_buffer.current_chunk_id,
            current_frame,
            action,
            current_state,
        )

        # Publish — joint smoothing handled by ChunkBuffer/ActionSmoother above,
        # but base velocity limiting (accel cap, magnitude cap) is in ActionPublisher.
        self.action_publisher.publish_action(
            action=action,
            current_state=current_state,
            apply_smoothing=True,
            apply_safety=self.enable_safety_limits,
            chunk_id=self._chunk_buffer.current_chunk_id,
            action_idx=current_frame % self.action_chunk_size,
        )

    def _time_to_frame(self, mono_time: float) -> int:
        """Convert monotonic time to frame index since activation."""
        if self._activation_time is None:
            return 0
        return int((mono_time - self._activation_time) / ACTION_STEP_PERIOD)

    def _init_action_csv(self, base_path: str):
        """Initialize per-action CSV log (one row per action step)."""
        p = Path(base_path)
        action_path = p.parent / f'{p.stem}_actions{p.suffix}'
        action_path.parent.mkdir(parents=True, exist_ok=True)
        self._action_csv_file = open(action_path, 'w', newline='')
        self._action_csv_writer = csv.writer(self._action_csv_file)

        joint_names = ActionPublisher.JOINT_NAMES
        header = ['timestamp', 'chunk_id', 'action_idx', 'effective_skip']
        for prefix in ['action', 'state']:
            for name in joint_names:
                header.append(f'{prefix}_{name}')
        self._action_csv_writer.writerow(header)
        self._action_csv_file.flush()
        self.get_logger().info(f'Per-action CSV logging enabled: {action_path}')

    def _log_action_step(self, chunk_id: int, abs_idx: int,
                         action: np.ndarray, state: Optional[np.ndarray]):
        """Write one row to the per-action CSV when action index changes."""
        if self._action_csv_writer is None:
            return
        if chunk_id == self._last_logged_chunk_id and abs_idx == self._last_logged_abs_idx:
            return
        self._last_logged_chunk_id = chunk_id
        self._last_logged_abs_idx = abs_idx

        state_vals = state if state is not None else np.zeros(22)
        row = [time.time(), chunk_id, abs_idx, self._effective_skip]
        row.extend(action.tolist())
        row.extend(state_vals.tolist() if isinstance(state_vals, np.ndarray) else state_vals)
        self._action_csv_writer.writerow(row)
        self._action_csv_file.flush()

    def _close_action_csv(self):
        """Close per-action CSV log file."""
        if self._action_csv_file is not None:
            self._action_csv_file.close()
            self._action_csv_file = None
            self._action_csv_writer = None

    def _command_callback(self):
        """Command publishing callback (runs at exactly 100 Hz).

        Dispatches to continuous mode or classic overlapped mode.
        """
        if self.continuous_inference:
            return self._continuous_command_callback()
        return self._classic_command_callback()

    def _classic_command_callback(self):
        """Classic overlapped command callback (original behavior).

        Steps through the action chunk at the training data rate (67ms per
        action step). All body parts share the same chunk and timeline.

        When n_action_steps are consumed and a pending chunk is available,
        promotes with a one-step blend (67ms lerp from old last action to
        new chunk's first action). If no pending, continues executing up
        to all 16 actions. Fatal exit if all 16 exhaust with no pending.

        Inter-action interpolation lerps between consecutive actions for
        smooth 100 Hz output within each chunk.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        now = time.monotonic()

        with self._action_lock:
            chunk = self._action_chunk
            timestamp = self._chunk_timestamp
            pending = self._pending_chunk

        if chunk is None:
            return

        # Compute current position in chunk
        elapsed = now - timestamp
        t_frac = elapsed / ACTION_STEP_PERIOD
        exec_idx = int(t_frac)
        abs_idx = self._effective_skip + exec_idx

        # Promote pending chunk when execution window consumed
        if pending is not None and elapsed >= self.n_action_steps * ACTION_STEP_PERIOD:
            # Save current action for blend transition (only if blending enabled)
            if self.chunk_blend_steps > 0:
                blend_idx = min(abs_idx, len(chunk) - 1)
                self._blend_from = chunk[blend_idx].copy()

            with self._action_lock:
                # Fixed latency skip on every promotion. No overshoot
                # compensation: latency_skip already accounts for the
                # observation-to-action delay (~280ms = 4 steps at 15 FPS).
                # In exhaust mode (trigger_step == n_action_steps), the
                # observation is captured fresh at chunk exhaust — adding
                # inference RTT as extra skip double-compensates and causes
                # a death spiral of shrinking execution windows.
                # +1 when blending to use 67ms for old→new lerp.
                self._effective_skip = self.latency_skip + (1 if self.chunk_blend_steps > 0 else 0)
                self._action_chunk = pending
                self._pending_chunk = None
                self._chunk_timestamp = now

            # Recompute on new chunk
            chunk = pending
            timestamp = now
            elapsed = 0.0
            t_frac = 0.0
            exec_idx = 0
            abs_idx = self._effective_skip

        # Hold position if all 16 actions exhausted with no pending.
        # Zero base velocity, hold last joint positions. The safety watchdog
        # will catch genuinely stuck situations (timeout = exec_time + 1.0s).
        if abs_idx >= len(chunk):
            if not hasattr(self, '_hold_logged') or not self._hold_logged:
                self.get_logger().warn(
                    f'[chunk] Holding: exhausted all {len(chunk)} actions '
                    f'with no pending chunk (elapsed={elapsed:.3f}s, '
                    f'effective_skip={self._effective_skip})')
                self._hold_logged = True
            last_action = chunk[-1].copy()
            last_action[0:6] = 0.0  # zero base velocity
            # Feed hold target through rate limiter for smooth deceleration
            if self._rate_limiter is not None:
                self._rate_limiter.set_target(last_action)
                hold_action = self._rate_limiter.step()
                if hold_action is not None:
                    last_action = hold_action
            obs = self.observation_bridge.get_latest_observation()
            current_state = obs.state if obs is not None else None
            self.action_publisher.publish_action(
                action=last_action,
                current_state=current_state,
                apply_smoothing=True,
                apply_safety=self.enable_safety_limits,
                chunk_id=self._total_chunks,
                action_idx=len(chunk) - 1,
            )
            return

        self._hold_logged = False

        # Clamp to valid range
        abs_idx = min(abs_idx, len(chunk) - 1)

        # Select raw target from chunk
        target = chunk[abs_idx].copy()

        # Action selection with chunk transition blend.
        # On promotion, the first step blends from old chunk's last action
        # to the new chunk's first executed action over ~67ms.
        if self._blend_from is not None:
            if exec_idx == 0:
                alpha = t_frac  # 0→1 over one step (~67ms)
                target = (1.0 - alpha) * self._blend_from + alpha * target
            else:
                self._blend_from = None  # blend complete

        # Smoothing pipeline: rate-limited interpolator OR legacy ActionInterpolator
        if self._rate_limiter is not None:
            # G1-style: feed target, step rate limiter at 100 Hz.
            # Position joints are velocity-capped; base velocity passes through.
            self._rate_limiter.set_target(target)
            action = self._rate_limiter.step()
            if action is None:
                action = target
        elif self.interpolate_actions and abs_idx < len(chunk) - 1:
            # Legacy: ActionInterpolator lerps between chunk actions
            for wi in range(max(0, abs_idx - 2), min(len(chunk), abs_idx + 4)):
                self._action_interpolator.update_waypoint(wi, chunk[wi])
            frac_t = abs_idx + (t_frac - int(t_frac))
            interp = self._action_interpolator.evaluate(frac_t)
            action = interp if interp is not None else target
        else:
            action = target

        # Log raw chunk action once per action step (before smoothing)
        obs = self.observation_bridge.get_latest_observation()
        current_state = obs.state if obs is not None else None
        self._log_action_step(
            self._total_chunks, abs_idx, chunk[abs_idx], current_state)

        # Publish action to robot at 100 Hz
        # apply_smoothing=True triggers base velocity limiting in ActionPublisher
        self.action_publisher.publish_action(
            action=action,
            current_state=current_state,
            apply_smoothing=True,
            apply_safety=self.enable_safety_limits,
            chunk_id=self._total_chunks,
            action_idx=abs_idx,
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
        self._close_action_csv()
        self.zmq_client.close()
        if self.async_client is not None:
            self.async_client.close()

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
