#!/usr/bin/env python3
"""GR00T N1.6 inference client node for Alfiebot.

This node connects to a remote GR00T inference server via ZeroMQ,
sends synchronized observations, and publishes action predictions
to control the robot.

Key timing:
- Inference runs on a background thread, paced to target_fps
- Command publishing runs at exactly 100 Hz on the ROS2 executor
- The two are decoupled so blocking inference never starves commands
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

from ..core.action_publisher import ActionPublisher
from ..core.observation_bridge import Observation, ObservationBridge
from ..core.zmq_client import ZMQClient, build_server_address, DEFAULT_IPC_PATH
from ..utils.safety import SafetyMonitor


# Command publishing rate (Hz) - must match robot expectations
COMMAND_RATE_HZ = 100


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
    - Action publishing to robot (100 Hz on ROS2 executor)
    - Safety monitoring and state management

    Normalization contract:
    - Raw state is sent to the server; Gr00tPolicy normalizes internally
    - Server returns raw/unnormalized actions; no denormalization needed

    Threading model:
    - Inference runs on a dedicated background thread, paced to target_fps.
      The blocking ZMQ call (~300ms) never stalls the ROS2 executor.
    - Command publishing runs at 100 Hz on the single-threaded ROS2 executor.
    - _action_lock protects the shared action/state between threads.
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
        self.action_execution_index = self.get_parameter('action_execution_index').value
        self.csv_log_path = self.get_parameter('csv_log_path').value

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

        # Current action to publish (protected by lock for thread safety)
        self._current_action: Optional[np.ndarray] = None
        self._current_state: Optional[np.ndarray] = None
        self._action_lock = threading.Lock()

        # Initialize safety monitor
        self.safety = SafetyMonitor(
            watchdog_timeout=0.5,
            max_consecutive_failures=5,
        )

        # Initialize ZMQ client
        self.zmq_client = ZMQClient(
            server_address=self.server_address,
            timeout_ms=self.inference_timeout_ms,
            logger=lambda msg: self.get_logger().info(msg),
        )

        # Initialize observation bridge
        self.observation_bridge = ObservationBridge(node=self)

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
        self.get_logger().info(f'  Action Exec Index:    {self.action_execution_index}')
        self.get_logger().info(f'  CSV Log Path:         {self.csv_log_path or "(disabled)"}')
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
        self.declare_parameter('action_smoothing_alpha', 0.7)
        self.declare_parameter('action_execution_index', 0)
        self.declare_parameter('csv_log_path', '')

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

        # Clear current action
        with self._action_lock:
            self._current_action = None
            self._current_state = None

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

        # Clear current action
        with self._action_lock:
            self._current_action = None
            self._current_state = None

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

        # Clear current action
        with self._action_lock:
            self._current_action = None
            self._current_state = None

        # Reset smoothing
        self.action_publisher.reset_smoothing()

    def _inference_loop(self):
        """Background inference loop (runs on dedicated thread).

        Continuously sends observations to the server and stores action
        predictions for the 100 Hz command loop on the ROS2 executor.
        Paced to target_fps; if inference takes longer, runs as fast as
        the server allows.
        """
        frame_period = 1.0 / self.target_fps

        while self._running:
            # Idle-poll when not active
            if not self._active or self._state != ClientState.ACTIVE:
                time.sleep(0.05)
                continue

            loop_start = time.monotonic()

            # Get latest observation
            obs = self.observation_bridge.get_latest_observation()
            if obs is None or not obs.valid:
                time.sleep(0.01)
                continue

            # Send raw state — Gr00tPolicy normalizes internally
            # This is the blocking call (~300ms) that motivated the thread
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
                        self._current_action = None
                continue

            # Update safety watchdog
            self.safety.update_inference_time()

            # Extract action from response
            action = self._extract_action(response)
            if action is None:
                self.get_logger().warn('Invalid action response from server')
                continue

            # Store action for command loop to publish at 100 Hz
            with self._action_lock:
                self._current_action = action
                self._current_state = obs.state.copy()

            # Pace to target_fps (sleep only if we finished faster)
            elapsed = time.monotonic() - loop_start
            sleep_time = frame_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _command_callback(self):
        """Command publishing callback (runs at exactly 100 Hz).

        Publishes the current action to /alfie/robotlowcmd at 100 Hz.
        Between inference updates, this holds/republishes the last action.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        # Get current action (thread-safe)
        with self._action_lock:
            action = self._current_action
            state = self._current_state

        if action is None:
            # No action yet, skip this cycle
            return

        # Publish action to robot at 100 Hz
        # Note: smoothing is applied here, so rapid republishing helps
        # maintain smooth servo motion
        self.action_publisher.publish_action(
            action=action,
            current_state=state,
            apply_smoothing=True,
            apply_safety=self.enable_safety_limits,
        )

    def _extract_action(self, response: dict) -> Optional[np.ndarray]:
        """Extract action from server response.

        Args:
            response: Server response dictionary with 'actions' key.

        Returns:
            Selected action from action horizon, or None on error.
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

        # Select action based on execution index
        action_idx = min(self.action_execution_index, len(actions) - 1)
        action = actions[action_idx]

        return np.array(action, dtype=np.float32)

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
