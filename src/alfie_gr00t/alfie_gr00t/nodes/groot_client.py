#!/usr/bin/env python3
"""GR00T N1.6 inference client node for Alfiebot.

This node connects to a remote GR00T inference server via ZeroMQ,
sends synchronized observations, and publishes action predictions
to control the robot.

Key timing:
- Inference runs at 15 FPS (configurable)
- Command publishing runs at exactly 100 Hz to match robot expectations
"""

from enum import Enum
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from ..core.action_publisher import ActionPublisher
from ..core.normalization import Normalizer
from ..core.observation_bridge import Observation, ObservationBridge
from ..core.zmq_client import ZMQClient, build_server_address, DEFAULT_IPC_PATH
from ..utils.safety import SafetyMonitor


# Command publishing rate (Hz) - must match robot expectations
COMMAND_RATE_HZ = 100


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
    - ZeroMQ communication with inference server (15 FPS)
    - Action publishing to robot (100 Hz)
    - Safety monitoring and state management

    The inference loop runs at target_fps (default 15) to get new actions
    from the server. The command loop runs at 100 Hz to publish commands
    to the robot, holding/interpolating the last received action between
    inference updates.
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
        self.stats_file = self.get_parameter('stats_file').value

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

        # Current action to publish (protected by lock for thread safety)
        self._current_action: Optional[np.ndarray] = None
        self._current_state: Optional[np.ndarray] = None
        self._action_lock = threading.Lock()

        # Initialize normalizer
        self.normalizer = Normalizer(self.stats_file)
        if self.normalizer.is_loaded:
            self.get_logger().info(f'Loaded normalization stats from: {self.stats_file}')
        else:
            self.get_logger().warn(f'Could not load stats from: {self.stats_file}')

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
        self.observation_bridge = ObservationBridge(
            node=self,
            normalizer=self.normalizer,
        )

        # Initialize action publisher
        self.action_publisher = ActionPublisher(
            node=self,
            normalizer=self.normalizer,
            safety=self.safety if self.enable_safety_limits else None,
            smoothing_alpha=self.action_smoothing_alpha,
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

        # Inference loop timer (15 FPS - gets new actions from server)
        self.inference_timer = self.create_timer(
            1.0 / self.target_fps,
            self._inference_callback,
        )

        # Command publishing timer (100 Hz - sends commands to robot)
        self.command_timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ,
            self._command_callback,
        )

        # Status publishing timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'GR00T Client initialized: server={self.server_address}, '
            f'inference_fps={self.target_fps}, command_hz={COMMAND_RATE_HZ}, '
            f'task="{self.task_description}"'
        )

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        # Transport configuration
        # Use "ipc" for on-device inference (faster), "tcp" for remote server
        self.declare_parameter('transport', 'ipc')
        self.declare_parameter('server_host', '192.168.1.100')
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('ipc_path', DEFAULT_IPC_PATH)

        # Inference settings
        self.declare_parameter('target_fps', 15)
        self.declare_parameter('inference_timeout_ms', 100)
        self.declare_parameter('task_description', 'find the can and pick it up')
        self.declare_parameter('enable_safety_limits', True)
        self.declare_parameter('action_smoothing_alpha', 0.7)
        self.declare_parameter('action_execution_index', 0)
        self.declare_parameter(
            'stats_file',
            '/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge/meta/stats.jsonl'
        )

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

    def _inference_callback(self):
        """Inference loop callback (runs at target_fps, e.g., 15 FPS).

        Gets new action predictions from the server and stores them
        for the command loop to publish at 100 Hz.
        """
        if not self._active or self._state != ClientState.ACTIVE:
            return

        # Get latest observation
        obs = self.observation_bridge.get_latest_observation()
        if obs is None or not obs.valid:
            return

        # Send observation to server
        response = self.zmq_client.send_observation(
            images=obs.images,
            state=obs.state_normalized,
            language=self.task_description,
        )

        if response is None:
            # Inference failed
            self.safety.record_failure()

            if self.safety.in_safe_mode:
                self.get_logger().error('Too many failures, entering safe mode')
                self._state = ClientState.ERROR
                self._active = False
                self.action_publisher.publish_stop()
                with self._action_lock:
                    self._current_action = None
            return

        # Update safety watchdog
        self.safety.update_inference_time()

        # Extract action from response
        action = self._extract_action(response)
        if action is None:
            self.get_logger().warn('Invalid action response from server')
            return

        # Store action for command loop to publish at 100 Hz
        with self._action_lock:
            self._current_action = action
            self._current_state = obs.state.copy()

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
            normalized=True,  # Server returns normalized actions
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

        # Send stop command
        self.action_publisher.publish_stop()

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
