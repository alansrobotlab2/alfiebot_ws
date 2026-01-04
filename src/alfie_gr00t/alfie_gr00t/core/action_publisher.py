"""Action publisher for converting GR00T actions to ROS2 robot commands."""

from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from alfie_msgs.msg import BackCmd, RobotLowCmd, ServoCmd

from .normalization import Normalizer
from ..utils.safety import SafetyMonitor


class ActionPublisher:
    """Publishes GR00T action predictions to /alfie/robotlowcmd.

    Converts 21D action vectors to RobotLowCmd messages with:
    - Action denormalization
    - Exponential moving average smoothing
    - Safety limit enforcement
    - Servo parameter configuration
    """

    ACTION_DIM = 21
    NUM_SERVOS = 15

    def __init__(
        self,
        node: Node,
        cmd_topic: str = '/alfie/robotlowcmd',
        normalizer: Optional[Normalizer] = None,
        safety: Optional[SafetyMonitor] = None,
        smoothing_alpha: float = 0.7,
        default_servo_speed: float = 1.5,
        default_servo_acceleration: float = 5.0,
        default_servo_torque: float = 0.5,
    ):
        """Initialize action publisher.

        Args:
            node: ROS2 node for creating publisher.
            cmd_topic: Topic to publish RobotLowCmd.
            normalizer: Normalizer for denormalizing actions.
            safety: Safety monitor for limit enforcement.
            smoothing_alpha: EMA smoothing coefficient (0-1, higher = less smoothing).
            default_servo_speed: Default servo speed in rad/s.
            default_servo_acceleration: Default servo acceleration in rad/s^2.
            default_servo_torque: Default servo torque (0-1 fraction of max).
        """
        self.node = node
        self.normalizer = normalizer or Normalizer()
        self.safety = safety or SafetyMonitor()

        self.smoothing_alpha = smoothing_alpha
        self.default_servo_speed = default_servo_speed
        self.default_servo_acceleration = default_servo_acceleration
        self.default_servo_torque = default_servo_torque

        # QoS profile for commands (best effort for real-time)
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create publisher
        self.cmd_pub = node.create_publisher(RobotLowCmd, cmd_topic, qos_cmd)

        # Action smoothing state
        self._last_action: Optional[np.ndarray] = None
        self._last_state: Optional[np.ndarray] = None

        # Statistics
        self._publish_count = 0

    def reset_smoothing(self):
        """Reset action smoothing state."""
        self._last_action = None
        self._last_state = None

    def publish_action(
        self,
        action: np.ndarray,
        current_state: Optional[np.ndarray] = None,
        normalized: bool = True,
        apply_smoothing: bool = True,
        apply_safety: bool = True,
    ) -> bool:
        """Publish action to robot.

        Args:
            action: 21D action vector.
            current_state: Current 21D state vector (for delta limits).
            normalized: Whether action is normalized (needs denormalization).
            apply_smoothing: Whether to apply EMA smoothing.
            apply_safety: Whether to apply safety limits.

        Returns:
            True if action was published, False if blocked by safety.
        """
        action = np.asarray(action, dtype=np.float32).flatten()

        if len(action) != self.ACTION_DIM:
            self.node.get_logger().error(
                f'Action dimension mismatch: expected {self.ACTION_DIM}, got {len(action)}'
            )
            return False

        # Denormalize if needed
        if normalized:
            action = self.normalizer.denormalize_action(action)

        # Apply smoothing
        if apply_smoothing and self._last_action is not None:
            action = (
                self.smoothing_alpha * action +
                (1 - self.smoothing_alpha) * self._last_action
            )

        # Apply safety limits
        if apply_safety:
            # Check if safe to publish
            if not self.safety.is_safe():
                self.node.get_logger().warn('Safety check failed, not publishing')
                return False

            # Apply velocity and joint limits
            action = self.safety.apply_limits(action)

            # Apply delta limits if we have current state
            if current_state is not None:
                action = self.safety.compute_delta_limits(action, current_state)

        # Store for next smoothing iteration
        self._last_action = action.copy()
        if current_state is not None:
            self._last_state = np.asarray(current_state, dtype=np.float32)

        # Build and publish message
        msg = self._build_robot_low_cmd(action)
        self.cmd_pub.publish(msg)

        self._publish_count += 1
        return True

    def publish_stop(self):
        """Publish a stop command (zero velocities, hold positions)."""
        msg = RobotLowCmd()

        # Zero base velocity
        msg.cmd_vel = Twist()

        # Hold current positions (send empty servo commands with enabled=False)
        for i in range(self.NUM_SERVOS):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = False
            servo_cmd.target_location = 0.0
            servo_cmd.target_speed = 0.0
            servo_cmd.target_acceleration = 0.0
            servo_cmd.target_torque = 0.0
            msg.servo_cmd[i] = servo_cmd

        # Zero back command
        msg.back_cmd = BackCmd()

        self.cmd_pub.publish(msg)

    def _build_robot_low_cmd(self, action: np.ndarray) -> RobotLowCmd:
        """Build RobotLowCmd message from action vector.

        Action vector layout:
        [0-5]:   cmd_vel (linear.x,y,z, angular.x,y,z)
        [6-11]:  servo_cmd[0-5].target_location (left arm)
        [12-17]: servo_cmd[6-11].target_location (right arm)
        [18-20]: servo_cmd[12-14].target_location (head)

        Args:
            action: 21D denormalized action vector.

        Returns:
            RobotLowCmd message.
        """
        msg = RobotLowCmd()

        # Base velocity (indices 0-5)
        msg.cmd_vel.linear.x = float(action[0])
        msg.cmd_vel.linear.y = float(action[1])
        msg.cmd_vel.linear.z = float(action[2])
        msg.cmd_vel.angular.x = float(action[3])
        msg.cmd_vel.angular.y = float(action[4])
        msg.cmd_vel.angular.z = float(action[5])

        # Servo commands (15 total)
        for i in range(self.NUM_SERVOS):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = True
            servo_cmd.target_location = float(action[6 + i])
            servo_cmd.target_speed = self.default_servo_speed
            servo_cmd.target_acceleration = self.default_servo_acceleration
            servo_cmd.target_torque = self.default_servo_torque
            msg.servo_cmd[i] = servo_cmd

        # Back command (not controlled by policy, use defaults)
        msg.back_cmd = BackCmd()

        # Eye PWM (not controlled by policy)
        msg.eye_pwm = [0, 0]

        return msg

    def get_stats(self) -> dict:
        """Get publisher statistics.

        Returns:
            Dictionary with publishing stats.
        """
        return {
            'publish_count': self._publish_count,
            'has_last_action': self._last_action is not None,
            'smoothing_alpha': self.smoothing_alpha,
        }
