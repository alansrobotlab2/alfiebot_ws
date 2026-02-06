"""Action publisher for converting GR00T actions to ROS2 robot commands."""

import csv
import time
from pathlib import Path
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

    Converts 22D action vectors to RobotLowCmd messages with:
    - Action denormalization
    - Exponential moving average smoothing
    - Safety limit enforcement
    - Servo parameter configuration
    """

    ACTION_DIM = 22
    NUM_SERVOS = 15

    # Joint names matching the 22D action vector layout
    JOINT_NAMES = [
        'cmd_vel_lx', 'cmd_vel_ly', 'cmd_vel_lz',
        'cmd_vel_ax', 'cmd_vel_ay', 'cmd_vel_az',
        'back_joint',
        'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow_pitch',
        'left_wrist_pitch', 'left_wrist_roll', 'left_gripper',
        'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow_pitch',
        'right_wrist_pitch', 'right_wrist_roll', 'right_gripper',
        'head_yaw', 'head_pitch', 'head_roll',
    ]

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
        csv_log_path: str = '',
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
        self._inference_step = 0

        # CSV logging
        self._csv_file = None
        self._csv_writer = None
        if csv_log_path:
            self._init_csv_log(csv_log_path)

    def reset_smoothing(self):
        """Reset action smoothing state."""
        self._last_action = None
        self._last_state = None

    def _init_csv_log(self, csv_log_path: str):
        """Initialize CSV log file with headers."""
        path = Path(csv_log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)

        # Build header row
        header = ['timestamp', 'step']
        for prefix in ['raw', 'denorm', 'smoothed', 'final', 'state']:
            for name in self.JOINT_NAMES:
                header.append(f'{prefix}_{name}')
        self._csv_writer.writerow(header)
        self._csv_file.flush()
        self.node.get_logger().info(f'CSV logging enabled: {csv_log_path}')

    def _write_csv_row(
        self,
        raw: np.ndarray,
        denorm: np.ndarray,
        smoothed: np.ndarray,
        final: np.ndarray,
        state: Optional[np.ndarray],
    ):
        """Write one row to the CSV log."""
        if self._csv_writer is None:
            return
        state_vals = state if state is not None else np.zeros(self.ACTION_DIM)
        row = [time.time(), self._inference_step]
        for arr in [raw, denorm, smoothed, final, state_vals]:
            row.extend(arr.tolist())
        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self._inference_step += 1

    def close_csv(self):
        """Close CSV log file."""
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None

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
            action: 22D action vector.
            current_state: Current 22D state vector (for delta limits).
            normalized: Whether action is normalized (needs denormalization).
            apply_smoothing: Whether to apply EMA smoothing.
            apply_safety: Whether to apply safety limits.

        Returns:
            True if action was published, False if blocked by safety.
        """
        logger = self.node.get_logger()
        action = np.asarray(action, dtype=np.float32).flatten()

        if len(action) != self.ACTION_DIM:
            logger.error(
                f'Action dimension mismatch: expected {self.ACTION_DIM}, got {len(action)}'
            )
            return False

        # Log action base values (every 100th publish to avoid spam)
        log_this = (self._publish_count % 100 == 0)

        # Capture input action for CSV
        raw_action = action.copy()

        if log_this:
            logger.info(
                f'[base_debug] input_action base[0:6]='
                f'{np.array2string(action[0:6], precision=4, suppress_small=True)}'
            )

        # Denormalize if needed
        if normalized:
            action = self.normalizer.denormalize_action(action)
            if log_this:
                logger.info(
                    f'[base_debug] denorm base[0:6]='
                    f'{np.array2string(action[0:6], precision=4, suppress_small=True)}'
                )

        # Capture denormalized action for CSV
        denorm_action = action.copy()

        # Apply smoothing
        if apply_smoothing and self._last_action is not None:
            pre_smooth = action[0:6].copy()
            action = (
                self.smoothing_alpha * action +
                (1 - self.smoothing_alpha) * self._last_action
            )
            if log_this:
                logger.info(
                    f'[base_debug] smoothed base[0:6]='
                    f'{np.array2string(action[0:6], precision=4, suppress_small=True)}'
                    f' (pre_smooth={np.array2string(pre_smooth, precision=4, suppress_small=True)})'
                )

        # Capture smoothed action for CSV
        smoothed_action = action.copy()

        # Apply safety limits
        if apply_safety:
            # Check if safe to publish
            if not self.safety.is_safe():
                logger.warn('Safety check failed, not publishing')
                return False

            pre_safety = action[0:6].copy()
            # Apply velocity and joint limits
            action = self.safety.apply_limits(action)

            # Apply delta limits if we have current state
            if current_state is not None:
                action = self.safety.compute_delta_limits(action, current_state)

            if log_this:
                logger.info(
                    f'[base_debug] post_safety base[0:6]='
                    f'{np.array2string(action[0:6], precision=4, suppress_small=True)}'
                    f' (pre_safety={np.array2string(pre_safety, precision=4, suppress_small=True)})'
                )

        # Final twist values that will be published
        if log_this:
            logger.info(
                f'[base_debug] FINAL twist: '
                f'lin=({action[0]:.4f}, {action[1]:.4f}, {action[2]:.4f}) '
                f'ang=({action[3]:.4f}, {action[4]:.4f}, {action[5]:.4f}) '
                f'[publish #{self._publish_count}]'
            )

        # Write CSV row on every publish (100Hz) for full fidelity
        if self._csv_writer is not None:
            self._write_csv_row(
                raw=raw_action,
                denorm=denorm_action,
                smoothed=smoothed_action,
                final=action,
                state=current_state,
            )

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
        [6]:     back_cmd.command_position
        [7-11]:  servo_cmd[0-4].target_location (left arm)
        [12]:    servo_cmd[5].target_location (left gripper)
        [13-17]: servo_cmd[6-10].target_location (right arm)
        [18]:    servo_cmd[11].target_location (right gripper)
        [19-21]: servo_cmd[12-14].target_location (head)

        Args:
            action: 22D denormalized action vector.

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

        # Back command (index 6)
        msg.back_cmd = BackCmd()
        msg.back_cmd.position = float(action[6])

        # Left arm servos (indices 7-11 -> servos 0-4)
        for i in range(5):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = True
            servo_cmd.target_location = float(action[7 + i])
            servo_cmd.target_speed = self.default_servo_speed
            servo_cmd.target_acceleration = self.default_servo_acceleration
            servo_cmd.target_torque = self.default_servo_torque
            msg.servo_cmd[i] = servo_cmd

        # Left gripper (index 12 -> servo 5)
        servo_cmd = ServoCmd()
        servo_cmd.enabled = True
        servo_cmd.target_location = float(action[12])
        servo_cmd.target_speed = self.default_servo_speed
        servo_cmd.target_acceleration = self.default_servo_acceleration
        servo_cmd.target_torque = self.default_servo_torque
        msg.servo_cmd[5] = servo_cmd

        # Right arm servos (indices 13-17 -> servos 6-10)
        for i in range(5):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = True
            servo_cmd.target_location = float(action[13 + i])
            servo_cmd.target_speed = self.default_servo_speed
            servo_cmd.target_acceleration = self.default_servo_acceleration
            servo_cmd.target_torque = self.default_servo_torque
            msg.servo_cmd[6 + i] = servo_cmd

        # Right gripper (index 18 -> servo 11)
        servo_cmd = ServoCmd()
        servo_cmd.enabled = True
        servo_cmd.target_location = float(action[18])
        servo_cmd.target_speed = self.default_servo_speed
        servo_cmd.target_acceleration = self.default_servo_acceleration
        servo_cmd.target_torque = self.default_servo_torque
        msg.servo_cmd[11] = servo_cmd

        # Head servos (indices 19-21 -> servos 12-14)
        for i in range(3):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = True
            servo_cmd.target_location = float(action[19 + i])
            servo_cmd.target_speed = self.default_servo_speed
            servo_cmd.target_acceleration = self.default_servo_acceleration
            servo_cmd.target_torque = self.default_servo_torque
            msg.servo_cmd[12 + i] = servo_cmd

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
