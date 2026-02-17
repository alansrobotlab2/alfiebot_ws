"""Action publisher for converting GR00T actions to ROS2 robot commands.

Includes:
- Base velocity capping and acceleration limiting (BEHAVIOR R1Pro-inspired)
- Per-joint servo speed/acceleration/torque configuration
- CSV logging at 100 Hz for diagnostics
"""

import csv
import time
from pathlib import Path
from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from alfie_msgs.msg import BackCmd, RobotLowCmd, ServoCmd

from ..utils.safety import SafetyMonitor


# Per-servo config: (speed rad/s, acceleration rad/s², torque 0-1)
# Indexed by servo number (0-14)
DEFAULT_SERVO_CONFIG = {
    # Left arm (servos 0-4): fast for manipulation
    0: (2.0, 5.0, 0.5),
    1: (2.0, 5.0, 0.5),
    2: (2.0, 5.0, 0.5),
    3: (2.0, 5.0, 0.5),
    4: (2.0, 5.0, 0.5),
    # Left gripper (servo 5): quick open/close
    5: (3.0, 8.0, 0.4),
    # Right arm (servos 6-10): fast for manipulation
    6: (2.0, 5.0, 0.5),
    7: (2.0, 5.0, 0.5),
    8: (2.0, 5.0, 0.5),
    9: (2.0, 5.0, 0.5),
    10: (2.0, 5.0, 0.5),
    # Right gripper (servo 11): quick open/close
    11: (3.0, 8.0, 0.4),
    # Head (servos 12-14): slow, smooth tracking
    12: (1.0, 3.0, 0.3),
    13: (1.0, 3.0, 0.3),
    14: (1.0, 3.0, 0.3),
}


class ActionPublisher:
    """Publishes GR00T action predictions to /alfie/robotlowcmd.

    Converts 22D raw action vectors (physical units) to RobotLowCmd messages with:
    - Base velocity capping and acceleration limiting
    - Per-joint servo speed/acceleration/torque
    - Safety limit enforcement
    - CSV logging
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
        safety: Optional[SafetyMonitor] = None,
        base_smoothing_alpha: float = 1.0,
        joint_smoothing_alpha: float = 0.95,
        default_servo_speed: float = 1.5,
        default_servo_acceleration: float = 5.0,
        default_servo_torque: float = 0.5,
        csv_log_path: str = '',
        # Base velocity limits (BEHAVIOR-inspired)
        max_base_linear_x: float = 0.15,
        max_base_linear_y: float = 0.15,
        max_base_angular_z: float = 0.8,
        max_base_linear_accel: float = 0.3,
        max_base_angular_accel: float = 1.5,
        # Per-group servo config overrides
        servo_speed_arms: float = 2.0,
        servo_speed_grippers: float = 3.0,
        servo_speed_head: float = 1.0,
        servo_accel_arms: float = 5.0,
        servo_accel_grippers: float = 8.0,
        servo_accel_head: float = 3.0,
        servo_torque_arms: float = 0.5,
        servo_torque_grippers: float = 0.4,
        servo_torque_head: float = 0.3,
    ):
        self.node = node
        self.safety = safety or SafetyMonitor()

        # Legacy EMA params (kept for backward compat, default 1.0 = disabled)
        self.base_smoothing_alpha = base_smoothing_alpha
        self.joint_smoothing_alpha = joint_smoothing_alpha

        # Base velocity limits
        self._max_base_vel = np.array([
            max_base_linear_x,   # lx
            max_base_linear_y,   # ly
            0.5,                 # lz (rarely used, generous limit)
            1.5,                 # ax (rarely used)
            1.5,                 # ay (rarely used)
            max_base_angular_z,  # az
        ], dtype=np.float32)
        self._max_base_linear_accel = max_base_linear_accel
        self._max_base_angular_accel = max_base_angular_accel
        self._base_dt = 1.0 / 100.0  # 100 Hz command rate

        # Build per-servo config from group parameters
        self._servo_config = {}
        for i in range(5):      # left arm servos 0-4
            self._servo_config[i] = (servo_speed_arms, servo_accel_arms, servo_torque_arms)
        self._servo_config[5] = (servo_speed_grippers, servo_accel_grippers, servo_torque_grippers)
        for i in range(6, 11):  # right arm servos 6-10
            self._servo_config[i] = (servo_speed_arms, servo_accel_arms, servo_torque_arms)
        self._servo_config[11] = (servo_speed_grippers, servo_accel_grippers, servo_torque_grippers)
        for i in range(12, 15): # head servos 12-14
            self._servo_config[i] = (servo_speed_head, servo_accel_head, servo_torque_head)

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
        self._last_base_vel: Optional[np.ndarray] = None

        # Statistics
        self._publish_count = 0
        self._inference_step = 0
        self._base_clips = 0
        self._accel_clips = 0

        # CSV logging
        self._csv_file = None
        self._csv_writer = None
        if csv_log_path:
            self._init_csv_log(csv_log_path)

    def reset_smoothing(self):
        """Reset action smoothing state."""
        self._last_action = None
        self._last_state = None
        self._last_base_vel = None

    def _limit_base_velocity(self, base_vel: np.ndarray) -> np.ndarray:
        """Clamp base velocity magnitude and acceleration.

        BEHAVIOR R1Pro-inspired: magnitude capping + acceleration limiting.
        Physics-based, no phase lag (unlike EMA).
        """
        result = base_vel.copy()

        # 1. Clamp magnitude per-axis
        clipped = np.clip(result, -self._max_base_vel, self._max_base_vel)
        if not np.array_equal(result, clipped):
            self._base_clips += 1
        result = clipped

        # 2. Clamp acceleration (dv/dt) to prevent jerk
        if self._last_base_vel is not None:
            dv = result - self._last_base_vel
            max_dv_linear = self._max_base_linear_accel * self._base_dt
            max_dv_angular = self._max_base_angular_accel * self._base_dt
            dv[0:3] = np.clip(dv[0:3], -max_dv_linear, max_dv_linear)
            dv[3:6] = np.clip(dv[3:6], -max_dv_angular, max_dv_angular)
            new_result = self._last_base_vel + dv
            if not np.allclose(result, new_result, atol=1e-6):
                self._accel_clips += 1
            result = new_result

        self._last_base_vel = result.copy()
        return result

    def _init_csv_log(self, csv_log_path: str):
        """Initialize CSV log file with headers."""
        path = Path(csv_log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)

        # Build header row
        header = ['timestamp', 'step', 'chunk_id', 'action_idx']
        for prefix in ['action', 'smoothed', 'state']:
            for name in self.JOINT_NAMES:
                header.append(f'{prefix}_{name}')
        self._csv_writer.writerow(header)
        self._csv_file.flush()
        self.node.get_logger().info(f'CSV logging enabled: {csv_log_path}')

    def _write_csv_row(
        self,
        action: np.ndarray,
        smoothed: np.ndarray,
        state: Optional[np.ndarray],
        chunk_id: int = 0,
        action_idx: int = 0,
    ):
        """Write one row to the CSV log."""
        if self._csv_writer is None:
            return
        state_vals = state if state is not None else np.zeros(self.ACTION_DIM)
        row = [time.time(), self._inference_step, chunk_id, action_idx]
        for arr in [action, smoothed, state_vals]:
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
        apply_smoothing: bool = True,
        apply_safety: bool = True,
        max_joint_delta: float = 0.3,
        chunk_id: int = 0,
        action_idx: int = 0,
    ) -> bool:
        """Publish action to robot.

        Actions are expected in raw physical units (server returns unnormalized).
        Base velocity is magnitude-capped and acceleration-limited.
        Joint positions pass through (rate limiting handled by RateLimitedInterpolator
        upstream in groot_client.py).

        Args:
            action: 22D action vector in raw physical units.
            current_state: Current 22D state vector (for delta limits).
            apply_smoothing: Whether to apply base velocity limiting.
            apply_safety: Whether to apply safety limits.
            max_joint_delta: Maximum joint position change per step (radians).
            chunk_id: Which inference chunk this action comes from.
            action_idx: Action index (0-15) within the chunk.

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

        # Apply base velocity limiting (magnitude cap + acceleration limit)
        if apply_smoothing:
            pre_limit = action[0:6].copy()
            action[0:6] = self._limit_base_velocity(action[0:6])
            if log_this:
                logger.info(
                    f'[base_debug] limited base[0:6]='
                    f'{np.array2string(action[0:6], precision=4, suppress_small=True)}'
                    f' (pre_limit={np.array2string(pre_limit, precision=4, suppress_small=True)})'
                )

        # Capture smoothed action for CSV
        smoothed_action = action.copy()

        # Safety check: e-stop and watchdog only (hardware enforces joint limits)
        if apply_safety:
            reason = self.safety.unsafe_reason()
            if reason is not None:
                logger.warn(f'Safety check failed: {reason}')
                return False

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
                action=raw_action,
                smoothed=smoothed_action,
                state=current_state,
                chunk_id=chunk_id,
                action_idx=action_idx,
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

        # Reset base velocity tracking
        self._last_base_vel = None

        self.cmd_pub.publish(msg)

    def _build_servo_cmd(self, servo_idx: int, position: float) -> ServoCmd:
        """Build a ServoCmd with per-joint speed/accel/torque config."""
        speed, accel, torque = self._servo_config.get(
            servo_idx, (1.5, 5.0, 0.5))
        cmd = ServoCmd()
        cmd.enabled = True
        cmd.target_location = position
        cmd.target_speed = speed
        cmd.target_acceleration = accel
        cmd.target_torque = torque
        return cmd

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
            msg.servo_cmd[i] = self._build_servo_cmd(i, float(action[7 + i]))

        # Left gripper (index 12 -> servo 5)
        msg.servo_cmd[5] = self._build_servo_cmd(5, float(action[12]))

        # Right arm servos (indices 13-17 -> servos 6-10)
        for i in range(5):
            msg.servo_cmd[6 + i] = self._build_servo_cmd(6 + i, float(action[13 + i]))

        # Right gripper (index 18 -> servo 11)
        msg.servo_cmd[11] = self._build_servo_cmd(11, float(action[18]))

        # Head servos (indices 19-21 -> servos 12-14)
        for i in range(3):
            msg.servo_cmd[12 + i] = self._build_servo_cmd(12 + i, float(action[19 + i]))

        # Eye PWM (not controlled by policy)
        msg.eye_pwm = [0, 0]

        return msg

    def get_stats(self) -> dict:
        """Get publisher statistics."""
        return {
            'publish_count': self._publish_count,
            'has_last_action': self._last_action is not None,
            'base_smoothing_alpha': self.base_smoothing_alpha,
            'joint_smoothing_alpha': self.joint_smoothing_alpha,
            'base_vel_clips': self._base_clips,
            'base_accel_clips': self._accel_clips,
        }
