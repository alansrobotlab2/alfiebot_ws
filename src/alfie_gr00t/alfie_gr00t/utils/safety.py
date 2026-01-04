"""Safety monitoring and limit enforcement for GR00T client."""

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class JointLimits:
    """Joint position limits in radians."""

    min_pos: float
    max_pos: float

    def clip(self, value: float) -> float:
        """Clip value to limits."""
        return np.clip(value, self.min_pos, self.max_pos)


class SafetyMonitor:
    """Safety monitoring and limit enforcement.

    Provides:
    - E-stop state tracking
    - Watchdog timer for inference failures
    - Joint position limits
    - Velocity limits
    - Graceful degradation on errors
    """

    # Joint limits (radians) - from servo observations and hardware specs
    # Indices match servo_state/servo_cmd array order
    JOINT_LIMITS = {
        # Left arm (servos 0-5)
        0: JointLimits(-1.5, 1.5),    # left_shoulder_yaw
        1: JointLimits(-1.0, 1.5),    # left_shoulder_pitch
        2: JointLimits(-1.6, 1.6),    # left_elbow_pitch
        3: JointLimits(-1.6, 1.6),    # left_wrist_pitch
        4: JointLimits(-1.6, 1.6),    # left_wrist_roll
        5: JointLimits(0.0, 0.8),     # left_gripper (open/close)
        # Right arm (servos 6-11)
        6: JointLimits(-1.5, 1.5),    # right_shoulder_yaw
        7: JointLimits(-1.5, 1.0),    # right_shoulder_pitch
        8: JointLimits(-1.6, 1.6),    # right_elbow_pitch
        9: JointLimits(-1.6, 2.7),    # right_wrist_pitch (extended range)
        10: JointLimits(-1.6, 0.8),   # right_wrist_roll
        11: JointLimits(0.0, 0.8),    # right_gripper
        # Head (servos 12-14)
        12: JointLimits(-1.0, 0.5),   # head_yaw
        13: JointLimits(-1.1, 0.2),   # head_pitch
        14: JointLimits(-0.2, 0.1),   # head_roll
    }

    # Velocity limits
    MAX_LINEAR_VEL = 0.15   # m/s (conservative for safety)
    MAX_ANGULAR_VEL = 0.5   # rad/s

    def __init__(
        self,
        watchdog_timeout: float = 0.5,
        max_consecutive_failures: int = 5,
    ):
        """Initialize safety monitor.

        Args:
            watchdog_timeout: Seconds before watchdog triggers on no inference.
            max_consecutive_failures: Max failures before entering safe mode.
        """
        self.watchdog_timeout = watchdog_timeout
        self.max_consecutive_failures = max_consecutive_failures

        # State tracking
        self._e_stop_active = False
        self._last_inference_time: Optional[float] = None
        self._consecutive_failures = 0
        self._in_safe_mode = False

    @property
    def e_stop_active(self) -> bool:
        """Check if E-stop is currently active."""
        return self._e_stop_active

    @e_stop_active.setter
    def e_stop_active(self, value: bool):
        """Set E-stop state."""
        self._e_stop_active = value

    @property
    def in_safe_mode(self) -> bool:
        """Check if in safe mode due to repeated failures."""
        return self._in_safe_mode

    def update_inference_time(self):
        """Update last successful inference timestamp."""
        self._last_inference_time = time.monotonic()
        self._consecutive_failures = 0
        self._in_safe_mode = False

    def record_failure(self):
        """Record an inference failure."""
        self._consecutive_failures += 1
        if self._consecutive_failures >= self.max_consecutive_failures:
            self._in_safe_mode = True

    def reset_failures(self):
        """Reset failure counter."""
        self._consecutive_failures = 0
        self._in_safe_mode = False

    def is_safe(self) -> bool:
        """Check if it's safe to publish commands.

        Returns:
            True if safe to publish, False otherwise.
        """
        # E-stop check
        if self._e_stop_active:
            return False

        # Safe mode check
        if self._in_safe_mode:
            return False

        # Watchdog check
        if self._last_inference_time is not None:
            elapsed = time.monotonic() - self._last_inference_time
            if elapsed > self.watchdog_timeout:
                return False

        return True

    def get_status(self) -> dict:
        """Get current safety status.

        Returns:
            Dictionary with safety state information.
        """
        elapsed = None
        if self._last_inference_time is not None:
            elapsed = time.monotonic() - self._last_inference_time

        return {
            'e_stop_active': self._e_stop_active,
            'in_safe_mode': self._in_safe_mode,
            'consecutive_failures': self._consecutive_failures,
            'time_since_inference': elapsed,
            'watchdog_timeout': self.watchdog_timeout,
            'is_safe': self.is_safe(),
        }

    def apply_velocity_limits(self, action: np.ndarray) -> np.ndarray:
        """Apply velocity limits to base velocity commands.

        Args:
            action: 21D action vector.

        Returns:
            Action vector with velocity limits applied.
        """
        action = action.copy()

        # Linear velocity limits (indices 0-2)
        action[0] = np.clip(action[0], -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        action[1] = np.clip(action[1], -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)
        action[2] = np.clip(action[2], -self.MAX_LINEAR_VEL, self.MAX_LINEAR_VEL)

        # Angular velocity limits (indices 3-5)
        # Only z-axis rotation is typically used for mobile base
        action[3] = np.clip(action[3], -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
        action[4] = np.clip(action[4], -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)
        action[5] = np.clip(action[5], -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        return action

    def apply_joint_limits(self, action: np.ndarray) -> np.ndarray:
        """Apply joint position limits to servo commands.

        Args:
            action: 21D action vector.

        Returns:
            Action vector with joint limits applied.
        """
        action = action.copy()

        # Joint positions are at indices 6-20 (15 servos)
        for servo_idx, limits in self.JOINT_LIMITS.items():
            action_idx = 6 + servo_idx
            action[action_idx] = limits.clip(action[action_idx])

        return action

    def apply_limits(self, action: np.ndarray) -> np.ndarray:
        """Apply all safety limits to action vector.

        Args:
            action: 21D action vector (denormalized).

        Returns:
            Action vector with all limits applied.
        """
        action = self.apply_velocity_limits(action)
        action = self.apply_joint_limits(action)
        return action

    def compute_delta_limits(
        self,
        action: np.ndarray,
        current_state: np.ndarray,
        max_joint_delta: float = 0.3,
    ) -> np.ndarray:
        """Apply delta limits to prevent large sudden movements.

        Args:
            action: Target action (21D).
            current_state: Current state (21D).
            max_joint_delta: Maximum allowed joint position change per step (radians).

        Returns:
            Action with delta limits applied.
        """
        action = action.copy()

        # Apply delta limits to joint positions (indices 6-20)
        for i in range(6, 21):
            delta = action[i] - current_state[i]
            if abs(delta) > max_joint_delta:
                action[i] = current_state[i] + np.sign(delta) * max_joint_delta

        return action
