"""G1-style rate-limited interpolation for position-commanded joints.

Inspired by GR00T WholeBodyControl's InterpolationPolicy: enforces per-joint
maximum velocity limits to prevent jerky transitions at chunk boundaries.
Each joint moves independently at up to its configured max speed.

Operates on the full 22D action vector:
- Base velocity (indices 0:6): passed through unchanged (velocity commands)
- Position joints (indices 6:22): rate-limited interpolation from current
  interpolated position to target

Used by groot_client.py at 100 Hz. Replaces ActionInterpolator for the
classic execution mode.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


# Default max speeds per body part (rad/s for joints, m/s for back)
DEFAULT_MAX_SPEEDS = {
    'back': 0.3,           # m/s — slow, heavy linear actuator
    'left_arm': 2.0,       # rad/s — fast for manipulation
    'left_gripper': 3.0,   # rad/s — quick open/close
    'right_arm': 2.0,      # rad/s
    'right_gripper': 3.0,  # rad/s
    'head': 1.0,           # rad/s — slow, smooth tracking
}

# Mapping from body part name to 22D action vector indices
BODY_PART_INDICES = {
    'back': [6],
    'left_arm': [7, 8, 9, 10, 11],
    'left_gripper': [12],
    'right_arm': [13, 14, 15, 16, 17],
    'right_gripper': [18],
    'head': [19, 20, 21],
}

# Indices that are position-commanded (not velocity)
POSITION_INDICES = list(range(6, 22))

# Indices that are velocity-commanded (base)
BASE_INDICES = list(range(0, 6))


class RateLimitedInterpolator:
    """Per-joint velocity-capped interpolation for smooth 100 Hz output.

    On each tick (10ms at 100 Hz), each position joint moves toward its
    target by at most max_speed * dt. This prevents discontinuous jumps
    at chunk boundaries while being transparent within chunks (where
    consecutive targets are already smooth from flow matching).

    Base velocity indices pass through unchanged — acceleration limiting
    for the base is handled separately by ActionPublisher.

    Parameters
    ----------
    max_speeds : dict
        Body part name → max speed (rad/s or m/s). See DEFAULT_MAX_SPEEDS.
    dt : float
        Control period in seconds (0.01 for 100 Hz).
    """

    ACTION_DIM = 22

    def __init__(
        self,
        max_speeds: Optional[dict] = None,
        dt: float = 0.01,
        target_ema_alpha: float = 1.0,
    ):
        speeds = dict(DEFAULT_MAX_SPEEDS)
        if max_speeds is not None:
            speeds.update(max_speeds)

        self._dt = dt
        self._ema_alpha = target_ema_alpha

        # Build per-index max delta (speed * dt) array
        # Index 0:6 = 0.0 (base velocity: passthrough)
        # Index 6:22 = configured max_speed * dt
        self._max_delta = np.zeros(self.ACTION_DIM, dtype=np.float64)
        for part_name, indices in BODY_PART_INDICES.items():
            speed = speeds.get(part_name, 1.5)
            for idx in indices:
                self._max_delta[idx] = speed * dt

        # Current interpolated position for position joints
        self._current: Optional[np.ndarray] = None
        # Latest target (after EMA smoothing)
        self._target: Optional[np.ndarray] = None
        # EMA state for target smoothing (position joints only)
        self._smoothed_target: Optional[np.ndarray] = None

    def set_target(self, target: np.ndarray):
        """Set a new action target (called at 100 Hz from chunk stepping).

        Position joints (6:22) are EMA-smoothed to dampen high-frequency
        model noise (oscillation during hold-still phases). Since set_target
        is called at 100 Hz but the raw target changes at ~15 FPS, the EMA
        converges within one action step (~7 ticks) and effectively filters
        at the action step rate. Base velocity passes through unsmoothed.

        Parameters
        ----------
        target : np.ndarray
            Full 22D action vector. Base velocity indices are stored for
            passthrough; position indices are tracked for rate limiting.
        """
        target = np.asarray(target, dtype=np.float64).copy()

        # EMA smooth position joints to dampen oscillation
        if self._ema_alpha < 1.0 and self._smoothed_target is not None:
            target[6:] = (
                self._ema_alpha * target[6:]
                + (1.0 - self._ema_alpha) * self._smoothed_target[6:]
            )

        self._smoothed_target = target.copy()
        self._target = target

    def step(self) -> Optional[np.ndarray]:
        """Advance one control tick. Call at 100 Hz.

        Returns
        -------
        Optional[np.ndarray]
            Rate-limited 22D action, or None if no target set.
        """
        if self._target is None:
            return None

        if self._current is None:
            # First call: snap to target (no history to interpolate from)
            self._current = self._target.copy()
            return self._current.astype(np.float32)

        result = self._current.copy()

        # Base velocity (0:6): pass through target directly
        result[0:6] = self._target[0:6]

        # Position joints (6:22): clamp per-tick delta by max speed
        delta = self._target[6:] - self._current[6:]
        max_d = self._max_delta[6:]
        clamped = np.clip(delta, -max_d, max_d)
        result[6:] = self._current[6:] + clamped

        self._current = result
        return result.astype(np.float32)

    def reset(self):
        """Clear interpolation state (e.g., on deactivation)."""
        self._current = None
        self._target = None
        self._smoothed_target = None

    def get_current(self) -> Optional[np.ndarray]:
        """Return the current interpolated position, or None."""
        if self._current is None:
            return None
        return self._current.astype(np.float32)

    @property
    def has_target(self) -> bool:
        return self._target is not None

    def get_stats(self) -> dict:
        """Return diagnostic info."""
        if self._current is None or self._target is None:
            return {'active': False, 'max_error': 0.0, 'joints_limited': 0}

        delta = np.abs(self._target[6:] - self._current[6:])
        max_d = self._max_delta[6:]
        # A joint is "being limited" if its remaining delta exceeds one tick
        limited = np.sum(delta > max_d * 1.01)

        return {
            'active': True,
            'max_error': float(np.max(delta)),
            'joints_limited': int(limited),
        }
