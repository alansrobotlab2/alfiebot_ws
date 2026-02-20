"""Pluggable 100 Hz interpolation between 15 FPS action waypoints.

Replaces the inline linear lerp in groot_client.py with configurable
interpolation strategies. Maintains a rolling window of recent waypoints
for spline fitting.

Used by both the offline smoothing simulator and the live groot_client.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
from scipy.interpolate import CubicSpline


class ActionInterpolator:
    """Interpolates between 15 FPS action waypoints for 100 Hz output.

    Maintains a rolling window of recent waypoints and evaluates the
    interpolating function at arbitrary sub-frame times.

    Parameters
    ----------
    method : str
        'linear' or 'cubic_spline'.
    spline_window : int
        Number of waypoints to use for spline fitting (minimum 4 for
        cubic spline). Larger windows produce smoother curves but may
        lag at chunk boundaries.
    action_dim : int
        Dimensionality of the action vector (default 22).
    """

    def __init__(
        self,
        method: str = 'linear',
        spline_window: int = 6,
        action_dim: int = 22,
    ):
        self.method = method
        self.action_dim = action_dim

        if method == 'cubic_spline':
            if spline_window < 4:
                raise ValueError(
                    f"spline_window must be >= 4 for cubic spline, got {spline_window}"
                )
            self._window_size = spline_window
        elif method == 'linear':
            self._window_size = 2  # only need current + next
        else:
            raise ValueError(f"Unknown interpolation method: {method!r}. "
                             f"Must be 'linear' or 'cubic_spline'.")

        # Rolling window of (frame_index, action) pairs
        self._waypoints: deque[tuple[int, np.ndarray]] = deque(
            maxlen=max(self._window_size + 2, 8)
        )
        self._spline: Optional[CubicSpline] = None
        self._spline_valid_range: tuple[int, int] = (0, 0)

    def update_waypoint(self, frame_idx: int, action: np.ndarray):
        """Add or update a 15 FPS waypoint.

        Parameters
        ----------
        frame_idx : int
            Frame index (monotonically increasing).
        action : np.ndarray
            22D action vector for this frame.
        """
        action = np.asarray(action, dtype=np.float64)

        # Check if this frame already exists anywhere in the deque.
        # At 100Hz, the same frame is updated ~7 times before advancing.
        # Only checking the last entry would miss frame N after N+1 is added.
        for i, (f, _) in enumerate(self._waypoints):
            if f == frame_idx:
                self._waypoints[i] = (frame_idx, action.copy())
                if self.method == 'cubic_spline':
                    self._spline = None
                return

        # New frame — only append if strictly after the last one
        if self._waypoints and frame_idx <= self._waypoints[-1][0]:
            return

        self._waypoints.append((frame_idx, action.copy()))

        # Invalidate cached spline when waypoints change
        if self.method == 'cubic_spline':
            self._spline = None

    def evaluate(self, t: float) -> Optional[np.ndarray]:
        """Evaluate interpolation at fractional frame time.

        Parameters
        ----------
        t : float
            Frame time (e.g., 42.35 means 35% of the way between frame 42 and 43).

        Returns
        -------
        Optional[np.ndarray]
            Interpolated 22D action, or None if insufficient waypoints.
        """
        if not self._waypoints:
            return None

        if self.method == 'linear':
            return self._eval_linear(t)
        elif self.method == 'cubic_spline':
            return self._eval_spline(t)
        return None

    def evaluate_batch(
        self,
        waypoints: np.ndarray,
        frame_indices: np.ndarray,
        eval_times: np.ndarray,
    ) -> np.ndarray:
        """Batch evaluation for offline use — no state management needed.

        Fits a single spline/interpolant through all waypoints and evaluates
        at all requested times.

        Parameters
        ----------
        waypoints : np.ndarray
            (N, 22) action waypoints at 15 FPS.
        frame_indices : np.ndarray
            (N,) frame indices corresponding to each waypoint.
        eval_times : np.ndarray
            (M,) fractional frame times to evaluate at.

        Returns
        -------
        np.ndarray
            (M, 22) interpolated actions.
        """
        if self.method == 'linear':
            return self._batch_linear(waypoints, frame_indices, eval_times)
        elif self.method == 'cubic_spline':
            return self._batch_spline(waypoints, frame_indices, eval_times)
        return np.full((len(eval_times), self.action_dim), np.nan)

    def reset(self):
        """Clear all waypoint state."""
        self._waypoints.clear()
        self._spline = None
        self._spline_valid_range = (0, 0)

    # ── Linear interpolation ──────────────────────────────────────────

    def _eval_linear(self, t: float) -> Optional[np.ndarray]:
        """Linear lerp between the two nearest waypoints."""
        frame = int(t)
        alpha = t - frame

        # Find bracketing waypoints
        prev_action = None
        next_action = None
        for wp_frame, wp_action in self._waypoints:
            if wp_frame == frame:
                prev_action = wp_action
            elif wp_frame == frame + 1:
                next_action = wp_action

        if prev_action is None:
            # Fall back to nearest
            for wp_frame, wp_action in reversed(self._waypoints):
                if wp_frame <= frame:
                    return wp_action.copy()
            return self._waypoints[-1][1].copy() if self._waypoints else None

        if next_action is None:
            return prev_action.copy()

        return (1.0 - alpha) * prev_action + alpha * next_action

    def _batch_linear(
        self, waypoints: np.ndarray, frame_indices: np.ndarray, eval_times: np.ndarray
    ) -> np.ndarray:
        """Batch linear interpolation."""
        result = np.empty((len(eval_times), self.action_dim))
        for i, t in enumerate(eval_times):
            frame = int(t)
            alpha = t - frame

            # Find indices in frame_indices
            idx = np.searchsorted(frame_indices, frame, side='right') - 1
            idx = max(0, min(idx, len(frame_indices) - 1))

            if idx + 1 < len(frame_indices):
                result[i] = (1.0 - alpha) * waypoints[idx] + alpha * waypoints[idx + 1]
            else:
                result[i] = waypoints[idx]
        return result

    # ── Cubic spline interpolation ────────────────────────────────────

    def _build_spline(self):
        """Build cubic spline from current waypoints."""
        if len(self._waypoints) < 4:
            self._spline = None
            return

        frames = np.array([wp[0] for wp in self._waypoints], dtype=np.float64)
        actions = np.array([wp[1] for wp in self._waypoints])  # (N, 22)

        # Not-a-knot boundary conditions: avoids artificial flattening at edges
        self._spline = CubicSpline(frames, actions, bc_type='not-a-knot')
        self._spline_valid_range = (int(frames[0]), int(frames[-1]))

    def _eval_spline(self, t: float) -> Optional[np.ndarray]:
        """Evaluate cubic spline at fractional time t."""
        if self._spline is None:
            self._build_spline()

        if self._spline is None:
            # Fall back to linear if not enough waypoints
            return self._eval_linear(t)

        # Clamp to valid range to avoid extrapolation artifacts
        t_clamped = max(self._spline_valid_range[0],
                        min(t, self._spline_valid_range[1]))

        return self._spline(t_clamped)

    def _batch_spline(
        self, waypoints: np.ndarray, frame_indices: np.ndarray, eval_times: np.ndarray
    ) -> np.ndarray:
        """Batch cubic spline interpolation."""
        if len(waypoints) < 4:
            # Fall back to linear
            return self._batch_linear(waypoints, frame_indices, eval_times)

        frames = frame_indices.astype(np.float64)
        spline = CubicSpline(frames, waypoints, bc_type='not-a-knot')

        # Clamp evaluation times to valid range
        t_clamped = np.clip(eval_times, frames[0], frames[-1])
        return spline(t_clamped)
