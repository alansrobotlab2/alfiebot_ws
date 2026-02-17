"""Causal post-processing filters for 15 FPS ensembled actions.

Applies Savitzky-Golay or Butterworth low-pass filtering to the ensembled
action stream before 100 Hz interpolation. Maintains internal ring buffer
state for causal (live) operation.

Used by both the offline smoothing simulator and the live groot_client.
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
from scipy.signal import savgol_coeffs, sosfilt, butter


class ActionSmoother:
    """Causal post-processing filter for 15 FPS action waypoints.

    Sits between ChunkBuffer output and the interpolation stage.
    Supports per-body-part configuration via body_part_ranges.

    Parameters
    ----------
    method : str
        'none', 'savgol', or 'butterworth'.
    savgol_window : int
        Window length for Savitzky-Golay filter (must be odd, >= 3).
    savgol_polyorder : int
        Polynomial order for Savitzky-Golay filter.
    butter_order : int
        Order for Butterworth low-pass filter.
    butter_cutoff_hz : float
        Cutoff frequency for Butterworth filter (Hz). Nyquist at 15 FPS = 7.5 Hz.
    sample_rate : float
        Input sample rate in Hz (default 15.0 for 15 FPS actions).
    action_dim : int
        Dimensionality of the action vector (default 22).
    """

    def __init__(
        self,
        method: str = 'none',
        savgol_window: int = 5,
        savgol_polyorder: int = 2,
        butter_order: int = 2,
        butter_cutoff_hz: float = 5.0,
        sample_rate: float = 15.0,
        action_dim: int = 22,
    ):
        self.method = method
        self.action_dim = action_dim

        if method == 'savgol':
            self._init_savgol(savgol_window, savgol_polyorder)
        elif method == 'butterworth':
            self._init_butterworth(butter_order, butter_cutoff_hz, sample_rate)
        elif method != 'none':
            raise ValueError(f"Unknown smoothing method: {method!r}. "
                             f"Must be 'none', 'savgol', or 'butterworth'.")

    def _init_savgol(self, window: int, polyorder: int):
        """Initialize Savitzky-Golay filter state."""
        if window < 3 or window % 2 == 0:
            raise ValueError(f"savgol_window must be odd and >= 3, got {window}")
        if polyorder >= window:
            raise ValueError(f"savgol_polyorder ({polyorder}) must be < window ({window})")

        self._sg_window = window
        self._sg_polyorder = polyorder

        # Pre-compute causal SG coefficients.
        # For a causal filter, we use a one-sided window: the current sample
        # is at the right edge of the window (pos = window - 1).
        # use='dot' gives coefficients in data order: coeffs[0] * oldest + ... + coeffs[-1] * newest
        self._sg_coeffs = savgol_coeffs(
            window, polyorder, pos=window - 1, use='dot'
        )  # shape: (window,)

        # Ring buffer: stores the last `window` actions
        self._sg_buffer: deque[np.ndarray] = deque(maxlen=window)

    def _init_butterworth(self, order: int, cutoff_hz: float, fs: float):
        """Initialize Butterworth low-pass filter state."""
        nyquist = fs / 2.0
        if cutoff_hz >= nyquist:
            raise ValueError(
                f"butter_cutoff_hz ({cutoff_hz}) must be < Nyquist ({nyquist})"
            )

        # Second-order sections for numerical stability
        self._bw_sos = butter(order, cutoff_hz, btype='low', fs=fs, output='sos')

        # Filter state: one state vector per SOS section per action dimension.
        # sosfilt with zi expects shape (n_sections, 2) per channel.
        n_sections = self._bw_sos.shape[0]
        self._bw_zi = np.zeros((n_sections, 2, self.action_dim))

        self._bw_initialized = False

    def smooth(self, action: np.ndarray) -> np.ndarray:
        """Process one 15 FPS action through the filter.

        Parameters
        ----------
        action : np.ndarray
            22D action vector (the ensembled output for one frame).

        Returns
        -------
        np.ndarray
            Smoothed 22D action vector.
        """
        action = np.asarray(action, dtype=np.float64)

        if self.method == 'none':
            return action.copy()
        elif self.method == 'savgol':
            return self._smooth_savgol(action)
        elif self.method == 'butterworth':
            return self._smooth_butterworth(action)
        else:
            return action.copy()

    def smooth_batch(self, actions: np.ndarray) -> np.ndarray:
        """Process a batch of actions (for offline use).

        Resets state before processing. Each row is filtered causally
        (only uses current and past samples).

        Parameters
        ----------
        actions : np.ndarray
            (N, 22) array of actions.

        Returns
        -------
        np.ndarray
            (N, 22) smoothed actions. NaN rows pass through as NaN.
        """
        self.reset()
        result = np.empty_like(actions)
        for i in range(len(actions)):
            if np.isnan(actions[i, 0]):
                result[i] = actions[i]
            else:
                result[i] = self.smooth(actions[i])
        return result

    def reset(self):
        """Reset all internal filter state."""
        if self.method == 'savgol':
            self._sg_buffer.clear()
        elif self.method == 'butterworth':
            n_sections = self._bw_sos.shape[0]
            self._bw_zi = np.zeros((n_sections, 2, self.action_dim))
            self._bw_initialized = False

    def _smooth_savgol(self, action: np.ndarray) -> np.ndarray:
        """Causal Savitzky-Golay: dot product of coefficients with ring buffer."""
        self._sg_buffer.append(action.copy())

        if len(self._sg_buffer) < self._sg_window:
            # Not enough history — return unfiltered
            return action.copy()

        # Stack buffer: [oldest, ..., newest], shape (window, 22)
        buf = np.array(self._sg_buffer)
        # Dot product: coeffs (window,) @ buf (window, 22) -> (22,)
        return self._sg_coeffs @ buf

    def _smooth_butterworth(self, action: np.ndarray) -> np.ndarray:
        """Causal Butterworth: one-sample-at-a-time IIR filtering via sosfilt."""
        if not self._bw_initialized:
            # Initialize filter state to the first sample value to avoid
            # startup transient. This sets zi such that the filter output
            # for a constant input equals that constant.
            self._bw_initialized = True
            from scipy.signal import sosfilt_zi
            zi_template = sosfilt_zi(self._bw_sos)  # (n_sections, 2)
            # Broadcast to all action dims
            for dim in range(self.action_dim):
                self._bw_zi[:, :, dim] = zi_template * action[dim]

        # Process one sample: input shape (1, action_dim)
        x = action.reshape(1, -1)
        y, self._bw_zi = sosfilt(self._bw_sos, x, axis=0, zi=self._bw_zi)
        return y[0]
