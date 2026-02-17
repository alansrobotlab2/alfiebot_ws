"""Rolling action chunk buffer with temporal ensembling.

Maintains a deque of recent 16-action chunks from the inference server.
For any given timestep, finds all chunks with predictions covering that
timestep, combines them using a configurable weighting strategy, and
optionally applies per-body-part EMA smoothing.

Used by both the offline smoothing simulator and the live groot_client.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class TimestampedChunk:
    """A single 16-action chunk with timing metadata."""

    actions: np.ndarray  # (N, 22) raw actions from server, typically N=16
    obs_frame: int  # frame index when observation was captured
    arrival_frame: int  # frame index when chunk result arrived
    chunk_id: int  # sequential chunk identifier
    obs_timestamp: float = 0.0  # monotonic time of observation capture (live)
    arrival_timestamp: float = 0.0  # monotonic time of chunk arrival (live)


class ChunkBuffer:
    """Rolling buffer of recent action chunks with temporal ensembling.

    For any target frame, queries which buffered chunks have predictions
    covering that frame, then combines them using the configured weighting
    strategy. Optionally applies per-body-part EMA on top.

    Thread-safe: all public methods acquire _lock.

    Parameters
    ----------
    max_chunks : int
        Maximum number of chunks to retain in the buffer.
    strategy : str
        Weighting strategy: 'latest', 'uniform', 'recency', 'exp_decay',
        'triangle'.
    decay_m : float
        Exponential decay parameter for 'exp_decay' strategy (ACT paper).
        w = exp(-decay_m * k) where k is the action index offset within
        the chunk (0 = first valid action, higher = further into chunk).
    latency_skip : int
        Number of leading actions to skip in each chunk (latency
        compensation). Valid prediction window is [latency_skip, N).
    chunk_size : int
        Expected number of actions per chunk (default 16).
    ema_alpha_base : float
        EMA alpha for base velocity (indices 0:6). 1.0 = no smoothing.
    ema_alpha_joints : float
        EMA alpha for joints (indices 6:22). 1.0 = no smoothing.
    """

    def __init__(
        self,
        max_chunks: int = 8,
        strategy: str = 'latest',
        decay_m: float = 0.01,
        latency_skip: int = 0,
        chunk_size: int = 16,
        ema_alpha_base: float = 1.0,
        ema_alpha_joints: float = 1.0,
    ):
        self.max_chunks = max_chunks
        self.strategy = strategy
        self.decay_m = decay_m
        self.latency_skip = latency_skip
        self.chunk_size = chunk_size
        self.ema_alpha_base = ema_alpha_base
        self.ema_alpha_joints = ema_alpha_joints

        self._chunks: deque[TimestampedChunk] = deque(maxlen=max_chunks)
        self._last_output: Optional[np.ndarray] = None
        self._lock = threading.Lock()

    # ── Public API ────────────────────────────────────────────────────

    def add_chunk(self, chunk: TimestampedChunk) -> None:
        """Add a new chunk to the buffer."""
        with self._lock:
            self._chunks.append(chunk)

    def get_action(self, target_frame: int) -> Optional[np.ndarray]:
        """Get the smoothed action for a target frame.

        Combines overlapping chunk predictions, then applies EMA.
        Returns None if no chunk covers the target frame.
        """
        with self._lock:
            raw = self._get_combined(target_frame)
            if raw is None:
                return None
            return self._apply_ema(raw)

    def get_action_raw(self, target_frame: int) -> Optional[np.ndarray]:
        """Get the combined action BEFORE EMA (for CSV logging)."""
        with self._lock:
            return self._get_combined(target_frame)

    @property
    def current_chunk_id(self) -> int:
        """ID of the most recently added chunk, or -1."""
        with self._lock:
            if self._chunks:
                return self._chunks[-1].chunk_id
            return -1

    @property
    def num_chunks(self) -> int:
        with self._lock:
            return len(self._chunks)

    def reset(self) -> None:
        """Clear all state."""
        with self._lock:
            self._chunks.clear()
            self._last_output = None

    # ── Internals (must hold _lock) ───────────────────────────────────

    def _get_combined(self, target_frame: int) -> Optional[np.ndarray]:
        """Combine overlapping chunk predictions for target_frame."""
        if not self._chunks:
            return None

        # For 'latest' strategy, fast path: only use the most recent chunk
        if self.strategy == 'latest':
            return self._get_latest(target_frame)

        # Collect predictions and weights from all covering chunks
        predictions = []
        weights = []

        for chunk in self._chunks:
            action_idx = self._frame_to_action_idx(chunk, target_frame)
            if action_idx is None:
                continue

            predictions.append(chunk.actions[action_idx])
            w = self._compute_weight(chunk, action_idx, target_frame)
            weights.append(w)

        if not predictions:
            return None

        if len(predictions) == 1:
            return predictions[0].copy()

        predictions_arr = np.array(predictions)  # (N, 22)
        weights_arr = np.array(weights)
        weights_arr /= weights_arr.sum()

        return np.average(predictions_arr, axis=0, weights=weights_arr)

    def _get_latest(self, target_frame: int) -> Optional[np.ndarray]:
        """Fast path: only use the most recent chunk covering target_frame."""
        # Search from newest to oldest
        for chunk in reversed(self._chunks):
            action_idx = self._frame_to_action_idx(chunk, target_frame)
            if action_idx is not None:
                return chunk.actions[action_idx].copy()
        return None

    def _frame_to_action_idx(
        self, chunk: TimestampedChunk, target_frame: int
    ) -> Optional[int]:
        """Map a target frame to an action index within a chunk.

        Returns None if the chunk doesn't cover target_frame.

        The chunk's valid prediction window starts at:
            arrival_frame (when the chunk becomes available)
        The action index for target_frame is:
            latency_skip + (target_frame - arrival_frame)
        Valid range: [latency_skip, len(chunk.actions))
        """
        offset = target_frame - chunk.arrival_frame
        action_idx = self.latency_skip + offset

        if action_idx < self.latency_skip:
            return None
        if action_idx >= len(chunk.actions):
            return None

        return action_idx

    def _compute_weight(
        self,
        chunk: TimestampedChunk,
        action_idx: int,
        target_frame: int,
    ) -> float:
        """Compute weight for a chunk's prediction at target_frame."""
        # k = how far into the chunk's valid window we are
        k = action_idx - self.latency_skip

        if self.strategy == 'uniform':
            return 1.0

        elif self.strategy == 'recency':
            # Newer chunks (smaller k) get higher weight
            max_k = self.chunk_size - self.latency_skip
            return max(0.01, 1.0 - k / max_k)

        elif self.strategy == 'exp_decay':
            # ACT-style: w = exp(-m * k)
            return float(np.exp(-self.decay_m * k))

        elif self.strategy == 'triangle':
            # Peak weight at the start of valid window, linear decay
            max_k = self.chunk_size - self.latency_skip
            center = 0  # Peak at first valid action (freshest prediction)
            dist = abs(k - center)
            return max(0.01, 1.0 - dist / max_k)

        # Fallback
        return 1.0

    def _apply_ema(self, action: np.ndarray) -> np.ndarray:
        """Apply per-body-part EMA smoothing."""
        if self._last_output is None:
            self._last_output = action.copy()
            return action.copy()

        smoothed = action.copy()

        # Base velocity (indices 0:6)
        if self.ema_alpha_base < 1.0:
            smoothed[0:6] = (
                self.ema_alpha_base * action[0:6]
                + (1.0 - self.ema_alpha_base) * self._last_output[0:6]
            )

        # Joints (indices 6:22)
        if self.ema_alpha_joints < 1.0:
            smoothed[6:] = (
                self.ema_alpha_joints * action[6:]
                + (1.0 - self.ema_alpha_joints) * self._last_output[6:]
            )

        self._last_output = smoothed.copy()
        return smoothed
