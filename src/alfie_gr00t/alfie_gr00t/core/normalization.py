"""State and action normalization using training statistics."""

import json
from pathlib import Path
from typing import Optional, Union

import numpy as np


class Normalizer:
    """Handles normalization and denormalization using training statistics.

    Uses stats from the GR00T training dataset to normalize observations
    and denormalize actions for consistent policy inference.

    State/Action vector layout (22 dimensions):
        [0-5]:   base velocity (linear x,y,z, angular x,y,z)
        [6]:     back joint position
        [7-11]:  left arm joint positions
        [12]:    left gripper
        [13-17]: right arm joint positions
        [18]:    right gripper
        [19-21]: head joint positions
    """

    STATE_DIM = 22
    ACTION_DIM = 22

    def __init__(self, stats_path: Optional[Union[str, Path]] = None):
        """Initialize normalizer with training statistics.

        Args:
            stats_path: Path to stats.jsonl file from training dataset.
                       If None, uses identity normalization (no-op).
        """
        self.stats_path = Path(stats_path) if stats_path else None

        # Default to identity normalization
        self.state_mean = np.zeros(self.STATE_DIM, dtype=np.float32)
        self.state_std = np.ones(self.STATE_DIM, dtype=np.float32)
        self.action_mean = np.zeros(self.ACTION_DIM, dtype=np.float32)
        self.action_std = np.ones(self.ACTION_DIM, dtype=np.float32)

        # Min/max for clamping
        self.state_min = np.full(self.STATE_DIM, -np.inf, dtype=np.float32)
        self.state_max = np.full(self.STATE_DIM, np.inf, dtype=np.float32)
        self.action_min = np.full(self.ACTION_DIM, -np.inf, dtype=np.float32)
        self.action_max = np.full(self.ACTION_DIM, np.inf, dtype=np.float32)

        self._stats_loaded = False

        if stats_path:
            self.load_stats(stats_path)

    def load_stats(self, stats_path: Union[str, Path]) -> bool:
        """Load normalization statistics from file.

        Args:
            stats_path: Path to stats.jsonl file.

        Returns:
            True if stats loaded successfully, False otherwise.
        """
        stats_path = Path(stats_path)

        if not stats_path.exists():
            return False

        try:
            with open(stats_path, 'r') as f:
                stats = json.load(f)

            # Extract state statistics
            if 'state' in stats:
                state_stats = stats['state']
                if 'mean' in state_stats:
                    self.state_mean = np.array(state_stats['mean'], dtype=np.float32)
                if 'std' in state_stats:
                    self.state_std = np.array(state_stats['std'], dtype=np.float32)
                if 'min' in state_stats:
                    self.state_min = np.array(state_stats['min'], dtype=np.float32)
                if 'max' in state_stats:
                    self.state_max = np.array(state_stats['max'], dtype=np.float32)

            # Extract action statistics
            if 'action' in stats:
                action_stats = stats['action']
                if 'mean' in action_stats:
                    self.action_mean = np.array(action_stats['mean'], dtype=np.float32)
                if 'std' in action_stats:
                    self.action_std = np.array(action_stats['std'], dtype=np.float32)
                if 'min' in action_stats:
                    self.action_min = np.array(action_stats['min'], dtype=np.float32)
                if 'max' in action_stats:
                    self.action_max = np.array(action_stats['max'], dtype=np.float32)

            # Replace zero std with 1.0 to avoid division by zero
            self.state_std = np.where(self.state_std == 0, 1.0, self.state_std)
            self.action_std = np.where(self.action_std == 0, 1.0, self.action_std)

            self._stats_loaded = True
            return True

        except (json.JSONDecodeError, KeyError, ValueError):
            return False

    @property
    def is_loaded(self) -> bool:
        """Check if statistics have been loaded."""
        return self._stats_loaded

    def normalize_state(self, state: np.ndarray) -> np.ndarray:
        """Normalize state vector using z-score normalization.

        Args:
            state: Raw state vector of shape (22,) or (N, 22).

        Returns:
            Normalized state vector with zero mean and unit variance.
        """
        state = np.asarray(state, dtype=np.float32)
        return (state - self.state_mean) / self.state_std

    def denormalize_state(self, state: np.ndarray) -> np.ndarray:
        """Denormalize state vector back to original scale.

        Args:
            state: Normalized state vector of shape (22,) or (N, 22).

        Returns:
            Denormalized state vector.
        """
        state = np.asarray(state, dtype=np.float32)
        return state * self.state_std + self.state_mean

    def normalize_action(self, action: np.ndarray) -> np.ndarray:
        """Normalize action vector using z-score normalization.

        Args:
            action: Raw action vector of shape (22,) or (N, 22).

        Returns:
            Normalized action vector.
        """
        action = np.asarray(action, dtype=np.float32)
        return (action - self.action_mean) / self.action_std

    def denormalize_action(self, action: np.ndarray) -> np.ndarray:
        """Denormalize action vector back to original scale.

        Args:
            action: Normalized action vector of shape (22,) or (N, 22).

        Returns:
            Denormalized action vector in robot command units.
        """
        action = np.asarray(action, dtype=np.float32)
        return action * self.action_std + self.action_mean

    def clip_state(self, state: np.ndarray) -> np.ndarray:
        """Clip state to observed min/max range.

        Args:
            state: State vector to clip.

        Returns:
            Clipped state vector.
        """
        return np.clip(state, self.state_min, self.state_max)

    def clip_action(self, action: np.ndarray) -> np.ndarray:
        """Clip action to observed min/max range.

        Args:
            action: Action vector to clip.

        Returns:
            Clipped action vector.
        """
        return np.clip(action, self.action_min, self.action_max)
