"""Committed-action prefix for Real-Time Chunking (RTC).

With RTC the client tells the server which actions it has already committed
to execute for the frames after the observation. The server seeds the first
``overlap`` steps of the new chunk with them (GR00T N1.7's native inpainting
in ``Gr00tN1d7ActionHead.get_action_with_features``): the first ``frozen``
steps are held exactly, because the robot executes them before the new chunk
can arrive, and the remaining overlap steps are ramped toward the model's
fresh prediction. Consecutive chunks therefore agree where they meet.

Time base: index 0 of the new chunk is the action step in which the
observation was captured. Everything here is in physical units; the server
normalizes (and converts RELATIVE groups) exactly as the processor does in
training.
"""

from typing import Callable, Optional

import numpy as np


def frozen_steps(latency_skip: int, chunk_blend_steps: int = 0) -> int:
    """How many new-chunk steps the robot executes from the OLD chunk.

    This is the effective skip the client applies when it promotes a chunk:
    ``latency_skip`` steps of inference round trip, plus one when a blend
    step lerps from the old chunk to the new one.
    """
    return int(latency_skip) + (1 if chunk_blend_steps > 0 else 0)


def classic_prefix(
    chunk: Optional[np.ndarray],
    effective_skip: int,
    steps_since_promotion: int,
    overlap: int,
    frozen: int,
) -> Optional[np.ndarray]:
    """Prefix for the trigger/promote execution model.

    The action executed ``s`` steps after a promotion is
    ``chunk[effective_skip + s]``. The observation is captured
    ``steps_since_promotion`` steps in, so the committed future starts there.

    Returns up to ``overlap`` rows (N, 22) or None when fewer than ``frozen``
    rows remain in the chunk (the server then runs without RTC this round).
    """
    if chunk is None or overlap <= 0:
        return None
    start = int(effective_skip) + int(steps_since_promotion)
    if start < 0 or start >= len(chunk):
        return None
    rows = np.asarray(chunk[start:start + overlap], dtype=np.float32)
    if len(rows) < max(int(frozen), 1):
        return None
    return rows.copy()


def buffer_prefix(
    planned_action: Callable[[int], Optional[np.ndarray]],
    obs_frame: int,
    overlap: int,
    frozen: int,
) -> Optional[np.ndarray]:
    """Prefix for continuous mode: the ChunkBuffer's plan for the next frames.

    ``planned_action(frame)`` is ``ChunkBuffer.get_action_raw`` (the ensembled
    action before EMA, i.e. what the command callback will send).
    """
    if overlap <= 0:
        return None
    rows = []
    for i in range(int(overlap)):
        action = planned_action(int(obs_frame) + i)
        if action is None:
            break
        rows.append(np.asarray(action, dtype=np.float32))
    if len(rows) < max(int(frozen), 1):
        return None
    return np.stack(rows)
