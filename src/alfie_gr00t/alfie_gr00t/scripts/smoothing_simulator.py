#!/usr/bin/env python3
"""Offline Action Smoothing Strategy Simulator.

Takes cached horizons (.npz from overlapped_execution_test.py) + GT episode
data, simulates continuous inference at configurable rates, applies different
temporal ensembling strategies via ChunkBuffer, and outputs comparison CSVs
and plots.

No ROS2 or inference server dependency — runs purely offline.

Usage:
    python smoothing_simulator.py \
        --horizons /tmp/overlapped_test/horizons_ep320.npz \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 320 \
        --inference-rates 3.3,5.0 \
        --strategies latest,uniform,exp_m1,exp_m5,triangle \
        --output-dir /tmp/smoothing_results/

    # Batch mode
    python smoothing_simulator.py \
        --horizons-dir /tmp/overlapped_test/ \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-indices 317,318,319,320,321 \
        --inference-rates 3.3,5.0 \
        --output-dir /tmp/smoothing_results/
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Add parent to path for imports
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.chunk_buffer import ChunkBuffer, TimestampedChunk
from alfie_gr00t.core.action_smoother import ActionSmoother
from alfie_gr00t.core.action_interpolator import ActionInterpolator
from alfie_gr00t.scripts.overlapped_execution_test import (
    BODY_PART_GROUPS,
    JOINT_NAMES,
    load_episode,
    load_horizons,
)

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────

TRAINING_FPS = 15
ACTION_STEP_PERIOD = 1.0 / TRAINING_FPS  # ~0.0667s

# Key joints for plots
PLOT_JOINTS = [
    (0,  'cmd_vel_lx',           'Base Forward Vel'),
    (5,  'cmd_vel_az',           'Base Angular Vel'),
    (14, 'right_shoulder_pitch', 'R Shoulder Pitch'),
    (15, 'right_elbow_pitch',    'R Elbow Pitch'),
    (18, 'right_gripper',        'R Gripper'),
    (19, 'head_yaw',             'Head Yaw'),
]

# ── Strategy presets ──────────────────────────────────────────────────

STRATEGY_PRESETS = {
    'latest': {
        'strategy': 'latest',
        'decay_m': 0.0,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'uniform': {
        'strategy': 'uniform',
        'decay_m': 0.0,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'recency': {
        'strategy': 'recency',
        'decay_m': 0.0,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'exp_m01': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'exp_m1': {
        'strategy': 'exp_decay',
        'decay_m': 0.1,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'exp_m5': {
        'strategy': 'exp_decay',
        'decay_m': 0.5,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'triangle': {
        'strategy': 'triangle',
        'decay_m': 0.0,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
    },
    'exp_m1_ema90': {
        'strategy': 'exp_decay',
        'decay_m': 0.1,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 0.9,
    },
    # ── Advanced: temporal ensembling + post-processing filter ──
    'exp_m01_savgol5_p2': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'savgol', 'savgol_window': 5, 'savgol_polyorder': 2},
    },
    'exp_m01_savgol7_p3': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'savgol', 'savgol_window': 7, 'savgol_polyorder': 3},
    },
    'exp_m01_butter_c3': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'butterworth', 'butter_cutoff_hz': 3.0},
    },
    'exp_m01_butter_c5': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'butterworth', 'butter_cutoff_hz': 5.0},
    },
    'exp_m01_butter_c7': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'butterworth', 'butter_cutoff_hz': 7.0},
    },
    # ── Advanced: cubic spline interpolation (requires --output-rate 100) ──
    'exp_m01_spline': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'interpolator': {'method': 'cubic_spline', 'spline_window': 6},
    },
    'exp_m01_spline_savgol5': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'savgol', 'savgol_window': 5, 'savgol_polyorder': 2},
        'interpolator': {'method': 'cubic_spline', 'spline_window': 6},
    },
    'exp_m01_spline_butter5': {
        'strategy': 'exp_decay',
        'decay_m': 0.01,
        'ema_alpha_base': 1.0,
        'ema_alpha_joints': 1.0,
        'smoother': {'method': 'butterworth', 'butter_cutoff_hz': 5.0},
        'interpolator': {'method': 'cubic_spline', 'spline_window': 6},
    },
}


# ── Continuous inference simulation ───────────────────────────────────

def simulate_continuous_chunks(
    horizons: list[np.ndarray | None],
    num_frames: int,
    inference_rate_hz: float,
    latency_frames: int,
) -> list[TimestampedChunk]:
    """Simulate continuous inference at a given rate.

    Uses pre-collected horizons (one per GT frame). Selects observation
    frames at intervals matching the target inference rate, adds latency
    offset for arrival frame.

    Parameters
    ----------
    horizons : list
        One (16,22) array per GT frame (or None if inference failed).
    num_frames : int
        Total frames in the episode.
    inference_rate_hz : float
        Target inference rate (e.g. 3.3, 5.0).
    latency_frames : int
        Round-trip inference latency in frames.

    Returns
    -------
    list[TimestampedChunk]
        Chunks in arrival order.
    """
    obs_interval = TRAINING_FPS / inference_rate_hz  # frames between obs
    chunks = []
    chunk_id = 0
    obs_time = 0.0  # float frame index

    while obs_time < num_frames:
        obs_idx = int(round(obs_time))
        if obs_idx >= len(horizons):
            break

        horizon = horizons[obs_idx]
        if horizon is not None:
            arrival_frame = obs_idx + latency_frames
            chunks.append(TimestampedChunk(
                actions=horizon,
                obs_frame=obs_idx,
                arrival_frame=arrival_frame,
                chunk_id=chunk_id,
                obs_timestamp=obs_idx * ACTION_STEP_PERIOD,
                arrival_timestamp=arrival_frame * ACTION_STEP_PERIOD,
            ))
            chunk_id += 1

        obs_time += obs_interval

    return chunks


# ── Strategy simulation ───────────────────────────────────────────────

def run_strategy(
    chunks: list[TimestampedChunk],
    num_frames: int,
    strategy_config: dict,
    latency_skip: int,
    output_rate_hz: float = 0.0,
) -> np.ndarray:
    """Run a smoothing strategy over all frames.

    When output_rate_hz == 0 (default), returns (num_frames, 22) at 15 FPS.
    When output_rate_hz > 0, returns (num_output_samples, 22) at that rate,
    using the configured interpolator for sub-frame sampling.

    NaN for frames/samples with no prediction available.
    """
    # Separate smoother/interpolator config from ChunkBuffer config
    config = dict(strategy_config)
    smoother_config = config.pop('smoother', None)
    interpolator_config = config.pop('interpolator', None)

    buffer = ChunkBuffer(
        max_chunks=8,
        latency_skip=latency_skip,
        **config,
    )

    # Build optional post-processing smoother
    smoother = None
    if smoother_config:
        smoother = ActionSmoother(**smoother_config)

    # Get 15 FPS ensembled actions first
    raw_actions = np.full((num_frames, 22), np.nan)

    sorted_chunks = sorted(chunks, key=lambda c: c.arrival_frame)
    chunk_idx = 0

    for frame in range(num_frames):
        while (chunk_idx < len(sorted_chunks)
               and sorted_chunks[chunk_idx].arrival_frame <= frame):
            buffer.add_chunk(sorted_chunks[chunk_idx])
            chunk_idx += 1

        action = buffer.get_action(frame)
        if action is not None:
            raw_actions[frame] = action

    # Apply post-processing filter (causal, sample-by-sample)
    if smoother is not None:
        smoothed_actions = smoother.smooth_batch(raw_actions)
    else:
        smoothed_actions = raw_actions

    # If no upsampling requested, return 15 FPS result
    if output_rate_hz <= 0:
        return smoothed_actions

    # Upsample to target output rate using interpolator
    interp_method = 'linear'
    interp_kwargs = {}
    if interpolator_config:
        interp_method = interpolator_config.get('method', 'linear')
        interp_kwargs = {k: v for k, v in interpolator_config.items() if k != 'method'}

    # Collect valid waypoints
    valid_frames = np.where(~np.isnan(smoothed_actions[:, 0]))[0]
    if len(valid_frames) < 2:
        return smoothed_actions

    valid_waypoints = smoothed_actions[valid_frames]

    # Generate evaluation times at output_rate_hz
    total_duration = (num_frames - 1) / TRAINING_FPS  # seconds
    num_samples = int(total_duration * output_rate_hz) + 1
    eval_times_sec = np.linspace(0, total_duration, num_samples)
    eval_times_frames = eval_times_sec * TRAINING_FPS  # fractional frame indices

    interpolator = ActionInterpolator(method=interp_method, **interp_kwargs)
    result = interpolator.evaluate_batch(
        valid_waypoints,
        valid_frames.astype(np.float64),
        eval_times_frames,
    )
    return result


# ── Metrics ───────────────────────────────────────────────────────────

def compute_metrics(
    strategy_actions: np.ndarray,
    gt_actions: np.ndarray,
    chunks: list[TimestampedChunk],
    output_rate_hz: float = 0.0,
) -> dict:
    """Compute all metrics for a strategy's output vs GT.

    When output_rate_hz > 0, strategy_actions is at that rate and gt_actions
    is at 15 FPS. MAE is computed by resampling GT to match.
    """
    valid = ~np.isnan(strategy_actions[:, 0])
    n_valid = valid.sum()

    metrics = {'n_valid_frames': int(n_valid), 'n_total_frames': len(gt_actions)}

    if n_valid == 0:
        return metrics

    # For high-rate output, resample GT to match for MAE comparison
    if output_rate_hz > 0:
        gt_resampled = _resample_gt(gt_actions, len(strategy_actions), output_rate_hz)
    else:
        gt_resampled = gt_actions

    # Align lengths
    n = min(len(strategy_actions), len(gt_resampled))
    sa = strategy_actions[:n]
    gt = gt_resampled[:n]
    valid = ~np.isnan(sa[:, 0]) & ~np.isnan(gt[:, 0])

    if valid.sum() == 0:
        return metrics

    # Per-body-part MAE
    for name, start, end in BODY_PART_GROUPS:
        mask = valid
        if mask.sum() > 0:
            mae = float(np.mean(np.abs(sa[mask, start:end] - gt[mask, start:end])))
        else:
            mae = float('nan')
        metrics[f'mae_{name}'] = mae

    # Overall MAE
    metrics['mae_overall'] = float(np.mean(np.abs(sa[valid] - gt[valid])))

    # Smoothness: action jerk RMS (2nd derivative)
    diffs = np.diff(sa[valid], axis=0)
    if len(diffs) > 1:
        jerk = np.diff(diffs, axis=0)
        metrics['jerk_rms'] = float(np.sqrt(np.mean(jerk ** 2)))
    else:
        metrics['jerk_rms'] = 0.0

    # Boundary discontinuity (at chunk arrival frames)
    # For high-rate output, scale arrival frames to sample indices
    arrival_frames = sorted(set(c.arrival_frame for c in chunks))
    if output_rate_hz > 0:
        scale = output_rate_hz / TRAINING_FPS
        arrival_indices = [int(af * scale) for af in arrival_frames]
    else:
        arrival_indices = arrival_frames

    jumps = []
    for ai in arrival_indices:
        if 0 < ai < len(sa):
            prev = sa[ai - 1]
            curr = sa[ai]
            if not np.isnan(prev).any() and not np.isnan(curr).any():
                jumps.append(np.abs(curr - prev))

    if jumps:
        jumps_arr = np.array(jumps)
        metrics['boundary_jump_mean'] = float(np.mean(jumps_arr))
        for name, start, end in BODY_PART_GROUPS:
            metrics[f'boundary_jump_{name}'] = float(
                np.mean(jumps_arr[:, start:end])
            )
    else:
        metrics['boundary_jump_mean'] = 0.0

    # Pogo score: sign alternation at chunk boundaries (right arm focus)
    for jname, jidx in [('right_shoulder_pitch', 14), ('right_elbow_pitch', 15)]:
        boundary_jumps = []
        for ai in arrival_indices:
            if 0 < ai < len(sa):
                prev = sa[ai - 1, jidx]
                curr = sa[ai, jidx]
                if not np.isnan(prev) and not np.isnan(curr):
                    boundary_jumps.append(curr - prev)

        if len(boundary_jumps) > 1:
            signs = np.sign(boundary_jumps)
            alternations = np.sum(np.abs(np.diff(signs)) == 2)
            pogo = alternations / (len(signs) - 1)
        else:
            pogo = 0.0
        metrics[f'pogo_{jname}'] = pogo

    # Action delta RMS (frame-to-frame smoothness)
    if len(diffs) > 0:
        metrics['action_delta_rms'] = float(np.sqrt(np.mean(diffs ** 2)))
    else:
        metrics['action_delta_rms'] = 0.0

    # Velocity continuity (100 Hz specific): RMS of velocity discontinuities
    # at 15 FPS waypoint boundaries
    if output_rate_hz > 0 and len(diffs) > 2:
        # Second derivative = acceleration; its discontinuities reveal
        # interpolation quality
        accel = np.diff(diffs, axis=0)
        metrics['accel_rms'] = float(np.sqrt(np.mean(accel ** 2)))

    return metrics


def _resample_gt(gt_actions: np.ndarray, num_samples: int, output_rate_hz: float) -> np.ndarray:
    """Resample 15 FPS GT actions to match high-rate output for MAE comparison.

    Uses linear interpolation of GT to the output timestamps.
    """
    num_gt = len(gt_actions)
    gt_times = np.arange(num_gt, dtype=np.float64) / TRAINING_FPS
    out_duration = (num_gt - 1) / TRAINING_FPS
    out_times = np.linspace(0, out_duration, num_samples)

    result = np.empty((num_samples, gt_actions.shape[1]))
    for dim in range(gt_actions.shape[1]):
        result[:, dim] = np.interp(out_times, gt_times, gt_actions[:, dim])
    return result


# ── CSV output (compatible with live debug CSV format) ────────────────

def strategy_to_csv(
    strategy_actions: np.ndarray,
    gt_states: np.ndarray,
    gt_actions: np.ndarray,
    chunks: list[TimestampedChunk],
    output_path: Path,
):
    """Save strategy output as CSV compatible with the live debug format.

    Columns: timestamp, step, chunk_id, action_idx,
             action_<joint>*22, smoothed_<joint>*22, state_<joint>*22
    """
    num_frames = len(strategy_actions)

    # Build chunk_id map: for each frame, which chunk is "current"
    sorted_chunks = sorted(chunks, key=lambda c: c.arrival_frame)
    chunk_id_map = np.full(num_frames, -1, dtype=int)
    for chunk in sorted_chunks:
        start = chunk.arrival_frame
        end = min(num_frames, chunk.arrival_frame + len(chunk.actions))
        chunk_id_map[start:end] = chunk.chunk_id

    rows = []
    for frame in range(num_frames):
        if np.isnan(strategy_actions[frame, 0]):
            continue

        row = {
            'timestamp': frame * ACTION_STEP_PERIOD,
            'step': frame,
            'chunk_id': int(chunk_id_map[frame]),
            'action_idx': frame % 16,
        }
        for j, jname in enumerate(JOINT_NAMES):
            row[f'action_{jname}'] = gt_actions[frame, j]
            row[f'smoothed_{jname}'] = strategy_actions[frame, j]
            row[f'state_{jname}'] = gt_states[frame, j]
        rows.append(row)

    df = pd.DataFrame(rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)
    logger.info(f'Saved CSV: {output_path} ({len(df)} rows)')


# ── Plotting ──────────────────────────────────────────────────────────

COLORS = [
    '#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd',
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]


def plot_strategy_comparison(
    results: dict[str, np.ndarray],
    gt_actions: np.ndarray,
    inference_rate: float,
    output_path: Path,
    output_rate_hz: float = 0.0,
):
    """Overlay multiple strategy trajectories for key joints."""
    n_joints = len(PLOT_JOINTS)
    fig, axes = plt.subplots(n_joints, 1, figsize=(18, 6.0 * n_joints),
                             sharex=True)
    gt_time_s = np.arange(len(gt_actions)) * ACTION_STEP_PERIOD

    for i, (idx, jname, title) in enumerate(PLOT_JOINTS):
        ax = axes[i]

        # GT reference
        ax.plot(gt_time_s, gt_actions[:, idx], color='black', alpha=0.3,
                linewidth=1.5, label='GT', zorder=1)

        # Strategies (may be at different sample rate)
        for ci, (sname, sactions) in enumerate(results.items()):
            n_samples = len(sactions)
            if output_rate_hz > 0 and n_samples != len(gt_actions):
                total_dur = (len(gt_actions) - 1) * ACTION_STEP_PERIOD
                s_time = np.linspace(0, total_dur, n_samples)
            else:
                s_time = np.arange(n_samples) * ACTION_STEP_PERIOD
            valid = ~np.isnan(sactions[:, idx])
            ax.plot(s_time[valid], sactions[valid, idx],
                    color=COLORS[ci % len(COLORS)],
                    alpha=0.8, linewidth=1.0, label=sname, zorder=2)

        ax.set_ylabel(jname, fontsize=8)
        ax.set_title(title, fontsize=9, loc='left')
        ax.grid(True, alpha=0.2)
        if i == 0:
            ax.legend(loc='upper right', fontsize=7, ncol=min(5, len(results) + 1))

    axes[-1].set_xlabel('Time (s)')
    plt.suptitle(f'Strategy Comparison @ {inference_rate:.1f}Hz Inference',
                 fontsize=12, y=0.99)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved plot: {output_path}')


def plot_boundary_jumps(
    results: dict[str, np.ndarray],
    chunks_per_strategy: dict[str, list[TimestampedChunk]],
    output_path: Path,
):
    """Bar chart of mean boundary jump per body part per strategy."""
    strategy_names = list(results.keys())
    body_parts = [name for name, _, _ in BODY_PART_GROUPS]
    n_strategies = len(strategy_names)
    n_parts = len(body_parts)

    fig, ax = plt.subplots(figsize=(14, 12))
    x = np.arange(n_parts)
    width = 0.8 / max(n_strategies, 1)

    for si, sname in enumerate(strategy_names):
        sactions = results[sname]
        chunks = chunks_per_strategy[sname]
        arrival_frames = sorted(set(c.arrival_frame for c in chunks))

        jumps = []
        for af in arrival_frames:
            if 0 < af < len(sactions):
                prev = sactions[af - 1]
                curr = sactions[af]
                if not np.isnan(prev).any() and not np.isnan(curr).any():
                    jumps.append(np.abs(curr - prev))

        if not jumps:
            continue

        jumps_arr = np.array(jumps)
        means = [float(np.mean(jumps_arr[:, s:e])) for _, s, e in BODY_PART_GROUPS]
        ax.bar(x + si * width, means, width, label=sname,
               color=COLORS[si % len(COLORS)], alpha=0.8)

    ax.set_xticks(x + width * (n_strategies - 1) / 2)
    ax.set_xticklabels(body_parts, rotation=30)
    ax.set_ylabel('Mean |Δaction| at chunk boundary')
    ax.set_title('Chunk Boundary Discontinuity by Body Part')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2, axis='y')

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved plot: {output_path}')


def plot_zoomed_comparison(
    results: dict[str, np.ndarray],
    gt_actions: np.ndarray,
    output_path: Path,
    output_rate_hz: float = 0.0,
):
    """Zoomed view on the highest-activity region of right_shoulder_pitch."""
    jidx = 14  # right_shoulder_pitch
    gt_series = gt_actions[:, jidx]
    gt_time_s = np.arange(len(gt_actions)) * ACTION_STEP_PERIOD

    # Find peak activity region (in GT time)
    rolling_std = pd.Series(gt_series).rolling(window=30, center=True).std()
    peak_idx = int(rolling_std.idxmax()) if not rolling_std.isna().all() else len(gt_series) // 2
    zoom_start_t = max(0, (peak_idx - 60)) * ACTION_STEP_PERIOD
    zoom_end_t = min(len(gt_actions), peak_idx + 60) * ACTION_STEP_PERIOD

    fig, ax = plt.subplots(figsize=(16, 12))

    # GT
    gt_mask = (gt_time_s >= zoom_start_t) & (gt_time_s <= zoom_end_t)
    ax.plot(gt_time_s[gt_mask], gt_series[gt_mask], color='black', alpha=0.4,
            linewidth=2, label='GT')

    # Strategies
    for ci, (sname, sactions) in enumerate(results.items()):
        n_samples = len(sactions)
        if output_rate_hz > 0 and n_samples != len(gt_actions):
            total_dur = (len(gt_actions) - 1) * ACTION_STEP_PERIOD
            s_time = np.linspace(0, total_dur, n_samples)
        else:
            s_time = np.arange(n_samples) * ACTION_STEP_PERIOD
        s_mask = (s_time >= zoom_start_t) & (s_time <= zoom_end_t)
        valid = s_mask & ~np.isnan(sactions[:, jidx])
        ax.plot(s_time[valid], sactions[valid, jidx],
                color=COLORS[ci % len(COLORS)],
                alpha=0.85, linewidth=1.2, label=sname)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('right_shoulder_pitch')
    ax.set_title('Zoomed: Right Shoulder Pitch (Peak Activity Region)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved plot: {output_path}')


# ── Scoreboard ────────────────────────────────────────────────────────

def print_scoreboard(
    all_metrics: dict[str, dict],
    inference_rate: float,
):
    """Print ranked strategy comparison."""
    print()
    print('=' * 90)
    print(f'  SMOOTHING STRATEGY SCOREBOARD @ {inference_rate:.1f}Hz')
    print('=' * 90)

    strategy_names = list(all_metrics.keys())

    # Header
    W = 26  # strategy name column width
    print()
    print(f'  {"Strategy":<{W}} {"MAE":>8} {"Jerk":>8} '
          f'{"BndJmp":>8} {"DeltaRMS":>8} '
          f'{"PogoRSP":>8} {"PogoREP":>8}')
    print(f'  {"-"*W} {"-"*8} {"-"*8} {"-"*8} {"-"*8} {"-"*8} {"-"*8}')

    # Sort by overall MAE
    sorted_names = sorted(
        strategy_names,
        key=lambda n: all_metrics[n].get('mae_overall', float('inf')),
    )

    for sname in sorted_names:
        m = all_metrics[sname]
        print(f'  {sname:<{W}} '
              f'{m.get("mae_overall", float("nan")):>8.5f} '
              f'{m.get("jerk_rms", float("nan")):>8.5f} '
              f'{m.get("boundary_jump_mean", float("nan")):>8.5f} '
              f'{m.get("action_delta_rms", float("nan")):>8.5f} '
              f'{m.get("pogo_right_shoulder_pitch", float("nan")):>8.3f} '
              f'{m.get("pogo_right_elbow_pitch", float("nan")):>8.3f}')

    # Per-body-part MAE breakdown
    print()
    header = f'  {"Strategy":<{W}}'
    for name, _, _ in BODY_PART_GROUPS:
        header += f' {name:>10}'
    print(header)
    print(f'  {"-"*W}' + f' {"-"*10}' * len(BODY_PART_GROUPS))

    for sname in sorted_names:
        m = all_metrics[sname]
        line = f'  {sname:<{W}}'
        for name, _, _ in BODY_PART_GROUPS:
            val = m.get(f'mae_{name}', float('nan'))
            line += f' {val:>10.5f}'
        print(line)

    print()
    print('=' * 90)
    print()


def save_scoreboard_csv(
    all_metrics: dict[str, dict],
    inference_rate: float,
    output_path: Path,
):
    """Save scoreboard as CSV."""
    rows = []
    for sname, m in all_metrics.items():
        row = {'strategy': sname, 'inference_rate_hz': inference_rate}
        row.update(m)
        rows.append(row)
    df = pd.DataFrame(rows)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(output_path, index=False)
    logger.info(f'Saved scoreboard: {output_path}')


# ── Main ──────────────────────────────────────────────────────────────

def process_episode(
    args,
    episode_index: int,
    inference_rate: float,
    strategy_names: list[str],
    latency_frames: int,
) -> dict:
    """Process a single episode at a given inference rate."""
    # Load GT data
    states, gt_actions = load_episode(args.dataset_path, episode_index)
    num_frames = len(states)

    # Load cached horizons
    if args.horizons:
        horizons_path = Path(args.horizons)
    else:
        horizons_dir = Path(args.horizons_dir)
        horizons_path = horizons_dir / f'horizons_ep{episode_index}.npz'

    if not horizons_path.exists():
        logger.error(f'Horizons not found: {horizons_path}')
        return None

    horizons, latencies = load_horizons(horizons_path)
    if len(horizons) != num_frames:
        logger.warning(
            f'Ep {episode_index}: horizons ({len(horizons)}) != '
            f'frames ({num_frames}). Using min.'
        )
        num_frames = min(len(horizons), num_frames)
        states = states[:num_frames]
        gt_actions = gt_actions[:num_frames]
        horizons = horizons[:num_frames]

    valid_count = sum(1 for h in horizons if h is not None)
    logger.info(
        f'Ep {episode_index} @ {inference_rate:.1f}Hz: '
        f'{num_frames} frames, {valid_count} valid horizons'
    )

    # Simulate continuous inference chunks
    chunks = simulate_continuous_chunks(
        horizons, num_frames, inference_rate, latency_frames,
    )
    logger.info(f'  Simulated {len(chunks)} chunks '
                f'(interval={TRAINING_FPS/inference_rate:.1f} frames)')

    # Run each strategy
    strategy_results = {}
    strategy_metrics = {}
    strategy_chunks = {}

    output_rate = getattr(args, 'output_rate', 0.0)

    for sname in strategy_names:
        preset = STRATEGY_PRESETS.get(sname)
        if preset is None:
            logger.warning(f'Unknown strategy: {sname}, skipping')
            continue

        t0 = time.monotonic()
        sactions = run_strategy(
            chunks, num_frames, preset,
            latency_skip=args.latency_skip,
            output_rate_hz=output_rate,
        )
        elapsed = time.monotonic() - t0

        metrics = compute_metrics(
            sactions, gt_actions, chunks, output_rate_hz=output_rate,
        )
        strategy_results[sname] = sactions
        strategy_metrics[sname] = metrics
        strategy_chunks[sname] = chunks

        logger.info(
            f'  {sname:<24} MAE={metrics.get("mae_overall", 0):.5f}  '
            f'jerk={metrics.get("jerk_rms", 0):.5f}  '
            f'bnd_jump={metrics.get("boundary_jump_mean", 0):.5f}  '
            f'({elapsed*1000:.0f}ms)'
        )

    return {
        'episode': episode_index,
        'num_frames': num_frames,
        'gt_actions': gt_actions,
        'gt_states': states,
        'chunks': chunks,
        'strategy_results': strategy_results,
        'strategy_metrics': strategy_metrics,
        'strategy_chunks': strategy_chunks,
    }


def aggregate_episode_metrics(
    all_episode_results: list[dict],
) -> dict[str, dict]:
    """Aggregate metrics across episodes (weighted by frame count)."""
    if not all_episode_results:
        return {}

    strategy_names = list(all_episode_results[0]['strategy_metrics'].keys())
    aggregated = {}

    for sname in strategy_names:
        total_frames = 0
        weighted_sums = {}

        for ep_result in all_episode_results:
            m = ep_result['strategy_metrics'].get(sname, {})
            n = m.get('n_valid_frames', 0)
            if n == 0:
                continue
            total_frames += n
            for key, val in m.items():
                if isinstance(val, (int, float)) and key != 'n_valid_frames' and key != 'n_total_frames':
                    if not np.isnan(val):
                        weighted_sums[key] = weighted_sums.get(key, 0) + val * n

        agg = {'n_valid_frames': total_frames}
        for key, wsum in weighted_sums.items():
            agg[key] = wsum / total_frames if total_frames > 0 else float('nan')
        aggregated[sname] = agg

    return aggregated


def main():
    parser = argparse.ArgumentParser(
        description='Offline action smoothing strategy simulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--dataset-path', required=True,
                        help='Path to LeRobot dataset')
    parser.add_argument('--horizons', default=None,
                        help='Path to single horizons .npz file')
    parser.add_argument('--horizons-dir', default='/tmp/overlapped_test/',
                        help='Directory containing horizons_epN.npz files')
    parser.add_argument('--episode-index', type=int, default=None,
                        help='Single episode index')
    parser.add_argument('--episode-indices', type=str, default=None,
                        help='Comma-separated episode indices')
    parser.add_argument('--inference-rates', type=str, default='3.3,5.0',
                        help='Comma-separated inference rates in Hz')
    parser.add_argument('--strategies', type=str,
                        default='latest,uniform,exp_m01,exp_m1,exp_m5,triangle,exp_m1_ema90',
                        help='Comma-separated strategy names')
    parser.add_argument('--latency-ms', type=float, default=280,
                        help='Round-trip inference latency in ms')
    parser.add_argument('--latency-skip', type=int, default=0,
                        help='Actions to skip at start of each chunk '
                             '(0 = no skip, use for offline sim since '
                             'horizons already account for observation timing)')
    parser.add_argument('--output-dir', default='/tmp/smoothing_results/',
                        help='Output directory')
    parser.add_argument('--output-rate', type=float, default=0.0,
                        help='Output sample rate in Hz (0 = 15 FPS native, '
                             '100 = 100 Hz upsampled). Use 100 to benchmark '
                             'interpolation methods (spline vs linear).')
    parser.add_argument('--save-csv', action='store_true',
                        help='Save per-strategy CSVs (compatible with pogo analysis)')
    parser.add_argument('--verbose', '-v', action='store_true')
    args = parser.parse_args()

    # Resolve episodes
    if args.episode_indices:
        episode_indices = [int(x.strip()) for x in args.episode_indices.split(',')]
    elif args.episode_index is not None:
        episode_indices = [args.episode_index]
    else:
        episode_indices = [320]

    inference_rates = [float(x.strip()) for x in args.inference_rates.split(',')]
    strategy_names = [x.strip() for x in args.strategies.split(',')]

    # Validate strategies
    for sname in strategy_names:
        if sname not in STRATEGY_PRESETS:
            logger.error(f'Unknown strategy: {sname}. '
                         f'Available: {list(STRATEGY_PRESETS.keys())}')
            sys.exit(1)

    latency_frames = round(args.latency_ms / (ACTION_STEP_PERIOD * 1000))
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S',
    )

    print()
    print('=' * 70)
    print('  Action Smoothing Strategy Simulator')
    print('=' * 70)
    print(f'  Episodes:        {episode_indices}')
    print(f'  Inference rates: {inference_rates} Hz')
    print(f'  Strategies:      {strategy_names}')
    print(f'  Latency:         {args.latency_ms}ms ({latency_frames} frames)')
    print(f'  Latency skip:    {args.latency_skip}')
    print(f'  Output rate:     {"15 FPS (native)" if args.output_rate <= 0 else f"{args.output_rate:.0f} Hz"}')
    print(f'  Output:          {output_dir}')
    print('=' * 70)
    print()

    # Process each inference rate
    for rate in inference_rates:
        rate_dir = output_dir / f'rate_{rate:.1f}Hz'
        rate_dir.mkdir(parents=True, exist_ok=True)

        all_episode_results = []

        for ep_idx in episode_indices:
            logger.info(f'\n--- Episode {ep_idx} @ {rate:.1f}Hz ---')
            result = process_episode(
                args, ep_idx, rate, strategy_names, latency_frames,
            )
            if result is None:
                continue
            all_episode_results.append(result)

            # Save per-episode outputs
            ep_dir = rate_dir / f'ep{ep_idx}'

            # Trajectory comparison plot
            plot_strategy_comparison(
                result['strategy_results'],
                result['gt_actions'],
                rate,
                ep_dir / 'trajectory_comparison.png',
                output_rate_hz=args.output_rate,
            )

            # Zoomed view
            plot_zoomed_comparison(
                result['strategy_results'],
                result['gt_actions'],
                ep_dir / 'zoomed_comparison.png',
                output_rate_hz=args.output_rate,
            )

            # Boundary jumps
            plot_boundary_jumps(
                result['strategy_results'],
                result['strategy_chunks'],
                ep_dir / 'boundary_jumps.png',
            )

            # Per-strategy CSVs
            if args.save_csv:
                for sname, sactions in result['strategy_results'].items():
                    strategy_to_csv(
                        sactions,
                        result['gt_states'],
                        result['gt_actions'],
                        result['chunks'],
                        ep_dir / f'{sname}.csv',
                    )

            # Save GT actions for analysis tool
            np.save(ep_dir / 'gt_actions.npy', result['gt_actions'])
            np.save(ep_dir / 'gt_states.npy', result['gt_states'])

        if not all_episode_results:
            logger.error(f'No episodes processed for rate {rate:.1f}Hz')
            continue

        # Aggregate metrics
        agg_metrics = aggregate_episode_metrics(all_episode_results)

        # Print and save scoreboard
        print_scoreboard(agg_metrics, rate)
        save_scoreboard_csv(agg_metrics, rate, rate_dir / 'scoreboard.csv')

        # Per-episode metric breakdown
        if len(all_episode_results) > 1:
            print(f'  Per-episode MAE (right_arm):')
            print(f'  {"Episode":<10}', end='')
            for sname in strategy_names:
                print(f' {sname:>12}', end='')
            print()

            for ep_result in all_episode_results:
                print(f'  {ep_result["episode"]:<10}', end='')
                for sname in strategy_names:
                    m = ep_result['strategy_metrics'].get(sname, {})
                    val = m.get('mae_right_arm', float('nan'))
                    print(f' {val:>12.5f}', end='')
                print()
            print()

    print(f'\nAll outputs saved to: {output_dir}/')


if __name__ == '__main__':
    main()
