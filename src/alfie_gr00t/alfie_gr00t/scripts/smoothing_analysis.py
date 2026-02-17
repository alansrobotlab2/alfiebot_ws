#!/usr/bin/env python3
"""Unified Action Smoothing Analysis Tool.

Analyzes smoothed action trajectories from either:
  - Simulator output CSVs (with GT comparison)
  - Live robot capture CSVs (smoothness metrics only)

Extends the pogo analysis patterns with multi-strategy comparison,
GT metrics, and a ranked scoreboard.

Usage:
    # Compare simulator strategies against GT
    python smoothing_analysis.py \
        --csv latest=/tmp/smoothing_results/latest.csv,exp_m1=/tmp/smoothing_results/exp_m1.csv \
        --gt-actions /tmp/smoothing_results/gt_actions.npy \
        --output-dir /tmp/smoothing_analysis/

    # Analyze single live capture
    python smoothing_analysis.py \
        --csv live=/tmp/groot_client_debug.csv \
        --output-dir /tmp/smoothing_analysis/

    # Analyze multiple live captures for comparison
    python smoothing_analysis.py \
        --csv run1=/tmp/run1.csv,run2=/tmp/run2.csv \
        --output-dir /tmp/smoothing_analysis/
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.scripts.overlapped_execution_test import (
    BODY_PART_GROUPS,
    JOINT_NAMES,
)

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────

FOCUS_JOINTS = [
    (0,  'cmd_vel_lx',           'Base Forward Vel'),
    (14, 'right_shoulder_pitch', 'R Shoulder Pitch'),
    (15, 'right_elbow_pitch',    'R Elbow Pitch'),
    (16, 'right_wrist_pitch',    'R Wrist Pitch'),
    (18, 'right_gripper',        'R Gripper'),
    (19, 'head_yaw',             'Head Yaw'),
]

COLORS = [
    '#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd',
    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf',
]


# ── Data loading ──────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
    """Load a CSV file (either live debug or simulator output format)."""
    df = pd.read_csv(path)
    logger.info(f'Loaded {path}: {len(df)} rows, {len(df.columns)} columns')
    return df


def extract_columns(df: pd.DataFrame, prefix: str) -> np.ndarray | None:
    """Extract 22D joint array from DataFrame columns with given prefix.

    Looks for columns like '{prefix}_{joint_name}' for all 22 joints.
    Returns (N, 22) array or None if columns not found.
    """
    cols = [f'{prefix}_{jname}' for jname in JOINT_NAMES]
    if all(c in df.columns for c in cols):
        return df[cols].values.astype(np.float64)
    return None


def get_time_axis(df: pd.DataFrame) -> np.ndarray:
    """Get relative time axis in seconds."""
    if 'timestamp' in df.columns:
        t = df['timestamp'].values
        return t - t[0]
    return np.arange(len(df)) * (1.0 / 15)


def get_chunk_boundaries(df: pd.DataFrame) -> list[int]:
    """Find row indices where chunk_id changes."""
    if 'chunk_id' not in df.columns:
        return []
    boundaries = df.index[df['chunk_id'].diff() != 0].tolist()
    if 0 not in boundaries:
        boundaries = [0] + boundaries
    return boundaries


# ── Metrics ───────────────────────────────────────────────────────────

def compute_smoothness_metrics(
    actions: np.ndarray,
    chunk_boundaries: list[int],
) -> dict:
    """Compute smoothness metrics (no GT needed)."""
    metrics = {}

    # Action delta RMS (frame-to-frame)
    diffs = np.diff(actions, axis=0)
    valid_diffs = diffs[~np.isnan(diffs).any(axis=1)]
    if len(valid_diffs) > 0:
        metrics['action_delta_rms'] = float(np.sqrt(np.mean(valid_diffs ** 2)))
    else:
        metrics['action_delta_rms'] = float('nan')

    # Jerk RMS (2nd derivative)
    if len(valid_diffs) > 1:
        jerk = np.diff(valid_diffs, axis=0)
        metrics['jerk_rms'] = float(np.sqrt(np.mean(jerk ** 2)))
    else:
        metrics['jerk_rms'] = float('nan')

    # Per-body-part smoothness
    for name, start, end in BODY_PART_GROUPS:
        part_diffs = diffs[:, start:end]
        valid = ~np.isnan(part_diffs).any(axis=1)
        if valid.sum() > 0:
            metrics[f'delta_rms_{name}'] = float(
                np.sqrt(np.mean(part_diffs[valid] ** 2))
            )
        else:
            metrics[f'delta_rms_{name}'] = float('nan')

    # Boundary jump analysis
    if chunk_boundaries:
        jumps = []
        for bi in chunk_boundaries[1:]:
            if 0 < bi < len(actions):
                prev = actions[bi - 1]
                curr = actions[bi]
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

    # Pogo scores for key joints
    for jname, jidx in [('right_shoulder_pitch', 14),
                         ('right_elbow_pitch', 15),
                         ('right_wrist_pitch', 16)]:
        boundary_jumps = []
        for bi in chunk_boundaries[1:]:
            if 0 < bi < len(actions):
                prev = actions[bi - 1, jidx]
                curr = actions[bi, jidx]
                if not np.isnan(prev) and not np.isnan(curr):
                    boundary_jumps.append(curr - prev)

        if len(boundary_jumps) > 1:
            signs = np.sign(boundary_jumps)
            alternations = np.sum(np.abs(np.diff(signs)) == 2)
            metrics[f'pogo_{jname}'] = alternations / (len(signs) - 1)
        else:
            metrics[f'pogo_{jname}'] = 0.0

    return metrics


def compute_gt_metrics(
    actions: np.ndarray,
    gt_actions: np.ndarray,
) -> dict:
    """Compute GT comparison metrics."""
    n = min(len(actions), len(gt_actions))
    actions = actions[:n]
    gt_actions = gt_actions[:n]

    valid = ~np.isnan(actions).any(axis=1) & ~np.isnan(gt_actions).any(axis=1)
    metrics = {}

    if valid.sum() == 0:
        return metrics

    # Overall MAE
    metrics['mae_overall'] = float(np.mean(np.abs(
        actions[valid] - gt_actions[valid]
    )))

    # Per-body-part MAE
    for name, start, end in BODY_PART_GROUPS:
        metrics[f'mae_{name}'] = float(np.mean(np.abs(
            actions[valid, start:end] - gt_actions[valid, start:end]
        )))

    # Per-joint MAE for focus joints
    for jidx, jname, _ in FOCUS_JOINTS:
        metrics[f'mae_{jname}'] = float(np.mean(np.abs(
            actions[valid, jidx] - gt_actions[valid, jidx]
        )))

    return metrics


# ── Plotting ──────────────────────────────────────────────────────────

def plot_trajectory_overlay(
    datasets: dict[str, dict],
    gt_actions: np.ndarray | None,
    output_path: Path,
):
    """Overlay trajectory plots for all datasets on key joints."""
    n_joints = len(FOCUS_JOINTS)
    fig, axes = plt.subplots(n_joints, 1, figsize=(20, 6.4 * n_joints),
                             sharex=True)

    for i, (jidx, jname, title) in enumerate(FOCUS_JOINTS):
        ax = axes[i]

        # GT reference
        if gt_actions is not None:
            n_gt = len(gt_actions)
            t_gt = np.arange(n_gt) * (1.0 / 15)
            ax.plot(t_gt, gt_actions[:, jidx], color='black', alpha=0.3,
                    linewidth=1.5, label='GT', zorder=1)

        # Each dataset
        for ci, (dname, ddata) in enumerate(datasets.items()):
            t = ddata['time']
            actions = ddata['smoothed']  # Use smoothed if available
            if actions is None:
                actions = ddata['action']
            if actions is None:
                continue

            valid = ~np.isnan(actions[:, jidx])
            color = COLORS[ci % len(COLORS)]
            ax.plot(t[valid], actions[valid, jidx],
                    color=color, alpha=0.8, linewidth=1.0, label=dname,
                    zorder=2)

            # Chunk boundaries
            boundaries = ddata.get('chunk_boundaries', [])
            boundary_times = t[boundaries] if len(boundaries) > 0 else []
            for bt in boundary_times:
                ax.axvline(x=bt, color=color, alpha=0.15, linewidth=0.5,
                           linestyle='--')

        ax.set_ylabel(jname, fontsize=8)
        ax.set_title(title, fontsize=9, loc='left')
        ax.grid(True, alpha=0.2)
        if i == 0:
            ax.legend(loc='upper right', fontsize=7,
                      ncol=min(6, len(datasets) + 1))

    axes[-1].set_xlabel('Time (s)')
    plt.suptitle('Strategy Trajectory Comparison', fontsize=12, y=0.99)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {output_path}')


def plot_zoomed_overlay(
    datasets: dict[str, dict],
    gt_actions: np.ndarray | None,
    output_path: Path,
):
    """Zoomed view on peak-activity region for right arm joints."""
    zoom_joints = [
        (14, 'right_shoulder_pitch', 'R Shoulder Pitch'),
        (15, 'right_elbow_pitch', 'R Elbow Pitch'),
        (18, 'right_gripper', 'R Gripper'),
    ]

    fig, axes = plt.subplots(len(zoom_joints), 1,
                             figsize=(18, 8 * len(zoom_joints)), sharex=True)

    # Find peak region from first dataset or GT
    ref = gt_actions if gt_actions is not None else list(datasets.values())[0].get('smoothed')
    if ref is None:
        ref = list(datasets.values())[0].get('action')
    if ref is None:
        return

    # Use right_shoulder_pitch for peak detection
    series = ref[:, 14]
    rolling_std = pd.Series(series).rolling(window=30, center=True).std()
    peak = int(rolling_std.idxmax()) if not rolling_std.isna().all() else len(series) // 2
    zoom_start = max(0, peak - 60)
    zoom_end = min(len(ref), peak + 60)

    for i, (jidx, jname, title) in enumerate(zoom_joints):
        ax = axes[i]
        sl = slice(zoom_start, zoom_end)

        if gt_actions is not None:
            t_gt = np.arange(len(gt_actions)) * (1.0 / 15)
            ax.plot(t_gt[sl], gt_actions[sl, jidx], color='black',
                    alpha=0.4, linewidth=2, label='GT')

        for ci, (dname, ddata) in enumerate(datasets.items()):
            t = ddata['time']
            actions = ddata['smoothed'] if ddata['smoothed'] is not None else ddata['action']
            if actions is None:
                continue

            # Slice to zoom window (approximate by time alignment)
            t_start = zoom_start / 15.0
            t_end = zoom_end / 15.0
            mask = (t >= t_start) & (t <= t_end) & ~np.isnan(actions[:, jidx])
            color = COLORS[ci % len(COLORS)]
            ax.plot(t[mask], actions[mask, jidx],
                    color=color, alpha=0.85, linewidth=1.2, label=dname)

        ax.set_ylabel(jname, fontsize=9)
        ax.set_title(f'{title} (zoomed)', fontsize=10, loc='left')
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(fontsize=8)

    axes[-1].set_xlabel('Time (s)')
    plt.suptitle('Zoomed: Right Arm (Peak Activity Region)', fontsize=12)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {output_path}')


def plot_boundary_analysis(
    datasets: dict[str, dict],
    output_path: Path,
):
    """Boundary jump comparison across datasets for right arm joints."""
    analysis_joints = [
        (14, 'right_shoulder_pitch'),
        (15, 'right_elbow_pitch'),
        (16, 'right_wrist_pitch'),
        (18, 'right_gripper'),
    ]

    fig, axes = plt.subplots(len(analysis_joints), 1,
                             figsize=(18, 8 * len(analysis_joints)),
                             sharex=True)

    for ji, (jidx, jname) in enumerate(analysis_joints):
        ax = axes[ji]

        for ci, (dname, ddata) in enumerate(datasets.items()):
            actions = ddata['smoothed'] if ddata['smoothed'] is not None else ddata['action']
            if actions is None:
                continue

            t = ddata['time']
            boundaries = ddata.get('chunk_boundaries', [])
            color = COLORS[ci % len(COLORS)]

            # Plot trajectory
            valid = ~np.isnan(actions[:, jidx])
            ax.plot(t[valid], actions[valid, jidx],
                    color=color, alpha=0.6, linewidth=0.8, label=dname)

            # Mark boundary jumps
            for bi in boundaries[1:]:
                if 0 < bi < len(actions):
                    prev = actions[bi - 1, jidx]
                    curr = actions[bi, jidx]
                    if not np.isnan(prev) and not np.isnan(curr):
                        jump = curr - prev
                        if abs(jump) > 0.005:
                            ax.axvline(x=t[bi], color=color, alpha=0.3,
                                       linewidth=0.8)

        ax.set_ylabel(jname.replace('right_', 'R_'), fontsize=9)
        ax.set_title(f'{jname}: Chunk Boundary Behavior', fontsize=10)
        ax.grid(True, alpha=0.2)
        if ji == 0:
            ax.legend(fontsize=8)

    axes[-1].set_xlabel('Time (s)')
    plt.suptitle('Chunk Boundary Analysis', fontsize=12)
    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {output_path}')


# ── Scoreboard ────────────────────────────────────────────────────────

def print_scoreboard(
    all_metrics: dict[str, dict],
    has_gt: bool,
):
    """Print analysis scoreboard."""
    print()
    print('=' * 90)
    print('  SMOOTHING ANALYSIS SCOREBOARD')
    print('=' * 90)

    names = list(all_metrics.keys())

    if has_gt:
        print()
        print(f'  {"Name":<18} {"MAE":>8} {"Jerk":>8} '
              f'{"BndJmp":>8} {"DeltaRMS":>8} '
              f'{"PogoRSP":>8} {"PogoREP":>8}')
        print(f'  {"-"*18} {"-"*8} {"-"*8} {"-"*8} {"-"*8} {"-"*8} {"-"*8}')

        sorted_names = sorted(
            names,
            key=lambda n: all_metrics[n].get('mae_overall', float('inf')),
        )
    else:
        print()
        print(f'  {"Name":<18} {"Jerk":>8} '
              f'{"BndJmp":>8} {"DeltaRMS":>8} '
              f'{"PogoRSP":>8} {"PogoREP":>8}')
        print(f'  {"-"*18} {"-"*8} {"-"*8} {"-"*8} {"-"*8} {"-"*8}')

        sorted_names = sorted(
            names,
            key=lambda n: all_metrics[n].get('jerk_rms', float('inf')),
        )

    for sname in sorted_names:
        m = all_metrics[sname]
        if has_gt:
            print(f'  {sname:<18} '
                  f'{m.get("mae_overall", float("nan")):>8.5f} '
                  f'{m.get("jerk_rms", float("nan")):>8.5f} '
                  f'{m.get("boundary_jump_mean", float("nan")):>8.5f} '
                  f'{m.get("action_delta_rms", float("nan")):>8.5f} '
                  f'{m.get("pogo_right_shoulder_pitch", float("nan")):>8.3f} '
                  f'{m.get("pogo_right_elbow_pitch", float("nan")):>8.3f}')
        else:
            print(f'  {sname:<18} '
                  f'{m.get("jerk_rms", float("nan")):>8.5f} '
                  f'{m.get("boundary_jump_mean", float("nan")):>8.5f} '
                  f'{m.get("action_delta_rms", float("nan")):>8.5f} '
                  f'{m.get("pogo_right_shoulder_pitch", float("nan")):>8.3f} '
                  f'{m.get("pogo_right_elbow_pitch", float("nan")):>8.3f}')

    # Per-body-part MAE (only if GT available)
    if has_gt:
        print()
        header = f'  {"Name":<18}'
        for name, _, _ in BODY_PART_GROUPS:
            header += f' {name:>10}'
        print(header)
        print(f'  {"-"*18}' + f' {"-"*10}' * len(BODY_PART_GROUPS))

        for sname in sorted_names:
            m = all_metrics[sname]
            line = f'  {sname:<18}'
            for name, _, _ in BODY_PART_GROUPS:
                val = m.get(f'mae_{name}', float('nan'))
                line += f' {val:>10.5f}'
            print(line)

    print()
    print('=' * 90)
    print()


# ── Main ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Unified action smoothing analysis tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--csv', required=True,
        help='Comma-separated name=path pairs: '
             'e.g. "latest=/tmp/latest.csv,exp_m1=/tmp/exp_m1.csv"'
    )
    parser.add_argument(
        '--gt-actions', default=None,
        help='Path to GT actions .npy file (enables GT comparison metrics)'
    )
    parser.add_argument(
        '--output-dir', default='/tmp/smoothing_analysis/',
        help='Output directory for plots and scoreboard'
    )
    parser.add_argument('--verbose', '-v', action='store_true')
    args = parser.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S',
    )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Parse CSV inputs
    datasets = {}
    for entry in args.csv.split(','):
        entry = entry.strip()
        if '=' in entry:
            name, path = entry.split('=', 1)
        else:
            name = Path(entry).stem
            path = entry

        df = load_csv(path)
        t = get_time_axis(df)
        boundaries = get_chunk_boundaries(df)

        # Extract arrays — try smoothed first, then action
        smoothed = extract_columns(df, 'smoothed')
        action = extract_columns(df, 'action')
        state = extract_columns(df, 'state')

        datasets[name] = {
            'df': df,
            'time': t,
            'smoothed': smoothed,
            'action': action,
            'state': state,
            'chunk_boundaries': boundaries,
        }

    # Load GT if available
    gt_actions = None
    has_gt = False
    if args.gt_actions:
        gt_path = Path(args.gt_actions)
        if gt_path.exists():
            gt_actions = np.load(gt_path)
            has_gt = True
            logger.info(f'Loaded GT actions: {gt_actions.shape}')
        else:
            logger.warning(f'GT actions not found: {gt_path}')

    print()
    print('=' * 70)
    print('  Smoothing Analysis')
    print('=' * 70)
    print(f'  Datasets:  {list(datasets.keys())}')
    print(f'  GT:        {"yes" if has_gt else "no (smoothness metrics only)"}')
    print(f'  Output:    {output_dir}')
    print('=' * 70)

    # Compute metrics for each dataset
    all_metrics = {}
    for dname, ddata in datasets.items():
        # Use smoothed data if available, otherwise raw action
        actions = ddata['smoothed'] if ddata['smoothed'] is not None else ddata['action']
        if actions is None:
            logger.warning(f'{dname}: no action data found, skipping')
            continue

        metrics = compute_smoothness_metrics(
            actions, ddata['chunk_boundaries']
        )

        if has_gt and gt_actions is not None:
            gt_metrics = compute_gt_metrics(actions, gt_actions)
            metrics.update(gt_metrics)

        all_metrics[dname] = metrics

    # Print scoreboard
    print_scoreboard(all_metrics, has_gt)

    # Save scoreboard CSV
    rows = []
    for dname, m in all_metrics.items():
        row = {'name': dname}
        row.update(m)
        rows.append(row)
    scoreboard_df = pd.DataFrame(rows)
    scoreboard_path = output_dir / 'scoreboard.csv'
    scoreboard_df.to_csv(scoreboard_path, index=False)
    logger.info(f'Saved: {scoreboard_path}')

    # Plots
    plot_trajectory_overlay(datasets, gt_actions, output_dir / 'trajectory_overlay.png')
    plot_zoomed_overlay(datasets, gt_actions, output_dir / 'zoomed_overlay.png')
    plot_boundary_analysis(datasets, output_dir / 'boundary_analysis.png')

    print(f'All outputs saved to: {output_dir}/')


if __name__ == '__main__':
    main()
