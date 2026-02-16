#!/usr/bin/env python3
"""Overlapped Execution Strategy Test.

Simulates and compares two inference execution strategies using recorded
episode data and the ZMQ inference server:

1. SEQUENTIAL: Execute 16-step chunk (skip=4 → 12 useful), post-exhaust
   observation, ~280ms dead time per cycle. Cycle = 16 frames.
2. OVERLAPPED: Execute 8-step window, fire inference at window step 3
   (mid-chunk), result arrives before chunk exhausts. Cycle = 8 frames.
3. ORACLE: Fresh inference at every frame (zero-latency upper bound).

Produces trajectory comparison plots, per-body-part MAE metrics,
chunk boundary analysis, and observation staleness profiles.

No ROS2 dependency — runs standalone against the ZMQ inference server.

Usage:
    python overlapped_execution_test.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 3 \
        --host 192.168.50.201 --port 5555 \
        --save-dir /tmp/overlapped_test/
"""

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

try:
    import torchcodec.decoders
    HAS_TORCHCODEC = True
except ImportError:
    HAS_TORCHCODEC = False

try:
    import av
    HAS_AV = True
except ImportError:
    HAS_AV = False

# Import ZMQ client (no ROS2 dependency)
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.zmq_client import ZMQClient, build_server_address

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────

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

CAMERA_NAMES = ['left_wide', 'right_wide', 'left_center', 'right_center']

BODY_PART_GROUPS = [
    ('base',       0,  6),
    ('back',       6,  7),
    ('left_arm',   7, 12),
    ('left_hand', 12, 13),
    ('right_arm', 13, 18),
    ('right_hand', 18, 19),
    ('head',      19, 22),
]

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
JPEG_QUALITY = 80
ACTION_STEP_MS = 67  # 15 FPS

# Key joints for trajectory plots
KEY_JOINTS = [
    (0,  'cmd_vel_lx',           'Base Forward Velocity'),
    (5,  'cmd_vel_az',           'Base Angular Velocity'),
    (14, 'right_shoulder_pitch', 'Right Shoulder Pitch'),
    (18, 'right_gripper',        'Right Gripper'),
    (19, 'head_yaw',             'Head Yaw'),
]


# ── Data loading (shared with action_horizon_analysis.py) ────────────

def load_episode(dataset_path: str, episode_index: int):
    """Load episode states and actions from parquet."""
    dataset_path = Path(dataset_path)
    parquet_path = dataset_path / f'data/episode_{episode_index:06d}.parquet'
    if not parquet_path.exists():
        chunk_idx = episode_index // 1000
        parquet_path = (
            dataset_path / f'data/chunk-{chunk_idx:03d}/episode_{episode_index:06d}.parquet'
        )
    if not parquet_path.exists():
        raise FileNotFoundError(f'Episode parquet not found: {parquet_path}')

    df = pd.read_parquet(parquet_path, engine='pyarrow')
    states = np.array(df['observation.state'].tolist(), dtype=np.float32)
    actions = np.array(df['action'].tolist(), dtype=np.float32)
    logger.info(f'Loaded episode {episode_index}: {len(df)} frames')
    return states, actions


def load_video_frames(dataset_path: str, episode_index: int, num_frames: int):
    """Load video frames for all 4 cameras."""
    dataset_path = Path(dataset_path)
    chunk_idx = episode_index // 1000

    frames_per_camera = {}
    for cam_name in CAMERA_NAMES:
        video_path = (
            dataset_path
            / f'videos/chunk-{chunk_idx:03d}/observation.images.{cam_name}'
            / f'episode_{episode_index:06d}.mp4'
        )
        if not video_path.exists():
            raise FileNotFoundError(f'Video not found: {video_path}')

        if HAS_TORCHCODEC:
            decoder = torchcodec.decoders.VideoDecoder(
                str(video_path), device="cpu", dimension_order="NHWC",
                num_ffmpeg_threads=0,
            )
            frames_tensor = decoder.get_frames_at(
                indices=list(range(num_frames))
            ).data
            cam_frames = [frames_tensor[i].numpy() for i in range(len(frames_tensor))]
        elif HAS_AV:
            cam_frames = []
            container = av.open(str(video_path))
            for frame in container.decode(video=0):
                cam_frames.append(frame.to_ndarray(format='rgb24'))
                if len(cam_frames) >= num_frames:
                    break
            container.close()
        else:
            raise ImportError('Need torchcodec or av (PyAV) to decode video')

        frames_per_camera[cam_name] = cam_frames

    all_frames = []
    for i in range(num_frames):
        frame_dict = {}
        for cam_name in CAMERA_NAMES:
            if i < len(frames_per_camera[cam_name]):
                frame_dict[cam_name] = frames_per_camera[cam_name][i]
        all_frames.append(frame_dict)
    return all_frames


def compress_jpeg(img: np.ndarray, quality: int = JPEG_QUALITY) -> bytes:
    """Compress RGB image to JPEG bytes."""
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    _, encoded = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return encoded.tobytes()


# ── Horizon collection ────────────────────────────────────────────────

def collect_horizons(
    client: ZMQClient,
    video_frames: list[dict],
    states: np.ndarray,
    task: str,
) -> tuple[list[np.ndarray | None], list[float]]:
    """Run inference at every frame, return (horizons, latencies)."""
    num_frames = len(states)
    horizons = []
    latencies = []

    for t in range(num_frames):
        images = {}
        if t < len(video_frames):
            for cam_name, img in video_frames[t].items():
                images[cam_name] = compress_jpeg(img)

        t0 = time.monotonic()
        response = client.send_observation(
            images=images, state=states[t], language=task,
        )
        latency_ms = (time.monotonic() - t0) * 1000
        latencies.append(latency_ms)

        if response is None or response.get('status') != 'ok':
            horizons.append(None)
            continue

        actions_list = response.get('actions', [])
        if not actions_list:
            horizons.append(None)
            continue

        horizons.append(np.array(actions_list, dtype=np.float32))

        if (t + 1) % 50 == 0 or t == 0:
            logger.info(
                f'  [{t+1}/{num_frames}] latency={latency_ms:.0f}ms  '
                f'mean={np.mean(latencies):.0f}ms'
            )

    return horizons, latencies


def save_horizons(horizons, latencies, path):
    """Save horizons to .npz for caching."""
    # Convert None entries to NaN arrays
    horizon_arrays = []
    valid_mask = []
    for h in horizons:
        if h is not None:
            horizon_arrays.append(h)
            valid_mask.append(True)
        else:
            horizon_arrays.append(np.full((16, 22), np.nan, dtype=np.float32))
            valid_mask.append(False)
    np.savez_compressed(
        path,
        horizons=np.array(horizon_arrays),
        valid_mask=np.array(valid_mask),
        latencies=np.array(latencies),
    )
    logger.info(f'Saved horizons to {path}')


def load_horizons(path):
    """Load cached horizons from .npz."""
    data = np.load(path)
    horizon_arrays = data['horizons']
    valid_mask = data['valid_mask']
    latencies = data['latencies'].tolist()
    horizons = []
    for i in range(len(horizon_arrays)):
        if valid_mask[i]:
            horizons.append(horizon_arrays[i])
        else:
            horizons.append(None)
    logger.info(f'Loaded {len(horizons)} horizons from {path} '
                f'({sum(valid_mask)} valid)')
    return horizons, latencies


# ── Simulation ────────────────────────────────────────────────────────

def simulate_sequential(horizons, num_frames, skip=4, inference_delay=4):
    """Simulate sequential chunk execution with universal skip.

    Cycle: [dead × inference_delay] + [execute × (16 - skip)]
    Observation captured at chunk exhaust (post-exhaust).

    Returns:
        actions: (T, 22) — action delivered at each frame
        obs_frames: (T,) — observation frame that generated this action
        is_dead: (T,) bool — True during dead time
        boundaries: [(start_frame, obs_frame, skip), ...]
    """
    useful_steps = 16 - skip
    actions = np.full((num_frames, 22), np.nan)
    obs_frames_arr = np.full(num_frames, -1, dtype=int)
    is_dead = np.ones(num_frames, dtype=bool)
    boundaries = []

    obs_frame = 0
    t = 0

    while t < num_frames:
        # Dead time (inference in progress)
        for dt in range(inference_delay):
            f = t + dt
            if f >= num_frames:
                break
            # Hold: zero base velocity, keep last joint position
            if t == 0:
                actions[f] = np.zeros(22)
            else:
                hold = actions[t - 1].copy()
                hold[0:6] = 0.0
                actions[f] = hold
            obs_frames_arr[f] = obs_frame
            is_dead[f] = True

        # Chunk starts
        chunk_start = t + inference_delay
        if chunk_start >= num_frames:
            break

        horizon = horizons[obs_frame] if obs_frame < len(horizons) else None
        boundaries.append((chunk_start, obs_frame, skip))

        for step in range(useful_steps):
            f = chunk_start + step
            if f >= num_frames:
                break
            if horizon is not None:
                idx = min(skip + step, horizon.shape[0] - 1)
                actions[f] = horizon[idx]
            else:
                actions[f] = np.zeros(22)
            obs_frames_arr[f] = obs_frame
            is_dead[f] = False

        # Post-exhaust observation
        chunk_end = chunk_start + useful_steps
        obs_frame = min(chunk_end, num_frames - 1)
        t = chunk_end

    return actions, obs_frames_arr, is_dead, boundaries


def simulate_overlapped(horizons, num_frames, first_skip=4,
                        subsequent_skip=5, n_exec=8, trigger_at=3,
                        inference_delay=4):
    """Simulate overlapped chunk execution.

    First chunk: skip=first_skip (obs at frame 0, fresh).
    Subsequent: skip=subsequent_skip (obs captured mid-previous-chunk).
    Inference fired at window step trigger_at of each chunk.

    Returns same structure as simulate_sequential.
    """
    actions = np.full((num_frames, 22), np.nan)
    obs_frames_arr = np.full(num_frames, -1, dtype=int)
    is_dead = np.ones(num_frames, dtype=bool)
    boundaries = []

    obs_frame = 0
    skip = first_skip

    # Initial dead time (first inference)
    for f in range(min(inference_delay, num_frames)):
        actions[f] = np.zeros(22)
        obs_frames_arr[f] = 0
        is_dead[f] = True

    t = inference_delay
    while t < num_frames:
        horizon = horizons[obs_frame] if obs_frame < len(horizons) else None
        boundaries.append((t, obs_frame, skip))
        next_obs_frame = None

        for step in range(n_exec):
            f = t + step
            if f >= num_frames:
                break
            if horizon is not None:
                idx = min(skip + step, horizon.shape[0] - 1)
                actions[f] = horizon[idx]
            else:
                actions[f] = np.zeros(22)
            obs_frames_arr[f] = obs_frame
            is_dead[f] = False

            if step == trigger_at:
                next_obs_frame = f

        t += n_exec
        if next_obs_frame is not None and next_obs_frame < len(horizons):
            obs_frame = next_obs_frame
        skip = subsequent_skip

    return actions, obs_frames_arr, is_dead, boundaries


def oracle_actions(horizons, num_frames):
    """Zero-latency ideal: H[t][0] at every frame."""
    actions = np.full((num_frames, 22), np.nan)
    for t in range(num_frames):
        if t < len(horizons) and horizons[t] is not None:
            actions[t] = horizons[t][0]
    return actions


# ── Metrics ───────────────────────────────────────────────────────────

def compute_mae_table(strategy_actions, gt_actions, is_dead):
    """Per-body-part MAE vs GT, split by active/dead/total."""
    results = {}
    active = ~is_dead
    for name, start, end in BODY_PART_GROUPS:
        valid = ~np.isnan(strategy_actions[:, start])
        valid_active = valid & active
        valid_dead = valid & is_dead

        def _mae(mask):
            if mask.sum() == 0:
                return np.nan
            return np.mean(np.abs(
                strategy_actions[mask, start:end] - gt_actions[mask, start:end]
            ))

        results[name] = {
            'mae_total': _mae(valid),
            'mae_active': _mae(valid_active),
            'mae_dead': _mae(valid_dead),
            'n_active': int(valid_active.sum()),
            'n_dead': int(valid_dead.sum()),
        }
    return results


def compute_discontinuity(actions, boundaries):
    """Action jump magnitude at each chunk boundary."""
    jumps = []
    for start, obs, skip in boundaries:
        if 0 < start < len(actions):
            prev = actions[start - 1]
            curr = actions[start]
            if not np.isnan(prev).any() and not np.isnan(curr).any():
                jumps.append(np.abs(curr - prev))
    return np.array(jumps) if jumps else np.zeros((0, 22))


def integrate_base_velocity(actions, dt=ACTION_STEP_MS / 1000):
    """Integrate cmd_vel_lx to get cumulative forward displacement."""
    vel = actions[:, 0].copy()
    vel[np.isnan(vel)] = 0.0
    return np.cumsum(vel) * dt


# ── Plotting ──────────────────────────────────────────────────────────

COLORS = {'sequential': '#1f77b4', 'overlapped': '#d62728', 'oracle': '#2ca02c',
          'gt': '#333333'}


def plot_trajectory_comparison(seq_actions, ovl_actions, orc_actions,
                               gt_actions, seq_dead, ovl_dead,
                               seq_bounds, ovl_bounds, save_dir):
    """Multi-panel trajectory plot for key joints."""
    n = len(KEY_JOINTS) + 1  # +1 for integrated position
    fig, axes = plt.subplots(n, 1, figsize=(16, 2.8 * n), sharex=True)
    time_s = np.arange(len(gt_actions)) * ACTION_STEP_MS / 1000

    for i, (idx, name, title) in enumerate(KEY_JOINTS):
        ax = axes[i]

        # GT
        ax.plot(time_s, gt_actions[:, idx], color=COLORS['gt'],
                alpha=0.35, linewidth=1, label='GT (demo)')

        # Oracle
        valid = ~np.isnan(orc_actions[:, idx])
        ax.plot(time_s[valid], orc_actions[valid, idx], color=COLORS['oracle'],
                alpha=0.4, linewidth=1, label='Oracle (0ms)')

        # Sequential
        valid = ~np.isnan(seq_actions[:, idx])
        ax.plot(time_s[valid], seq_actions[valid, idx],
                color=COLORS['sequential'], alpha=0.7, linewidth=1.2,
                label='Sequential')

        # Shade sequential dead time
        dead_starts = []
        in_dead = False
        for f in range(len(seq_dead)):
            if seq_dead[f] and not in_dead:
                dead_starts.append(f)
                in_dead = True
            elif not seq_dead[f] and in_dead:
                ax.axvspan(time_s[dead_starts[-1]], time_s[f],
                           color=COLORS['sequential'], alpha=0.08)
                in_dead = False
        if in_dead:
            ax.axvspan(time_s[dead_starts[-1]], time_s[-1],
                       color=COLORS['sequential'], alpha=0.08)

        # Overlapped
        valid = ~np.isnan(ovl_actions[:, idx])
        ax.plot(time_s[valid], ovl_actions[valid, idx],
                color=COLORS['overlapped'], alpha=0.7, linewidth=1.2,
                label='Overlapped')

        ax.set_ylabel(name, fontsize=8)
        ax.set_title(title, fontsize=9, loc='left')
        ax.grid(True, alpha=0.2)
        if i == 0:
            ax.legend(loc='upper right', fontsize=7, ncol=4)

    # Integrated base position
    ax = axes[-1]
    gt_pos = integrate_base_velocity(gt_actions)
    seq_pos = integrate_base_velocity(seq_actions)
    ovl_pos = integrate_base_velocity(ovl_actions)
    orc_pos = integrate_base_velocity(orc_actions)

    ax.plot(time_s, gt_pos, color=COLORS['gt'], alpha=0.35, linewidth=1,
            label='GT')
    ax.plot(time_s, orc_pos, color=COLORS['oracle'], alpha=0.4, linewidth=1,
            label='Oracle')
    ax.plot(time_s, seq_pos, color=COLORS['sequential'], alpha=0.7,
            linewidth=1.2, label='Sequential')
    ax.plot(time_s, ovl_pos, color=COLORS['overlapped'], alpha=0.7,
            linewidth=1.2, label='Overlapped')
    ax.set_ylabel('Integrated position', fontsize=8)
    ax.set_title('Cumulative Forward Displacement (∫ cmd_vel_lx dt)',
                 fontsize=9, loc='left')
    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True, alpha=0.2)
    ax.set_xlabel('Time (s)')

    plt.tight_layout()
    path = save_dir / 'trajectory_comparison.png'
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


def plot_staleness(seq_obs, ovl_obs, num_frames, save_dir):
    """Observation staleness over time."""
    fig, ax = plt.subplots(figsize=(14, 3.5))
    frames = np.arange(num_frames)
    time_s = frames * ACTION_STEP_MS / 1000

    seq_stale = (frames - seq_obs) * ACTION_STEP_MS
    ovl_stale = (frames - ovl_obs) * ACTION_STEP_MS

    ax.fill_between(time_s, seq_stale, alpha=0.3, color=COLORS['sequential'],
                    label='Sequential')
    ax.fill_between(time_s, ovl_stale, alpha=0.3, color=COLORS['overlapped'],
                    label='Overlapped')
    ax.plot(time_s, seq_stale, color=COLORS['sequential'], alpha=0.7, linewidth=1)
    ax.plot(time_s, ovl_stale, color=COLORS['overlapped'], alpha=0.7, linewidth=1)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Observation Staleness (ms)')
    ax.set_title('Observation Staleness Over Time')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    path = save_dir / 'staleness_profile.png'
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


def plot_discontinuity(seq_jumps, ovl_jumps, save_dir):
    """Box plot of chunk boundary discontinuities per body part."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), sharey=True)

    for ax, jumps, title, color in [
        (axes[0], seq_jumps, 'Sequential', COLORS['sequential']),
        (axes[1], ovl_jumps, 'Overlapped', COLORS['overlapped']),
    ]:
        if len(jumps) == 0:
            ax.set_title(f'{title} (no boundaries)')
            continue

        data = []
        labels = []
        for name, start, end in BODY_PART_GROUPS:
            part_jumps = jumps[:, start:end].ravel()
            data.append(part_jumps)
            labels.append(name)

        bp = ax.boxplot(data, labels=labels, patch_artist=True)
        for patch in bp['boxes']:
            patch.set_facecolor(color)
            patch.set_alpha(0.5)
        ax.set_title(f'{title} Chunk Boundary Jumps')
        ax.set_ylabel('|action[t] - action[t-1]|')
        ax.tick_params(axis='x', rotation=30)
        ax.grid(True, alpha=0.2, axis='y')

    plt.tight_layout()
    path = save_dir / 'chunk_discontinuity.png'
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


# ── Summary ───────────────────────────────────────────────────────────

def print_summary(seq_metrics, ovl_metrics, orc_metrics,
                  seq_dead, ovl_dead, seq_bounds, ovl_bounds,
                  num_frames, latencies):
    """Print comparison summary table."""
    print()
    print('=' * 78)
    print('  OVERLAPPED EXECUTION STRATEGY TEST — SUMMARY')
    print('=' * 78)

    # Timing
    seq_dead_frac = seq_dead.sum() / num_frames * 100
    ovl_dead_frac = ovl_dead.sum() / num_frames * 100
    mean_lat = np.mean(latencies) if latencies else 0

    print()
    print(f'  {"Metric":<30} {"Sequential":>14} {"Overlapped":>14}')
    print(f'  {"-"*30} {"-"*14} {"-"*14}')
    print(f'  {"Chunks":<30} {len(seq_bounds):>14} {len(ovl_bounds):>14}')
    print(f'  {"Cycle (frames)":<30} {16:>14} {8:>14}')
    print(f'  {"Cycle (ms)":<30} {16*ACTION_STEP_MS:>14} {8*ACTION_STEP_MS:>14}')
    print(f'  {"Dead time frames":<30} {int(seq_dead.sum()):>14} {int(ovl_dead.sum()):>14}')
    print(f'  {"Dead time %":<30} {seq_dead_frac:>13.1f}% {ovl_dead_frac:>13.1f}%')
    print(f'  {"Re-plan rate (Hz)":<30} '
          f'{len(seq_bounds)/(num_frames*ACTION_STEP_MS/1000):>14.2f} '
          f'{len(ovl_bounds)/(num_frames*ACTION_STEP_MS/1000):>14.2f}')
    print(f'  {"Mean inference latency (ms)":<30} {mean_lat:>14.0f}')
    print()

    # MAE vs GT per body part
    print(f'  MAE vs Ground Truth (active frames only):')
    print(f'  {"Body Part":<14} {"Sequential":>12} {"Overlapped":>12} '
          f'{"Oracle":>12} {"Winner":>10}')
    print(f'  {"-"*14} {"-"*12} {"-"*12} {"-"*12} {"-"*10}')

    for name, _, _ in BODY_PART_GROUPS:
        s = seq_metrics[name]['mae_active']
        o = ovl_metrics[name]['mae_active']
        r = orc_metrics[name]['mae_active']

        if np.isnan(s) and np.isnan(o):
            winner = '—'
        elif np.isnan(s):
            winner = 'Overlapped'
        elif np.isnan(o):
            winner = 'Sequential'
        elif o < s:
            winner = 'Overlapped'
        elif s < o:
            winner = 'Sequential'
        else:
            winner = 'Tie'

        print(f'  {name:<14} {s:>12.5f} {o:>12.5f} {r:>12.5f} {winner:>10}')

    # MAE including dead time
    print()
    print(f'  MAE vs Ground Truth (ALL frames, dead time penalized):')
    print(f'  {"Body Part":<14} {"Sequential":>12} {"Overlapped":>12} {"Δ":>8}')
    print(f'  {"-"*14} {"-"*12} {"-"*12} {"-"*8}')

    for name, _, _ in BODY_PART_GROUPS:
        s = seq_metrics[name]['mae_total']
        o = ovl_metrics[name]['mae_total']
        delta = s - o if not (np.isnan(s) or np.isnan(o)) else np.nan
        sign = '+' if delta > 0 else ''
        print(f'  {name:<14} {s:>12.5f} {o:>12.5f} {sign}{delta:>7.5f}')

    print()
    print('=' * 78)
    print()


def save_per_frame_csv(seq_actions, ovl_actions, orc_actions, gt_actions,
                       seq_obs, ovl_obs, seq_dead, ovl_dead, save_dir,
                       episode_index=None):
    """Save per-frame data for offline analysis."""
    num_frames = len(gt_actions)
    rows = []
    for t in range(num_frames):
        row = {'frame': t, 'time_ms': t * ACTION_STEP_MS}
        if episode_index is not None:
            row['episode'] = episode_index
        for j, jname in enumerate(JOINT_NAMES):
            row[f'gt_{jname}'] = gt_actions[t, j]
            row[f'seq_{jname}'] = seq_actions[t, j]
            row[f'ovl_{jname}'] = ovl_actions[t, j]
            row[f'orc_{jname}'] = orc_actions[t, j]
        row['seq_obs_frame'] = seq_obs[t]
        row['ovl_obs_frame'] = ovl_obs[t]
        row['seq_staleness_ms'] = (t - seq_obs[t]) * ACTION_STEP_MS
        row['ovl_staleness_ms'] = (t - ovl_obs[t]) * ACTION_STEP_MS
        row['seq_is_dead'] = bool(seq_dead[t])
        row['ovl_is_dead'] = bool(ovl_dead[t])
        rows.append(row)

    df = pd.DataFrame(rows)
    suffix = f'_ep{episode_index}' if episode_index is not None else ''
    path = save_dir / f'per_frame{suffix}.csv'
    df.to_csv(path, index=False)
    logger.info(f'Saved: {path}')


# ── Main ──────────────────────────────────────────────────────────────

def process_episode(args, episode_index, client, inference_delay, save_dir):
    """Process a single episode: collect horizons, simulate, compute metrics.

    Returns dict with all per-episode results, or None on failure.
    """
    states, gt_actions = load_episode(args.dataset_path, episode_index)
    num_frames = len(states)

    # Collect or load horizons
    cache_path = save_dir / f'horizons_ep{episode_index}.npz'
    horizons = None

    if args.load_horizons and cache_path.exists():
        horizons, latencies = load_horizons(cache_path)
        if len(horizons) != num_frames:
            logger.warning(
                f'Ep {episode_index}: cached {len(horizons)} != '
                f'{num_frames} frames. Re-collecting.'
            )
            horizons = None

    if horizons is None:
        logger.info(f'Ep {episode_index}: loading {num_frames} video frames...')
        video_frames = load_video_frames(
            args.dataset_path, episode_index, num_frames,
        )
        est_time = num_frames * args.latency_ms / 1000
        logger.info(
            f'Ep {episode_index}: collecting {num_frames} horizons '
            f'(~{est_time:.0f}s)...'
        )
        horizons, latencies = collect_horizons(
            client, video_frames, states, args.task,
        )
        save_horizons(horizons, latencies, cache_path)
        del video_frames

    valid = sum(1 for h in horizons if h is not None)
    logger.info(f'Ep {episode_index}: {valid}/{num_frames} valid horizons')

    # Simulate
    seq_actions, seq_obs, seq_dead, seq_bounds = simulate_sequential(
        horizons, num_frames, skip=args.skip,
        inference_delay=inference_delay,
    )
    ovl_actions, ovl_obs, ovl_dead, ovl_bounds = simulate_overlapped(
        horizons, num_frames, first_skip=args.first_skip,
        subsequent_skip=args.subsequent_skip, n_exec=args.n_exec,
        trigger_at=args.trigger_at, inference_delay=inference_delay,
    )
    orc_actions = oracle_actions(horizons, num_frames)
    orc_dead = np.zeros(num_frames, dtype=bool)

    # Per-episode metrics
    seq_metrics = compute_mae_table(seq_actions, gt_actions, seq_dead)
    ovl_metrics = compute_mae_table(ovl_actions, gt_actions, ovl_dead)
    orc_metrics = compute_mae_table(orc_actions, gt_actions, orc_dead)
    seq_jumps = compute_discontinuity(seq_actions, seq_bounds)
    ovl_jumps = compute_discontinuity(ovl_actions, ovl_bounds)

    # Save per-frame CSV
    save_per_frame_csv(
        seq_actions, ovl_actions, orc_actions, gt_actions,
        seq_obs, ovl_obs, seq_dead, ovl_dead, save_dir,
        episode_index=episode_index,
    )

    return {
        'episode': episode_index,
        'num_frames': num_frames,
        'latencies': latencies,
        'gt_actions': gt_actions,
        'seq_actions': seq_actions, 'seq_obs': seq_obs,
        'seq_dead': seq_dead, 'seq_bounds': seq_bounds,
        'ovl_actions': ovl_actions, 'ovl_obs': ovl_obs,
        'ovl_dead': ovl_dead, 'ovl_bounds': ovl_bounds,
        'orc_actions': orc_actions, 'orc_dead': orc_dead,
        'seq_metrics': seq_metrics, 'ovl_metrics': ovl_metrics,
        'orc_metrics': orc_metrics,
        'seq_jumps': seq_jumps, 'ovl_jumps': ovl_jumps,
    }


def aggregate_metrics(results_list):
    """Aggregate per-body-part MAE across episodes (weighted by frame count)."""
    agg = {}
    for strategy in ['seq', 'ovl', 'orc']:
        key = f'{strategy}_metrics'
        agg[key] = {}
        for name, start, end in BODY_PART_GROUPS:
            total_abs_err = 0.0
            total_abs_err_active = 0.0
            total_abs_err_dead = 0.0
            n_total = 0
            n_active = 0
            n_dead = 0
            for r in results_list:
                m = r[key][name]
                na = m['n_active']
                nd = m['n_dead']
                nt = na + nd
                if not np.isnan(m['mae_total']) and nt > 0:
                    total_abs_err += m['mae_total'] * nt
                    n_total += nt
                if not np.isnan(m['mae_active']) and na > 0:
                    total_abs_err_active += m['mae_active'] * na
                    n_active += na
                if not np.isnan(m['mae_dead']) and nd > 0:
                    total_abs_err_dead += m['mae_dead'] * nd
                    n_dead += nd
            agg[key][name] = {
                'mae_total': total_abs_err / n_total if n_total > 0 else np.nan,
                'mae_active': total_abs_err_active / n_active if n_active > 0 else np.nan,
                'mae_dead': total_abs_err_dead / n_dead if n_dead > 0 else np.nan,
                'n_active': n_active,
                'n_dead': n_dead,
            }
    return agg


def run_test(args):
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    inference_delay = round(args.latency_ms / ACTION_STEP_MS)

    print()
    print('=' * 65)
    print('  Overlapped Execution Strategy Test')
    print('=' * 65)
    print(f'  Dataset:          {args.dataset_path}')
    print(f'  Episodes:         {args.episode_indices}')
    print(f'  Latency:          {args.latency_ms}ms '
          f'({inference_delay} frames)')
    print(f'  Sequential:       skip={args.skip}, n_steps=16, '
          f'cycle={16}f ({16*ACTION_STEP_MS}ms)')
    print(f'  Overlapped:       first_skip={args.first_skip}, '
          f'subsequent_skip={args.subsequent_skip}, '
          f'n_exec={args.n_exec}, trigger@step{args.trigger_at}')
    print(f'  Timing margin:    '
          f'{args.n_exec - args.trigger_at - inference_delay} frames '
          f'({(args.n_exec - args.trigger_at - inference_delay) * ACTION_STEP_MS}ms)')
    print(f'  Save dir:         {save_dir}')
    print('=' * 65)
    print()

    # Timing feasibility check
    margin = args.n_exec - args.trigger_at - inference_delay
    if margin < 0:
        logger.error(
            f'Timing infeasible: trigger@{args.trigger_at} + '
            f'{inference_delay}f delay = {args.trigger_at + inference_delay}f '
            f'> n_exec={args.n_exec}. Inference won\'t finish in time.'
        )
        sys.exit(1)
    if margin == 0:
        logger.warning('Zero timing margin — inference must finish exactly on time.')

    # Connect to server (shared across episodes)
    client = None
    need_server = not args.load_horizons or any(
        not (save_dir / f'horizons_ep{ep}.npz').exists()
        for ep in args.episode_indices
    )
    if need_server:
        server_address = build_server_address(
            transport=args.transport, host=args.host, port=args.port,
        )
        client = ZMQClient(
            server_address=server_address,
            timeout_ms=args.timeout_ms,
            logger=lambda msg: logger.info(msg),
        )
        if not client.connect():
            logger.error(f'Failed to connect to {server_address}')
            sys.exit(1)
        if not client.ping(timeout_ms=5000):
            logger.error(f'Server not responding at {server_address}')
            client.close()
            sys.exit(1)
        logger.info(f'Connected to {server_address}')

    # Process each episode
    results_list = []
    for ep_idx in args.episode_indices:
        logger.info(f'\n{"="*50} Episode {ep_idx} {"="*50}')
        result = process_episode(
            args, ep_idx, client, inference_delay, save_dir,
        )
        if result is not None:
            results_list.append(result)

    if client is not None:
        client.close()

    if not results_list:
        logger.error('No episodes processed successfully')
        sys.exit(1)

    # Aggregate metrics across episodes
    agg = aggregate_metrics(results_list)
    all_latencies = []
    total_frames = 0
    total_seq_dead = 0
    total_ovl_dead = 0
    total_seq_bounds = 0
    total_ovl_bounds = 0
    all_seq_jumps = []
    all_ovl_jumps = []

    for r in results_list:
        all_latencies.extend(r['latencies'])
        total_frames += r['num_frames']
        total_seq_dead += int(r['seq_dead'].sum())
        total_ovl_dead += int(r['ovl_dead'].sum())
        total_seq_bounds += len(r['seq_bounds'])
        total_ovl_bounds += len(r['ovl_bounds'])
        if len(r['seq_jumps']) > 0:
            all_seq_jumps.append(r['seq_jumps'])
        if len(r['ovl_jumps']) > 0:
            all_ovl_jumps.append(r['ovl_jumps'])

    seq_jumps_all = (np.concatenate(all_seq_jumps) if all_seq_jumps
                     else np.zeros((0, 22)))
    ovl_jumps_all = (np.concatenate(all_ovl_jumps) if all_ovl_jumps
                     else np.zeros((0, 22)))

    # Print aggregate summary
    print()
    print('=' * 78)
    print(f'  OVERLAPPED EXECUTION STRATEGY TEST — AGGREGATE SUMMARY')
    print(f'  ({len(results_list)} episodes: '
          f'{[r["episode"] for r in results_list]})')
    print('=' * 78)

    seq_dead_frac = total_seq_dead / total_frames * 100
    ovl_dead_frac = total_ovl_dead / total_frames * 100
    mean_lat = np.mean(all_latencies) if all_latencies else 0
    duration_s = total_frames * ACTION_STEP_MS / 1000

    print()
    print(f'  {"Metric":<30} {"Sequential":>14} {"Overlapped":>14}')
    print(f'  {"-"*30} {"-"*14} {"-"*14}')
    print(f'  {"Total frames":<30} {total_frames:>14}')
    print(f'  {"Total duration":<30} {duration_s:>13.1f}s')
    print(f'  {"Total chunks":<30} {total_seq_bounds:>14} {total_ovl_bounds:>14}')
    print(f'  {"Cycle (ms)":<30} {16*ACTION_STEP_MS:>14} {8*ACTION_STEP_MS:>14}')
    print(f'  {"Dead time frames":<30} {total_seq_dead:>14} {total_ovl_dead:>14}')
    print(f'  {"Dead time %":<30} {seq_dead_frac:>13.1f}% {ovl_dead_frac:>13.1f}%')
    print(f'  {"Re-plan rate (Hz)":<30} '
          f'{total_seq_bounds/duration_s:>14.2f} '
          f'{total_ovl_bounds/duration_s:>14.2f}')
    print(f'  {"Mean inference latency (ms)":<30} {mean_lat:>14.0f}')
    print()

    # Aggregate MAE tables
    print(f'  MAE vs Ground Truth (active frames only):')
    print(f'  {"Body Part":<14} {"Sequential":>12} {"Overlapped":>12} '
          f'{"Oracle":>12} {"Winner":>10}')
    print(f'  {"-"*14} {"-"*12} {"-"*12} {"-"*12} {"-"*10}')

    for name, _, _ in BODY_PART_GROUPS:
        s = agg['seq_metrics'][name]['mae_active']
        o = agg['ovl_metrics'][name]['mae_active']
        r = agg['orc_metrics'][name]['mae_active']
        if np.isnan(s) and np.isnan(o):
            winner = '—'
        elif np.isnan(s):
            winner = 'Overlapped'
        elif np.isnan(o):
            winner = 'Sequential'
        elif o < s:
            winner = 'Overlapped'
        elif s < o:
            winner = 'Sequential'
        else:
            winner = 'Tie'
        print(f'  {name:<14} {s:>12.5f} {o:>12.5f} {r:>12.5f} {winner:>10}')

    print()
    print(f'  MAE vs Ground Truth (ALL frames, dead time penalized):')
    print(f'  {"Body Part":<14} {"Sequential":>12} {"Overlapped":>12} '
          f'{"Δ":>8} {"% change":>10}')
    print(f'  {"-"*14} {"-"*12} {"-"*12} {"-"*8} {"-"*10}')

    for name, _, _ in BODY_PART_GROUPS:
        s = agg['seq_metrics'][name]['mae_total']
        o = agg['ovl_metrics'][name]['mae_total']
        delta = s - o if not (np.isnan(s) or np.isnan(o)) else np.nan
        pct = (delta / s * 100) if (not np.isnan(delta) and s > 1e-8) else np.nan
        sign = '+' if delta > 0 else ''
        pct_str = f'{sign}{pct:.1f}%' if not np.isnan(pct) else '—'
        print(f'  {name:<14} {s:>12.5f} {o:>12.5f} '
              f'{sign}{delta:>7.5f} {pct_str:>10}')

    # Discontinuity summary
    if len(seq_jumps_all) > 0 and len(ovl_jumps_all) > 0:
        print()
        print(f'  Chunk boundary discontinuity (mean |Δaction|):')
        print(f'  {"Body Part":<14} {"Sequential":>12} {"Overlapped":>12} '
              f'{"Reduction":>10}')
        print(f'  {"-"*14} {"-"*12} {"-"*12} {"-"*10}')
        for name, start, end in BODY_PART_GROUPS:
            s = np.mean(seq_jumps_all[:, start:end])
            o = np.mean(ovl_jumps_all[:, start:end])
            red = (1 - o / s) * 100 if s > 1e-8 else 0
            print(f'  {name:<14} {s:>12.5f} {o:>12.5f} {red:>9.0f}%')

    print()
    print('=' * 78)

    # Per-episode breakdown
    print()
    print(f'  Per-episode right_arm MAE (total):')
    print(f'  {"Episode":<10} {"Frames":>8} {"Sequential":>12} '
          f'{"Overlapped":>12} {"Winner":>10}')
    print(f'  {"-"*10} {"-"*8} {"-"*12} {"-"*12} {"-"*10}')
    for r in results_list:
        s = r['seq_metrics']['right_arm']['mae_total']
        o = r['ovl_metrics']['right_arm']['mae_total']
        w = 'Overlapped' if o < s else 'Sequential'
        print(f'  {r["episode"]:<10} {r["num_frames"]:>8} '
              f'{s:>12.5f} {o:>12.5f} {w:>10}')
    print()

    # Plots — use first episode for trajectory plots
    logger.info('Generating plots...')
    r0 = results_list[0]
    plot_trajectory_comparison(
        r0['seq_actions'], r0['ovl_actions'], r0['orc_actions'],
        r0['gt_actions'], r0['seq_dead'], r0['ovl_dead'],
        r0['seq_bounds'], r0['ovl_bounds'], save_dir,
    )
    plot_staleness(r0['seq_obs'], r0['ovl_obs'], r0['num_frames'], save_dir)
    plot_discontinuity(seq_jumps_all, ovl_jumps_all, save_dir)

    # Save aggregate metrics CSV
    agg_rows = []
    for name, start, end in BODY_PART_GROUPS:
        seq_disc = (np.mean(seq_jumps_all[:, start:end])
                    if len(seq_jumps_all) > 0 else np.nan)
        ovl_disc = (np.mean(ovl_jumps_all[:, start:end])
                    if len(ovl_jumps_all) > 0 else np.nan)
        agg_rows.append({
            'body_part': name,
            'seq_mae_total': agg['seq_metrics'][name]['mae_total'],
            'seq_mae_active': agg['seq_metrics'][name]['mae_active'],
            'ovl_mae_total': agg['ovl_metrics'][name]['mae_total'],
            'ovl_mae_active': agg['ovl_metrics'][name]['mae_active'],
            'orc_mae_total': agg['orc_metrics'][name]['mae_total'],
            'seq_disc_mean': seq_disc,
            'ovl_disc_mean': ovl_disc,
        })
    pd.DataFrame(agg_rows).to_csv(save_dir / 'aggregate_metrics.csv', index=False)
    logger.info(f'Saved: {save_dir / "aggregate_metrics.csv"}')

    print(f'  All outputs saved to: {save_dir}/')
    print()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Test overlapped vs sequential execution strategies',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # Single episode
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-index 320 --host 192.168.50.201

  # Multiple episodes for statistical robustness
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-indices 317,318,319,320,321,322 --host 192.168.50.201

  # Reuse cached horizons from previous run
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-indices 317,318,319,320,321,322 --load-horizons
""",
    )
    parser.add_argument('--dataset-path', required=True)
    parser.add_argument('--episode-index', type=int, default=None,
                        help='Single episode (shorthand for --episode-indices)')
    parser.add_argument('--episode-indices', type=str, default=None,
                        help='Comma-separated episode indices')
    parser.add_argument('--transport', choices=['tcp', 'ipc'], default='tcp')
    parser.add_argument('--host', default='192.168.50.201')
    parser.add_argument('--port', '-p', type=int, default=5555)
    parser.add_argument('--timeout-ms', type=int, default=5000)
    parser.add_argument('--task', default='find the can and pick it up')
    parser.add_argument('--latency-ms', type=float, default=280,
                        help='Expected round-trip inference latency')
    # Sequential params
    parser.add_argument('--skip', type=int, default=4,
                        help='Universal latency skip for sequential strategy')
    # Overlapped params
    parser.add_argument('--first-skip', type=int, default=4,
                        help='Skip for first overlapped chunk')
    parser.add_argument('--subsequent-skip', type=int, default=5,
                        help='Skip for subsequent overlapped chunks')
    parser.add_argument('--n-exec', type=int, default=8,
                        help='Actions executed per overlapped chunk')
    parser.add_argument('--trigger-at', type=int, default=3,
                        help='Window step index to trigger mid-chunk inference')
    # Cache
    parser.add_argument('--load-horizons', action='store_true',
                        help='Load cached horizons instead of re-collecting')
    parser.add_argument('--save-dir', default='/tmp/overlapped_test/')
    parser.add_argument('--verbose', '-v', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()

    # Resolve episode indices
    if args.episode_indices:
        args.episode_indices = [int(x.strip()) for x in args.episode_indices.split(',')]
    elif args.episode_index is not None:
        args.episode_indices = [args.episode_index]
    else:
        args.episode_indices = [3]

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )
    run_test(args)


if __name__ == '__main__':
    main()
