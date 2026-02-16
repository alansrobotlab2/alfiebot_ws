#!/usr/bin/env python3
"""Action Horizon Temporal Consistency Analysis.

Measures how well the model's k-step-ahead predictions agree with fresh
inference at that future timestep.  Formally, for each offset k=1..15,
compares H[T][k] vs H[T+k][0] across all valid T.

Runs inference at every frame of a recorded episode to collect overlapping
16-step horizons, then computes per-body-part correlation / MAE / RMSE
decay curves as a function of horizon offset.

No ROS2 dependency — runs standalone against the ZMQ inference server.

Usage:
    python action_horizon_analysis.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 3 \
        --host 192.168.50.201 --port 5555 \
        --save-dir /tmp/horizon_analysis/
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
from scipy import stats as sp_stats

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

# ── Constants (match groot_open_loop_eval.py) ──────────────────────────

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
    ('right_hand',18, 19),
    ('head',      19, 22),
]

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
JPEG_QUALITY = 80
ACTION_STEP_MS = 67  # 15 FPS → 67ms per step


# ── Data loading (from groot_open_loop_eval.py) ───────────────────────

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
    logger.info(f'Loaded episode {episode_index}: {len(df)} frames from {parquet_path}')
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
                str(video_path), device="cpu", dimension_order="NHWC", num_ffmpeg_threads=0
            )
            frames_tensor = decoder.get_frames_at(indices=list(range(num_frames))).data
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

        if len(cam_frames) < num_frames:
            logger.warning(f'{cam_name}: got {len(cam_frames)} frames, expected {num_frames}')
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


# ── Horizon collection ─────────────────────────────────────────────────

def collect_horizons(
    client: ZMQClient,
    video_frames: list[dict],
    states: np.ndarray,
    task: str,
    step_interval: int = 1,
) -> tuple[list[np.ndarray], list[float]]:
    """Run inference at every `step_interval` frames, return horizons and latencies.

    Returns:
        horizons: list of (16, 22) arrays — one per inference call
        latencies: list of round-trip times in ms
    """
    num_frames = len(states)
    horizons = []
    latencies = []
    frame_indices = list(range(0, num_frames, step_interval))

    for count, t in enumerate(frame_indices):
        # Compress images
        images = {}
        frame_dict = video_frames[t] if t < len(video_frames) else {}
        for cam_name, img in frame_dict.items():
            images[cam_name] = compress_jpeg(img, quality=JPEG_QUALITY)

        t0 = time.monotonic()
        response = client.send_observation(
            images=images,
            state=states[t],
            language=task,
        )
        latency_ms = (time.monotonic() - t0) * 1000
        latencies.append(latency_ms)

        if response is None or response.get('status') != 'ok':
            logger.warning(f'Inference failed at frame {t}')
            horizons.append(None)
            continue

        actions_list = response.get('actions', [])
        if not actions_list:
            logger.warning(f'Empty actions at frame {t}')
            horizons.append(None)
            continue

        horizon = np.array(actions_list, dtype=np.float32)  # (16, 22)
        horizons.append(horizon)

        if (count + 1) % 50 == 0 or count == 0:
            logger.info(
                f'  [{count+1}/{len(frame_indices)}] frame={t}  '
                f'latency={latency_ms:.0f}ms  '
                f'mean={np.mean(latencies):.0f}ms'
            )

    return horizons, latencies


# ── Consistency analysis ───────────────────────────────────────────────

def compute_consistency(
    horizons: list[np.ndarray | None],
    step_interval: int = 1,
    max_k: int = 15,
) -> dict:
    """Compare H[T][k] vs H[T+k][0] for k=1..max_k.

    Returns dict mapping body_part_name -> {
        'k_values': [1..max_k],
        'correlation': [...],
        'mae': [...],
        'rmse': [...],
        'n_pairs': [...],
    }
    Also includes 'per_joint' with per-joint-dimension breakdown.
    """
    results = {}

    for group_name, start, end in BODY_PART_GROUPS:
        correlations = []
        maes = []
        rmses = []
        n_pairs_list = []

        for k in range(1, max_k + 1):
            predicted_all = []
            actual_all = []

            # step_interval frames between consecutive horizons
            # k horizon steps = k * step_interval frames apart in the horizon list
            k_idx = k  # k steps ahead in the horizon = k entries apart when step_interval=1

            for t_idx in range(len(horizons) - k_idx):
                h_t = horizons[t_idx]
                h_tk = horizons[t_idx + k_idx]
                if h_t is None or h_tk is None:
                    continue
                if k >= h_t.shape[0]:
                    continue

                predicted_all.append(h_t[k, start:end])
                actual_all.append(h_tk[0, start:end])

            if len(predicted_all) < 5:
                correlations.append(np.nan)
                maes.append(np.nan)
                rmses.append(np.nan)
                n_pairs_list.append(len(predicted_all))
                continue

            predicted = np.array(predicted_all)  # (N, D)
            actual = np.array(actual_all)         # (N, D)

            # Flatten for correlation: (N*D,)
            p_flat = predicted.ravel()
            a_flat = actual.ravel()

            # Pearson correlation
            if np.std(p_flat) < 1e-10 or np.std(a_flat) < 1e-10:
                r = np.nan
            else:
                r, _ = sp_stats.pearsonr(p_flat, a_flat)

            mae = np.mean(np.abs(predicted - actual))
            rmse = np.sqrt(np.mean((predicted - actual) ** 2))

            correlations.append(r)
            maes.append(mae)
            rmses.append(rmse)
            n_pairs_list.append(len(predicted_all))

        results[group_name] = {
            'k_values': list(range(1, max_k + 1)),
            'correlation': correlations,
            'mae': maes,
            'rmse': rmses,
            'n_pairs': n_pairs_list,
        }

    # Per-joint breakdown at each k
    per_joint = {}
    for j, jname in enumerate(JOINT_NAMES):
        correlations = []
        maes = []
        for k in range(1, max_k + 1):
            predicted_all = []
            actual_all = []
            for t_idx in range(len(horizons) - k):
                h_t = horizons[t_idx]
                h_tk = horizons[t_idx + k]
                if h_t is None or h_tk is None:
                    continue
                if k >= h_t.shape[0]:
                    continue
                predicted_all.append(h_t[k, j])
                actual_all.append(h_tk[0, j])

            if len(predicted_all) < 5:
                correlations.append(np.nan)
                maes.append(np.nan)
                continue

            p = np.array(predicted_all)
            a = np.array(actual_all)
            if np.std(p) < 1e-10 or np.std(a) < 1e-10:
                r = np.nan
            else:
                r, _ = sp_stats.pearsonr(p, a)
            correlations.append(r)
            maes.append(np.mean(np.abs(p - a)))

        per_joint[jname] = {
            'correlation': correlations,
            'mae': maes,
        }

    results['_per_joint'] = per_joint
    return results


# ── Plotting ───────────────────────────────────────────────────────────

GROUP_COLORS = {
    'base':       '#1f77b4',
    'back':       '#ff7f0e',
    'left_arm':   '#2ca02c',
    'left_hand':  '#d62728',
    'right_arm':  '#9467bd',
    'right_hand': '#8c564b',
    'head':       '#e377c2',
}


def plot_decay_curves(metrics: dict, save_dir: Path, latency_ms: float = 290):
    """Plot correlation and MAE vs horizon step k, one line per body part."""
    fig, (ax_corr, ax_mae) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    latency_k = latency_ms / ACTION_STEP_MS

    for group_name, _, _ in BODY_PART_GROUPS:
        m = metrics[group_name]
        k = m['k_values']
        color = GROUP_COLORS[group_name]

        ax_corr.plot(k, m['correlation'], '-o', color=color, label=group_name,
                     markersize=4, linewidth=1.5)
        ax_mae.plot(k, m['mae'], '-o', color=color, label=group_name,
                    markersize=4, linewidth=1.5)

    # Mark latency line
    ax_corr.axvline(latency_k, color='red', linestyle='--', alpha=0.7,
                    label=f'latency = {latency_ms:.0f}ms ({latency_k:.1f} steps)')
    ax_mae.axvline(latency_k, color='red', linestyle='--', alpha=0.7)

    # Thresholds
    ax_corr.axhline(0.9, color='gray', linestyle=':', alpha=0.5, label='r=0.9 threshold')
    ax_corr.axhline(0.7, color='gray', linestyle=':', alpha=0.3, label='r=0.7 threshold')

    ax_corr.set_ylabel('Pearson Correlation')
    ax_corr.set_title('Action Horizon Temporal Consistency — Correlation Decay')
    ax_corr.legend(loc='lower left', fontsize=8, ncol=2)
    ax_corr.grid(True, alpha=0.3)
    ax_corr.set_ylim(-0.1, 1.05)

    ax_mae.set_ylabel('Mean Absolute Error')
    ax_mae.set_xlabel('Horizon Offset k (steps)')
    ax_mae.set_title('Action Horizon Temporal Consistency — MAE')
    ax_mae.legend(loc='upper left', fontsize=8, ncol=2)
    ax_mae.grid(True, alpha=0.3)

    # Second x-axis for milliseconds
    ax2 = ax_mae.twiny()
    ax2.set_xlim(ax_mae.get_xlim())
    tick_ks = list(range(1, 16))
    ax2.set_xticks(tick_ks)
    ax2.set_xticklabels([f'{k * ACTION_STEP_MS}' for k in tick_ks], fontsize=7)
    ax2.set_xlabel('Offset (ms)', fontsize=9)
    ax2.spines['top'].set_visible(True)

    plt.tight_layout()
    path = save_dir / 'decay_curves.png'
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


def plot_heatmap(metrics: dict, save_dir: Path, latency_ms: float = 290):
    """Heatmap of correlation: rows = body parts, columns = k values."""
    group_names = [name for name, _, _ in BODY_PART_GROUPS]
    max_k = len(metrics[group_names[0]]['k_values'])

    data = np.zeros((len(group_names), max_k))
    for i, name in enumerate(group_names):
        data[i, :] = metrics[name]['correlation']

    fig, ax = plt.subplots(figsize=(14, 4))
    im = ax.imshow(data, aspect='auto', cmap='RdYlGn', vmin=0, vmax=1)

    ax.set_xticks(range(max_k))
    ax.set_xticklabels([f'k={k+1}\n{(k+1)*ACTION_STEP_MS}ms' for k in range(max_k)], fontsize=8)
    ax.set_yticks(range(len(group_names)))
    ax.set_yticklabels(group_names, fontsize=10)

    # Annotate cells
    for i in range(len(group_names)):
        for j in range(max_k):
            val = data[i, j]
            color = 'white' if val < 0.5 else 'black'
            ax.text(j, i, f'{val:.2f}', ha='center', va='center',
                    fontsize=7, color=color, fontweight='bold')

    # Mark latency column
    latency_k_idx = int(round(latency_ms / ACTION_STEP_MS)) - 1
    if 0 <= latency_k_idx < max_k:
        ax.axvline(latency_k_idx, color='red', linewidth=2, linestyle='--', alpha=0.8)
        ax.set_xlabel(f'Horizon Offset k  (red line = {latency_ms:.0f}ms latency)', fontsize=10)

    plt.colorbar(im, ax=ax, label='Pearson r', shrink=0.8)
    ax.set_title('Per-Body-Part Horizon Consistency Heatmap', fontsize=13)

    plt.tight_layout()
    path = save_dir / 'consistency_heatmap.png'
    plt.savefig(path, dpi=120, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


def plot_per_joint_decay(metrics: dict, save_dir: Path, latency_ms: float = 290):
    """One subplot per joint dimension showing correlation decay."""
    per_joint = metrics['_per_joint']
    n_joints = len(JOINT_NAMES)
    fig, axes = plt.subplots(n_joints, 1, figsize=(12, 2.2 * n_joints), sharex=True)

    latency_k = latency_ms / ACTION_STEP_MS

    for idx, jname in enumerate(JOINT_NAMES):
        ax = axes[idx]
        corrs = per_joint[jname]['correlation']
        ks = list(range(1, len(corrs) + 1))

        # Find which body part group this joint belongs to
        color = 'gray'
        for gname, start, end in BODY_PART_GROUPS:
            if start <= idx < end:
                color = GROUP_COLORS[gname]
                break

        ax.plot(ks, corrs, '-o', color=color, markersize=3, linewidth=1.2)
        ax.axvline(latency_k, color='red', linestyle='--', alpha=0.5)
        ax.axhline(0.9, color='gray', linestyle=':', alpha=0.4)
        ax.set_ylabel('r', fontsize=8)
        ax.set_title(f'{jname} (dim {idx})', fontsize=9, color=color, loc='left')
        ax.set_ylim(-0.2, 1.05)
        ax.grid(True, alpha=0.2)

        # Annotate correlation at latency offset
        k_at_latency = int(round(latency_ms / ACTION_STEP_MS))
        if 0 < k_at_latency <= len(corrs):
            r_at_latency = corrs[k_at_latency - 1]
            ax.annotate(f'r={r_at_latency:.3f}',
                        xy=(k_at_latency, r_at_latency),
                        fontsize=7, color='red',
                        xytext=(k_at_latency + 1, r_at_latency),
                        arrowprops=dict(arrowstyle='->', color='red', lw=0.8))

    axes[-1].set_xlabel('Horizon Offset k (steps)')
    fig.suptitle('Per-Joint Horizon Consistency', fontsize=14, y=1.0)

    plt.tight_layout()
    path = save_dir / 'per_joint_decay.png'
    plt.savefig(path, dpi=100, bbox_inches='tight')
    plt.close()
    logger.info(f'Saved: {path}')


# ── Summary printing ───────────────────────────────────────────────────

def print_summary(metrics: dict, latency_ms: float = 290):
    """Print per-body-part summary at the latency-equivalent skip step."""
    k_latency = int(round(latency_ms / ACTION_STEP_MS))

    print()
    print('=' * 75)
    print(f'  ACTION HORIZON TEMPORAL CONSISTENCY SUMMARY')
    print(f'  Latency: {latency_ms:.0f}ms = {latency_ms/ACTION_STEP_MS:.1f} steps  '
          f'(evaluating at k={k_latency})')
    print('=' * 75)
    print()
    print(f'  {"Body Part":<14} {"Corr@k=" + str(k_latency):>10} '
          f'{"MAE@k=" + str(k_latency):>10} {"RMSE@k=" + str(k_latency):>11} '
          f'{"N pairs":>8}  {"Skip viable?"}')
    print(f'  {"-"*14} {"-"*10} {"-"*10} {"-"*11} {"-"*8}  {"-"*14}')

    for group_name, _, _ in BODY_PART_GROUPS:
        m = metrics[group_name]
        idx = k_latency - 1  # k_values is 1-indexed
        if idx >= len(m['correlation']):
            continue

        r = m['correlation'][idx]
        mae = m['mae'][idx]
        rmse = m['rmse'][idx]
        n = m['n_pairs'][idx]

        if np.isnan(r):
            verdict = '?? (no data)'
        elif r >= 0.9:
            verdict = 'YES (r>=0.9)'
        elif r >= 0.7:
            verdict = 'MAYBE (0.7-0.9)'
        else:
            verdict = 'NO (r<0.7)'

        print(f'  {group_name:<14} {r:>10.4f} {mae:>10.5f} {rmse:>11.5f} {n:>8}  {verdict}')

    print()

    # Also show "safe skip" per body part — largest k where r > 0.9
    print(f'  {"Body Part":<14} {"Safe skip (r>0.9)":>18} {"Max skip (r>0.7)":>18}')
    print(f'  {"-"*14} {"-"*18} {"-"*18}')

    for group_name, _, _ in BODY_PART_GROUPS:
        m = metrics[group_name]
        safe_k = 0
        max_k = 0
        for i, r in enumerate(m['correlation']):
            if not np.isnan(r) and r >= 0.9:
                safe_k = i + 1
            if not np.isnan(r) and r >= 0.7:
                max_k = i + 1

        safe_ms = safe_k * ACTION_STEP_MS
        max_ms = max_k * ACTION_STEP_MS
        print(f'  {group_name:<14} {safe_k:>8} ({safe_ms:>4}ms)    {max_k:>8} ({max_ms:>4}ms)')

    print()
    print('=' * 75)
    print()


def save_raw_data(
    horizons: list[np.ndarray | None],
    latencies: list[float],
    metrics: dict,
    save_dir: Path,
):
    """Save raw horizon data and metrics to CSV for offline analysis."""
    # Save metrics summary
    rows = []
    for group_name, start, end in BODY_PART_GROUPS:
        m = metrics[group_name]
        for i, k in enumerate(m['k_values']):
            rows.append({
                'body_part': group_name,
                'k': k,
                'k_ms': k * ACTION_STEP_MS,
                'correlation': m['correlation'][i],
                'mae': m['mae'][i],
                'rmse': m['rmse'][i],
                'n_pairs': m['n_pairs'][i],
            })
    df = pd.DataFrame(rows)
    path = save_dir / 'consistency_metrics.csv'
    df.to_csv(path, index=False)
    logger.info(f'Saved: {path}')

    # Save per-joint metrics
    per_joint = metrics.get('_per_joint', {})
    joint_rows = []
    for jname in JOINT_NAMES:
        if jname not in per_joint:
            continue
        for i, k in enumerate(range(1, len(per_joint[jname]['correlation']) + 1)):
            joint_rows.append({
                'joint': jname,
                'k': k,
                'k_ms': k * ACTION_STEP_MS,
                'correlation': per_joint[jname]['correlation'][i],
                'mae': per_joint[jname]['mae'][i],
            })
    if joint_rows:
        df_j = pd.DataFrame(joint_rows)
        path_j = save_dir / 'per_joint_metrics.csv'
        df_j.to_csv(path_j, index=False)
        logger.info(f'Saved: {path_j}')

    # Save latency log
    df_lat = pd.DataFrame({'frame': range(len(latencies)), 'latency_ms': latencies})
    path_lat = save_dir / 'latencies.csv'
    df_lat.to_csv(path_lat, index=False)
    logger.info(f'Saved: {path_lat}')


# ── Main ───────────────────────────────────────────────────────────────

def run_analysis(args):
    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    server_address = build_server_address(
        transport=args.transport, host=args.host, port=args.port,
    )

    print()
    print('=' * 65)
    print('  Action Horizon Temporal Consistency Analysis')
    print('=' * 65)
    print(f'  Dataset:     {args.dataset_path}')
    print(f'  Episodes:    {args.episode_indices}')
    print(f'  Server:      {server_address}')
    print(f'  Task:        "{args.task}"')
    print(f'  Interval:    every {args.step_interval} frame(s)')
    print(f'  Latency:     {args.latency_ms}ms ({args.latency_ms/ACTION_STEP_MS:.1f} steps)')
    print(f'  Save dir:    {save_dir}')
    print('=' * 65)
    print()

    # Connect
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

    # Collect horizons across all episodes
    all_horizons = []
    all_latencies = []

    for ep_idx in args.episode_indices:
        logger.info(f'\n--- Episode {ep_idx} ---')

        states, actions = load_episode(args.dataset_path, ep_idx)
        num_frames = len(states)
        logger.info(f'Loading {num_frames} video frames...')
        video_frames = load_video_frames(args.dataset_path, ep_idx, num_frames)

        n_inferences = len(range(0, num_frames, args.step_interval))
        est_time = n_inferences * args.latency_ms / 1000
        logger.info(
            f'Running {n_inferences} inferences (every {args.step_interval} frame(s)), '
            f'estimated {est_time:.0f}s'
        )

        horizons, latencies = collect_horizons(
            client, video_frames, states, args.task,
            step_interval=args.step_interval,
        )
        all_horizons.extend(horizons)
        all_latencies.extend(latencies)

        logger.info(
            f'Episode {ep_idx}: {len(horizons)} horizons, '
            f'{sum(1 for h in horizons if h is not None)} valid, '
            f'mean latency {np.mean(latencies):.0f}ms'
        )

    client.close()

    # Filter valid
    valid_count = sum(1 for h in all_horizons if h is not None)
    logger.info(f'\nTotal: {len(all_horizons)} horizons, {valid_count} valid')

    if valid_count < 20:
        logger.error('Not enough valid horizons for analysis (need >= 20)')
        sys.exit(1)

    # Compute consistency
    logger.info('Computing temporal consistency metrics...')
    metrics = compute_consistency(all_horizons, step_interval=args.step_interval)

    # Output
    print_summary(metrics, latency_ms=args.latency_ms)

    logger.info('Generating plots...')
    plot_decay_curves(metrics, save_dir, latency_ms=args.latency_ms)
    plot_heatmap(metrics, save_dir, latency_ms=args.latency_ms)
    plot_per_joint_decay(metrics, save_dir, latency_ms=args.latency_ms)
    save_raw_data(all_horizons, all_latencies, metrics, save_dir)

    print(f'  All outputs saved to: {save_dir}/')
    print()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Measure action horizon temporal consistency via ZMQ inference server',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # Single episode
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-index 3 --host 192.168.50.201

  # Multiple episodes for statistical robustness
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-indices 0,3,5,10 --host 192.168.50.201

  # Every 2nd frame (faster, still good coverage)
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-index 3 --step-interval 2 --host 192.168.50.201
""",
    )
    parser.add_argument('--dataset-path', required=True, help='Path to LeRobot-format dataset')
    parser.add_argument('--episode-index', type=int, default=None,
                        help='Single episode index (shorthand for --episode-indices)')
    parser.add_argument('--episode-indices', type=str, default=None,
                        help='Comma-separated episode indices (e.g., 0,3,5,10)')
    parser.add_argument('--transport', choices=['tcp', 'ipc'], default='tcp')
    parser.add_argument('--host', default='192.168.50.201')
    parser.add_argument('--port', '-p', type=int, default=5555)
    parser.add_argument('--timeout-ms', type=int, default=5000)
    parser.add_argument('--task', default='find the can and pick it up')
    parser.add_argument('--step-interval', type=int, default=1,
                        help='Run inference every N frames (1=every frame, 2=every other)')
    parser.add_argument('--latency-ms', type=float, default=290,
                        help='Expected round-trip latency in ms (for marking plots)')
    parser.add_argument('--save-dir', default='/tmp/horizon_analysis/')
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
        args.episode_indices = [0]

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S',
    )

    run_analysis(args)


if __name__ == '__main__':
    main()
