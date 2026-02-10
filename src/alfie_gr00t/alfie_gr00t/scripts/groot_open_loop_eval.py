#!/usr/bin/env python3
"""Open-loop evaluation through the GR00T ZMQ client→server pipeline.

Reads an episode from a LeRobot-format dataset (parquet + videos),
replays observations through the ZMQ inference server, and compares
predicted actions against ground truth. Produces trajectory plots
and communication diagnostics.

No ROS2 dependency — runs standalone.

Usage:
    python groot_open_loop_eval.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 3 \
        --host 192.168.50.108 --port 5555

    # With custom stats and save path
    python groot_open_loop_eval.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode-index 0 \
        --host 192.168.50.108 --port 5555 \
        --save-plot /tmp/eval_episode_0.png \
        --task "find the can and pick it up"
"""

import argparse
import json
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

# Import ZMQ client (no ROS2 dependency)
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from alfie_gr00t.core.zmq_client import ZMQClient, build_server_address

logger = logging.getLogger(__name__)

# Joint names matching the 22D action/state vector layout
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

# Camera names matching the observation bridge
CAMERA_NAMES = ['left_wide', 'right_wide', 'left_center', 'right_center']

# Target image size for inference (must match ObservationBridge)
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# Body-part groupings matching the modality config
# Maps group name -> (start_index, end_index) in the 22D vector
BODY_PART_GROUPS = [
    ('base',       0,  6),   # cmd_vel linear + angular
    ('back',       6,  7),   # back_joint
    ('left_arm',   7, 12),   # shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
    ('left_hand', 12, 13),   # gripper
    ('right_arm', 13, 18),   # shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll
    ('right_hand',18, 19),   # gripper
    ('head',      19, 22),   # yaw, pitch, roll
]
JPEG_QUALITY = 80


def load_stats(stats_path: str) -> dict:
    """Load normalization statistics from stats.json.

    Handles both 'state'/'observation.state' key conventions.

    Returns:
        Dict with 'state_mean', 'state_std', 'action_mean', 'action_std'.
    """
    with open(stats_path, 'r') as f:
        stats = json.load(f)

    # Handle observation.state vs state key
    state_key = 'observation.state' if 'observation.state' in stats else 'state'
    state_stats = stats.get(state_key, {})
    action_stats = stats.get('action', {})

    result = {
        'state_mean': np.array(state_stats.get('mean', np.zeros(22)), dtype=np.float32),
        'state_std': np.array(state_stats.get('std', np.ones(22)), dtype=np.float32),
        'action_mean': np.array(action_stats.get('mean', np.zeros(22)), dtype=np.float32),
        'action_std': np.array(action_stats.get('std', np.ones(22)), dtype=np.float32),
    }

    # Replace zero std with 1.0
    result['state_std'] = np.where(result['state_std'] == 0, 1.0, result['state_std'])
    result['action_std'] = np.where(result['action_std'] == 0, 1.0, result['action_std'])

    return result


def load_episode(dataset_path: str, episode_index: int):
    """Load episode data from parquet.

    Returns:
        (states, actions) as numpy arrays of shape (N, 22).
    """
    dataset_path = Path(dataset_path)

    # Try flat layout first, then chunked
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
    """Load video frames for all 4 cameras from MP4 files.

    Returns:
        List of dicts mapping camera_name -> RGB numpy array (H, W, 3),
        one dict per frame.
    """
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

        cap = cv2.VideoCapture(str(video_path))
        cam_frames = []
        while len(cam_frames) < num_frames:
            ret, frame = cap.read()
            if not ret:
                break
            # Resize if needed
            if frame.shape[1] != IMAGE_WIDTH or frame.shape[0] != IMAGE_HEIGHT:
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))
            # Convert BGR to RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cam_frames.append(frame)
        cap.release()

        if len(cam_frames) < num_frames:
            logger.warning(
                f'Camera {cam_name}: got {len(cam_frames)} frames, expected {num_frames}'
            )

        frames_per_camera[cam_name] = cam_frames

    # Reorganize to per-frame dicts
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


def plot_trajectory_results(
    state_across_time: np.ndarray,
    gt_action_across_time: np.ndarray,
    pred_action_across_time: np.ndarray,
    episode_index: int,
    action_horizon: int,
    save_plot_path: str,
):
    """Plot ground truth vs predicted actions per joint dimension."""
    actual_steps = len(gt_action_across_time)
    action_dim = gt_action_across_time.shape[1]

    fig, axes = plt.subplots(nrows=action_dim, ncols=1, figsize=(12, 3 * action_dim))
    if action_dim == 1:
        axes = [axes]

    fig.suptitle(
        f'Episode {episode_index} — Open-Loop Eval via ZMQ Client',
        fontsize=16, color='blue',
    )

    for idx in range(action_dim):
        ax = axes[idx]

        # Plot state if same shape
        if state_across_time.shape == gt_action_across_time.shape:
            ax.plot(state_across_time[:, idx], label='state', alpha=0.7)

        ax.plot(gt_action_across_time[:, idx], label='gt action')
        ax.plot(pred_action_across_time[:, idx], label='pred action')

        # Mark inference points
        for j in range(0, actual_steps, action_horizon):
            kwargs = {'label': 'inference point'} if j == 0 else {}
            ax.plot(j, gt_action_across_time[j, idx], 'ro', **kwargs)

        joint_name = JOINT_NAMES[idx] if idx < len(JOINT_NAMES) else f'dim_{idx}'
        ax.set_title(f'{joint_name} (dim {idx})')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    Path(save_plot_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_plot_path, dpi=100)
    plt.close()
    logger.info(f'Trajectory plot saved: {save_plot_path}')


def plot_grouped_results(
    state_across_time: np.ndarray,
    gt_action_across_time: np.ndarray,
    pred_action_across_time: np.ndarray,
    episode_index: int,
    action_horizon: int,
    save_plot_path: str,
):
    """Plot GT vs predicted actions grouped by body part, with per-group error metrics.

    Mirrors the style of Isaac-GR00T open_loop_eval.py — one subplot per joint
    dimension, grouped visually by body part with group-level MSE/MAE annotations.
    """
    actual_steps = len(gt_action_across_time)

    # Count total subplots (one per joint dimension)
    total_dims = sum(end - start for _, start, end in BODY_PART_GROUPS)
    fig, axes = plt.subplots(nrows=total_dims, ncols=1, figsize=(14, 2.5 * total_dims))
    if total_dims == 1:
        axes = [axes]

    fig.suptitle(
        f'Episode {episode_index} — GT vs Predicted by Body Part',
        fontsize=16, color='blue', y=1.0,
    )

    # Color palette for groups
    group_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

    plot_idx = 0
    for group_i, (group_name, start, end) in enumerate(BODY_PART_GROUPS):
        ndims = end - start
        group_gt = gt_action_across_time[:, start:end]
        group_pred = pred_action_across_time[:, start:end]
        group_state = state_across_time[:, start:end]

        # Per-group error
        group_mse = np.mean((group_gt - group_pred) ** 2)
        group_mae = np.mean(np.abs(group_gt - group_pred))

        color = group_colors[group_i % len(group_colors)]

        for d in range(ndims):
            ax = axes[plot_idx]
            dim_idx = start + d
            joint_name = JOINT_NAMES[dim_idx] if dim_idx < len(JOINT_NAMES) else f'dim_{dim_idx}'

            # Plot state, gt, pred
            if state_across_time.shape == gt_action_across_time.shape:
                ax.plot(group_state[:, d], color='gray', alpha=0.5, linewidth=1, label='state')
            ax.plot(group_gt[:, d], color=color, linewidth=1.2, label='gt action')
            ax.plot(group_pred[:, d], color=color, linewidth=1.2, linestyle='--', alpha=0.8,
                    label='pred action')

            # Inference points
            for j in range(0, actual_steps, action_horizon):
                kwargs = {'label': 'inference point'} if j == 0 else {}
                ax.plot(j, group_gt[j, d], 'ro', markersize=3, **kwargs)

            # Title with group context
            ax.set_title(f'{group_name} / {joint_name}', fontsize=10, color=color)
            ax.grid(True, alpha=0.2)

            # Only show legend on first subplot of each group
            if d == 0:
                ax.legend(loc='upper right', fontsize=7)
                # Annotate group-level error
                ax.annotate(
                    f'{group_name}: MSE={group_mse:.6f}  MAE={group_mae:.6f}',
                    xy=(0.01, 0.95), xycoords='axes fraction',
                    fontsize=8, color=color, va='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8),
                )

            # Only show x-axis label on last subplot
            if plot_idx == total_dims - 1:
                ax.set_xlabel('Frame')

            plot_idx += 1

    plt.tight_layout()
    Path(save_plot_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_plot_path, dpi=100, bbox_inches='tight')
    plt.close()
    logger.info(f'Grouped trajectory plot saved: {save_plot_path}')


def plot_comms_report(
    latencies: list[float],
    message_sizes: list[tuple[int, int]],
    save_plot_path: str,
):
    """Plot communication diagnostics."""
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 10))
    fig.suptitle('ZMQ Communication Diagnostics', fontsize=14)

    steps = range(len(latencies))

    # Latency
    ax = axes[0]
    ax.plot(steps, latencies, 'b-', linewidth=1)
    ax.axhline(np.mean(latencies), color='r', linestyle='--', label=f'mean={np.mean(latencies):.1f}ms')
    ax.set_ylabel('Round-trip Latency (ms)')
    ax.set_title('Inference Latency per Request')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Message sizes
    ax = axes[1]
    send_sizes = [s / 1024 for s, _ in message_sizes]
    recv_sizes = [r / 1024 for _, r in message_sizes]
    ax.plot(steps, send_sizes, 'b-', label='send (observation)', linewidth=1)
    ax.plot(steps, recv_sizes, 'r-', label='recv (action)', linewidth=1)
    ax.set_ylabel('Message Size (KB)')
    ax.set_title('Message Sizes per Request')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Effective FPS
    ax = axes[2]
    fps = [1000.0 / l if l > 0 else 0 for l in latencies]
    ax.plot(steps, fps, 'g-', linewidth=1)
    ax.axhline(np.mean(fps), color='r', linestyle='--', label=f'mean={np.mean(fps):.1f} FPS')
    ax.set_ylabel('Effective FPS')
    ax.set_xlabel('Inference Step')
    ax.set_title('Effective Inference Rate')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    Path(save_plot_path).parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(save_plot_path, dpi=100)
    plt.close()
    logger.info(f'Comms report plot saved: {save_plot_path}')


def print_comms_summary(latencies: list[float], message_sizes: list[tuple[int, int]]):
    """Print communication bandwidth and timing summary."""
    if not latencies:
        logger.info('No communication data to report.')
        return

    latencies_arr = np.array(latencies)
    send_sizes = np.array([s for s, _ in message_sizes])
    recv_sizes = np.array([r for _, r in message_sizes])

    print()
    print('=' * 70)
    print('  ZMQ COMMUNICATION SUMMARY')
    print('=' * 70)
    print()

    # Latency
    print('  LATENCY (round-trip)')
    print(f'    Mean:    {np.mean(latencies_arr):8.1f} ms')
    print(f'    Median:  {np.median(latencies_arr):8.1f} ms')
    print(f'    P95:     {np.percentile(latencies_arr, 95):8.1f} ms')
    print(f'    P99:     {np.percentile(latencies_arr, 99):8.1f} ms')
    print(f'    Min:     {np.min(latencies_arr):8.1f} ms')
    print(f'    Max:     {np.max(latencies_arr):8.1f} ms')
    print()

    # Effective FPS
    mean_latency_s = np.mean(latencies_arr) / 1000.0
    effective_fps = 1.0 / mean_latency_s if mean_latency_s > 0 else 0
    print(f'  EFFECTIVE INFERENCE RATE')
    print(f'    Mean:    {effective_fps:8.1f} FPS')
    print(f'    (at 15 FPS client rate, overhead = {np.mean(latencies_arr):.1f}ms per frame)')
    print()

    # Message sizes
    print('  MESSAGE SIZES')
    print(f'    Send (observation):')
    print(f'      Mean:  {np.mean(send_sizes)/1024:8.1f} KB')
    print(f'      Min:   {np.min(send_sizes)/1024:8.1f} KB')
    print(f'      Max:   {np.max(send_sizes)/1024:8.1f} KB')
    print(f'    Recv (action):')
    print(f'      Mean:  {np.mean(recv_sizes)/1024:8.1f} KB')
    print(f'      Min:   {np.min(recv_sizes)/1024:8.1f} KB')
    print(f'      Max:   {np.max(recv_sizes)/1024:8.1f} KB')
    print()

    # Bandwidth
    total_time_s = np.sum(latencies_arr) / 1000.0
    total_send_kb = np.sum(send_sizes) / 1024.0
    total_recv_kb = np.sum(recv_sizes) / 1024.0
    print(f'  BANDWIDTH (over {len(latencies)} requests, {total_time_s:.1f}s)')
    print(f'    Upload:    {total_send_kb/total_time_s:8.1f} KB/s  ({total_send_kb:.0f} KB total)')
    print(f'    Download:  {total_recv_kb/total_time_s:8.1f} KB/s  ({total_recv_kb:.0f} KB total)')
    print(f'    Total:     {(total_send_kb+total_recv_kb)/total_time_s:8.1f} KB/s')
    print()

    # At 15 FPS projection
    print(f'  PROJECTED AT 15 FPS CLIENT RATE')
    print(f'    Upload:    {np.mean(send_sizes)*15/1024:8.1f} KB/s')
    print(f'    Download:  {np.mean(recv_sizes)*15/1024:8.1f} KB/s')
    print(f'    Total:     {(np.mean(send_sizes)+np.mean(recv_sizes))*15/1024:8.1f} KB/s')
    print('=' * 70)
    print()


def run_eval(args):
    """Run open-loop evaluation."""
    dataset_path = args.dataset_path
    episode_index = args.episode_index
    action_horizon = args.action_horizon

    # Resolve defaults
    stats_path = args.stats_path or str(Path(dataset_path) / 'meta' / 'stats.json')
    save_plot = args.save_plot or f'/tmp/groot_eval/episode_{episode_index:03d}.png'
    comms_plot = save_plot.replace('.png', '_comms.png').replace('.jpeg', '_comms.jpeg')
    server_address = build_server_address(
        transport=args.transport, host=args.host, port=args.port,
    )

    # Print all parameters
    print()
    print('=' * 60)
    print('  GR00T Open-Loop Evaluation')
    print('=' * 60)
    print(f'  Dataset:          {dataset_path}')
    print(f'  Episode:          {episode_index}')
    print(f'  Stats:            {stats_path}')
    print(f'  Server:           {server_address}')
    print(f'  Timeout:          {args.timeout_ms} ms')
    print(f'  Action Horizon:   {action_horizon}')
    print(f'  Task:             "{args.task}"')
    print(f'  Closed-Loop:      {args.closed_loop}')
    grouped_plot_path = save_plot.replace('.png', '_grouped.png').replace('.jpeg', '_grouped.jpeg')
    print(f'  Trajectory Plot:  {save_plot}')
    print(f'  Grouped Plot:     {grouped_plot_path}')
    print(f'  Comms Plot:       {comms_plot}')
    print('=' * 60)
    print()

    # Verify server connectivity before loading data
    logger.info(f'Verifying server connectivity at {server_address}...')
    client = ZMQClient(
        server_address=server_address,
        timeout_ms=args.timeout_ms,
        logger=lambda msg: logger.info(msg),
    )

    if not client.connect():
        logger.error(f'Failed to connect to server at {server_address}')
        sys.exit(1)

    if not client.ping(timeout_ms=5000):
        logger.error(
            f'Server at {server_address} is not responding. '
            f'Ensure the GR00T inference server is running.'
        )
        client.close()
        sys.exit(1)

    logger.info(f'Server verified at {server_address}')

    # Load stats
    logger.info(f'Loading stats from: {stats_path}')
    stats = load_stats(stats_path)

    # Load episode
    gt_states, gt_actions = load_episode(dataset_path, episode_index)
    num_frames = len(gt_states)
    logger.info(f'Episode has {num_frames} frames')

    # Load video frames
    logger.info('Loading video frames...')
    video_frames = load_video_frames(dataset_path, episode_index, num_frames)
    logger.info(f'Loaded {len(video_frames)} frames from {len(CAMERA_NAMES)} cameras')

    # Run evaluation — step every action_horizon frames
    pred_action_across_time = []
    num_inference_steps = 0

    # Closed-loop state: updated with predicted actions instead of GT
    closed_loop_state = gt_states[0].copy() if args.closed_loop else None

    for step in range(0, num_frames, action_horizon):
        logger.info(f'Inference at step {step}/{num_frames}')

        # Get observation state at this step
        if args.closed_loop and closed_loop_state is not None:
            state_raw = closed_loop_state.copy()
            logger.info(
                f'[closed-loop] state_base={state_raw[0:6]}, '
                f'gt_base={gt_states[step][0:6]}'
            )
        else:
            state_raw = gt_states[step]

        # Send raw RGB arrays (no JPEG lossy compression)
        # to match the standard eval pipeline as closely as possible
        raw_images = {}
        frame_dict = video_frames[step] if step < len(video_frames) else {}
        for cam_name, img in frame_dict.items():
            raw_images[cam_name] = img  # RGB uint8 numpy array

        # Send raw state — Gr00tPolicy normalizes internally
        response = client.send_raw_observation(
            raw_images=raw_images,
            state=state_raw,
            language=args.task,
        )

        if response is None:
            logger.error(f'Inference failed at step {step}')
            continue

        if response.get('status') != 'ok':
            logger.error(f"Server error at step {step}: {response.get('error_message')}")
            continue

        # Extract action chunk (action_horizon x 22)
        actions_list = response.get('actions', [])
        if not actions_list:
            logger.warning(f'Empty actions at step {step}')
            continue

        for j in range(action_horizon):
            if j < len(actions_list):
                pred_action = np.array(actions_list[j], dtype=np.float32)
                pred_action_across_time.append(pred_action)

                # In closed-loop mode, propagate the predicted action as the
                # next state (simulates what the robot would actually see).
                # Base velocity (cmd_vel) commands become the new base state.
                # Joint positions become the new joint state.
                if args.closed_loop:
                    closed_loop_state[0:6] = pred_action[0:6]   # base velocity
                    closed_loop_state[6:22] = pred_action[6:22] # joint positions

        num_inference_steps += 1

    client.close()

    # Trim to match lengths
    actual_steps = min(num_frames, len(pred_action_across_time))
    gt_states_trimmed = gt_states[:actual_steps]
    gt_actions_trimmed = gt_actions[:actual_steps]
    pred_actions_arr = np.array(pred_action_across_time[:actual_steps])

    logger.info(f'Completed {num_inference_steps} inference steps, {actual_steps} total action frames')

    if actual_steps == 0:
        logger.error('No successful inference steps. Cannot compute metrics.')
        client.close()
        sys.exit(1)

    # Compute metrics
    eval_mode = 'CLOSED-LOOP' if args.closed_loop else 'OPEN-LOOP'
    mse = np.mean((gt_actions_trimmed - pred_actions_arr) ** 2)
    mae = np.mean(np.abs(gt_actions_trimmed - pred_actions_arr))

    print()
    print(f'  [{eval_mode}] MSE: {mse:.6f}')
    print(f'  [{eval_mode}] MAE: {mae:.6f}')
    print()

    # Per-joint metrics
    per_joint_mse = np.mean((gt_actions_trimmed - pred_actions_arr) ** 2, axis=0)
    per_joint_mae = np.mean(np.abs(gt_actions_trimmed - pred_actions_arr), axis=0)
    print('  Per-joint MSE / MAE:')
    for i, name in enumerate(JOINT_NAMES):
        print(f'    {name:25s}  MSE={per_joint_mse[i]:.6f}  MAE={per_joint_mae[i]:.6f}')
    print()

    # Plot trajectory
    plot_trajectory_results(
        state_across_time=gt_states_trimmed,
        gt_action_across_time=gt_actions_trimmed,
        pred_action_across_time=pred_actions_arr,
        episode_index=episode_index,
        action_horizon=action_horizon,
        save_plot_path=save_plot,
    )

    # Grouped body-part plot
    grouped_plot = save_plot.replace('.png', '_grouped.png').replace('.jpeg', '_grouped.jpeg')
    plot_grouped_results(
        state_across_time=gt_states_trimmed,
        gt_action_across_time=gt_actions_trimmed,
        pred_action_across_time=pred_actions_arr,
        episode_index=episode_index,
        action_horizon=action_horizon,
        save_plot_path=grouped_plot,
    )

    # Comms diagnostics
    latencies = client.get_latency_history()
    message_sizes = client.get_message_size_history()

    print_comms_summary(latencies, message_sizes)

    if latencies:
        plot_comms_report(latencies, message_sizes, comms_plot)

    logger.info('Done.')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Open-loop evaluation via GR00T ZMQ client→server pipeline',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
examples:
  # Eval episode 0 against remote server
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-index 0 --host 192.168.50.108 --port 5555

  # Eval with custom save path
  %(prog)s --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \\
           --episode-index 3 --host 192.168.50.108 --port 5555 \\
           --save-plot /tmp/eval_ep3.png
""",
    )

    parser.add_argument(
        '--dataset-path', required=True,
        help='Path to LeRobot-format dataset',
    )
    parser.add_argument(
        '--episode-index', type=int, default=0,
        help='Episode index to evaluate (default: 0)',
    )
    parser.add_argument(
        '--transport', choices=['tcp', 'ipc'], default='tcp',
        help='ZMQ transport type (default: tcp)',
    )
    parser.add_argument(
        '--host', default='192.168.50.108',
        help='Server host for TCP (default: 192.168.50.108)',
    )
    parser.add_argument(
        '--port', '-p', type=int, default=5555,
        help='Server port for TCP (default: 5555)',
    )
    parser.add_argument(
        '--timeout-ms', type=int, default=5000,
        help='Inference timeout in ms (default: 5000)',
    )
    parser.add_argument(
        '--stats-path', default='',
        help='Path to stats.json (default: {dataset_path}/meta/stats.json)',
    )
    parser.add_argument(
        '--action-horizon', type=int, default=16,
        help='Action horizon per inference step (default: 16)',
    )
    parser.add_argument(
        '--task', default='find the can and pick it up',
        help='Task description sent to model (default: "find the can and pick it up")',
    )
    parser.add_argument(
        '--save-plot', default='',
        help='Path to save trajectory plot (default: /tmp/groot_eval/episode_N.png)',
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Enable verbose logging',
    )
    parser.add_argument(
        '--closed-loop', action='store_true',
        help='Simulate closed-loop execution: update state with predicted actions '
             'instead of using ground truth state at each step. Shows how errors '
             'compound, matching live robot behavior.',
    )

    return parser.parse_args()


def main():
    args = parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )

    run_eval(args)


if __name__ == '__main__':
    main()
