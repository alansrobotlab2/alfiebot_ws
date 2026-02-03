#!/usr/bin/env python3
"""Dump episode actions from parquet to CSV for analysis.

Reads a recorded episode from a LeRobot-format parquet file and writes
both raw and denormalized action values to CSV. Use this to compare
ground-truth episode data against what the client actually commands.

Usage:
    python dump_episode_csv.py \
        --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
        --episode 0 \
        --output /tmp/episode_0_ground_truth.csv
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import pandas as pd


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


def main():
    parser = argparse.ArgumentParser(description='Dump episode actions to CSV')
    parser.add_argument(
        '--dataset-path',
        type=str,
        default='/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge',
        help='Path to LeRobot dataset root',
    )
    parser.add_argument(
        '--episode', type=int, default=0, help='Episode index to dump',
    )
    parser.add_argument(
        '--output', type=str, default='/tmp/episode_ground_truth.csv',
        help='Output CSV path',
    )
    parser.add_argument(
        '--stats-path', type=str, default='',
        help='Path to stats.json (auto-detected if empty)',
    )
    args = parser.parse_args()

    dataset_path = Path(args.dataset_path)
    episode_index = args.episode

    # Find parquet file (try flat layout first, then chunked)
    parquet_path = dataset_path / 'data' / f'episode_{episode_index:06d}.parquet'
    if not parquet_path.exists():
        chunk_idx = episode_index // 1000
        parquet_path = (
            dataset_path / 'data' / f'chunk-{chunk_idx:03d}'
            / f'episode_{episode_index:06d}.parquet'
        )

    if not parquet_path.exists():
        print(f'ERROR: Parquet file not found. Tried:')
        print(f'  {dataset_path / "data" / f"episode_{episode_index:06d}.parquet"}')
        print(f'  {parquet_path}')
        return

    print(f'Reading: {parquet_path}')
    df = pd.read_parquet(parquet_path)

    # Extract raw actions
    raw_actions = np.array(df['action'].tolist(), dtype=np.float32)
    print(f'Episode {episode_index}: {len(raw_actions)} steps, shape={raw_actions.shape}')

    # Load stats for denormalization
    stats_path = args.stats_path or str(dataset_path / 'meta' / 'stats.json')
    stats_path = Path(stats_path)

    action_mean = np.zeros(22, dtype=np.float32)
    action_std = np.ones(22, dtype=np.float32)

    if stats_path.exists():
        with open(stats_path, 'r') as f:
            stats = json.load(f)
        action_stats = stats.get('action', {})
        if 'mean' in action_stats:
            action_mean = np.array(action_stats['mean'], dtype=np.float32)
        if 'std' in action_stats:
            action_std = np.array(action_stats['std'], dtype=np.float32)
            action_std = np.where(action_std == 0, 1.0, action_std)
        print(f'Loaded stats from: {stats_path}')
    else:
        print(f'WARNING: Stats file not found: {stats_path}, skipping denormalization')

    # Compute normalized and denormalized versions
    normalized_actions = (raw_actions - action_mean) / action_std
    denormalized_actions = raw_actions * action_std + action_mean

    # Also compute what happens if raw data IS already normalized
    # (denormalize directly without normalizing first)
    direct_denorm = raw_actions * action_std + action_mean

    # Write CSV
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Header
        header = ['step']
        for prefix in ['raw', 'normalized', 'denorm_if_raw_is_raw', 'denorm_if_raw_is_normalized']:
            for name in JOINT_NAMES:
                header.append(f'{prefix}_{name}')
        writer.writerow(header)

        # Data rows
        for step in range(len(raw_actions)):
            row = [step]
            row.extend(raw_actions[step].tolist())
            row.extend(normalized_actions[step].tolist())
            # If raw data is truly raw: normalize then denormalize = identity
            row.extend(raw_actions[step].tolist())
            # If raw data is already normalized: just denormalize
            row.extend(direct_denorm[step].tolist())
            writer.writerow(row)

    print(f'Wrote {len(raw_actions)} rows to: {output_path}')

    # Print first few rows summary for quick inspection
    print(f'\n--- First step raw values ---')
    for i, name in enumerate(JOINT_NAMES):
        raw_val = raw_actions[0][i]
        mean_val = action_mean[i]
        std_val = action_std[i]
        print(
            f'  {name:25s}: raw={raw_val:10.6f}  '
            f'mean={mean_val:10.6f}  std={std_val:10.6f}  '
            f'denorm_if_normalized={raw_val * std_val + mean_val:10.6f}'
        )

    # Also extract and dump state if available
    if 'observation.state' in df.columns:
        raw_states = np.array(df['observation.state'].tolist(), dtype=np.float32)
        state_output = str(output_path).replace('.csv', '_states.csv')
        with open(state_output, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['step'] + [f'state_{name}' for name in JOINT_NAMES]
            writer.writerow(header)
            for step in range(len(raw_states)):
                row = [step] + raw_states[step].tolist()
                writer.writerow(row)
        print(f'Wrote {len(raw_states)} state rows to: {state_output}')


if __name__ == '__main__':
    main()
