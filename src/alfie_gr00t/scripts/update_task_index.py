#!/usr/bin/env python3
"""
Update task_index in parquet files based on episodes.jsonl

Reads the task_index for each episode from episodes.jsonl and updates
the corresponding parquet files to have the correct task_index value.

Usage:
    python3 update_task_index.py [--data-dir PATH] [--dry-run]
"""

import argparse
import json
from pathlib import Path

import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq


def load_episodes_metadata(meta_dir: Path) -> dict[int, int]:
    """Load episode_index -> task_index mapping from episodes.jsonl."""
    episodes_path = meta_dir / 'episodes.jsonl'

    if not episodes_path.exists():
        raise FileNotFoundError(f"episodes.jsonl not found at {episodes_path}")

    mapping = {}
    with open(episodes_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            ep = json.loads(line)
            mapping[ep['episode_index']] = ep['task_index']

    return mapping


def find_parquet_files(data_dir: Path) -> list[Path]:
    """Find all parquet files in the data directory."""
    return sorted(data_dir.glob('chunk-*/episode_*.parquet'))


def update_parquet_task_index(
    parquet_path: Path,
    task_index: int,
    dry_run: bool = False,
) -> tuple[bool, str]:
    """
    Update the task_index column in a parquet file.

    Returns:
        (changed, message) - whether the file was changed and a status message
    """
    # Read the parquet file
    table = pq.read_table(parquet_path)
    df = table.to_pandas()

    # Get current task_index (should be same for all rows)
    current_task_index = df['task_index'].iloc[0]

    if current_task_index == task_index:
        return False, f"already correct (task_index={task_index})"

    if dry_run:
        return True, f"would update task_index {current_task_index} -> {task_index}"

    # Update task_index for all rows
    df['task_index'] = task_index

    # Write back to parquet
    new_table = pa.Table.from_pandas(df)
    pq.write_table(new_table, parquet_path)

    return True, f"updated task_index {current_task_index} -> {task_index}"


def main():
    parser = argparse.ArgumentParser(
        description='Update task_index in parquet files based on episodes.jsonl'
    )
    parser.add_argument(
        '--data-dir',
        type=str,
        default='data/alfiebot.CanDoChallenge',
        help='Root directory containing data/ and meta/ subdirectories'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be changed without actually modifying files'
    )

    args = parser.parse_args()

    data_root = Path(args.data_dir)
    meta_dir = data_root / 'meta'
    data_dir = data_root / 'data'

    print("=" * 60)
    print("Update Task Index in Parquet Files")
    print("=" * 60)
    print(f"Data directory: {data_root}")
    print(f"Dry run: {args.dry_run}")
    print("=" * 60)

    # Load episode metadata
    print("\nLoading episodes.jsonl...")
    episode_task_map = load_episodes_metadata(meta_dir)
    print(f"Found {len(episode_task_map)} episodes in metadata")

    # Find parquet files
    print("\nFinding parquet files...")
    parquet_files = find_parquet_files(data_dir)
    print(f"Found {len(parquet_files)} parquet files")

    # Update each file
    print("\nProcessing parquet files...")
    updated_count = 0
    skipped_count = 0
    error_count = 0

    for pq_path in parquet_files:
        # Extract episode index from filename (e.g., episode_000002.parquet -> 2)
        episode_index = int(pq_path.stem.split('_')[1])

        if episode_index not in episode_task_map:
            print(f"  {pq_path.name}: WARNING - no metadata found, skipping")
            error_count += 1
            continue

        task_index = episode_task_map[episode_index]

        try:
            changed, message = update_parquet_task_index(
                pq_path, task_index, dry_run=args.dry_run
            )

            if changed:
                print(f"  {pq_path.name}: {message}")
                updated_count += 1
            else:
                skipped_count += 1

        except Exception as e:
            print(f"  {pq_path.name}: ERROR - {e}")
            error_count += 1

    # Summary
    print("\n" + "=" * 60)
    print("Summary:")
    print(f"  Updated: {updated_count}")
    print(f"  Skipped (already correct): {skipped_count}")
    print(f"  Errors: {error_count}")

    if args.dry_run and updated_count > 0:
        print("\nThis was a dry run. Run without --dry-run to apply changes.")

    print("=" * 60)


if __name__ == '__main__':
    main()
