#!/usr/bin/env python3
"""Move a dataset recorded before sessions into dated recording sessions.

A pre-session dataset is one folder of bags (demonstrations/) with a converted
LeRobot set (lerobot/) whose meta/episodes.jsonl says which task each bag is.
This copies every bag into the session of the day it was recorded
(<data-root>/YYYY-MM-DD_sNN_hwN/demonstrations/) and writes the episode.json
the recorder would have written, so rosbag_to_groot.py converts the session
like any other. Each episode keeps where it came from: legacy_dataset and
legacy_episode_index.

The source dataset is only read. Convert the sessions afterwards, compare, and
remove the old bags by hand:

    backfill_sessions.py --source ~/alfiebot_ws/data/CleanTheRoom --hw hw2
    rosbag_to_groot.py --dataset 2026-11-10_s01_hw2 --num-threads 4

Only the compressed bag (.mcap.zstd) and metadata.yaml are copied; the .mcap
next to them is the converter's decompressed copy.
"""

import argparse
import collections
import datetime
import json
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from alfie_gr00t import dataset_paths, task_registry  # noqa: E402

COPY_SKIP = shutil.ignore_patterns('*.mcap', 'episode.json')


def bag_time(name: str) -> datetime.datetime:
    """demo_YYYYMMDD_HHMMSS -> local time the recording started."""
    return datetime.datetime.strptime(name[len('demo_'):], '%Y%m%d_%H%M%S').astimezone()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--source', required=True, help='pre-session dataset folder')
    ap.add_argument('--data-root', default=None, help='where sessions live (default ~/alfie_data/raw)')
    ap.add_argument('--hw', required=True, help='hardware version the dataset was recorded on')
    ap.add_argument('--surface', default='', help='surface the picked objects stood on')
    ap.add_argument('--tasks-yaml', default=None)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    source = Path(args.source).expanduser()
    root = dataset_paths.data_root(args.data_root)
    tasks_yaml = args.tasks_yaml or str(
        Path(__file__).resolve().parent.parent / 'config' / 'tasks.yaml')
    registry = task_registry.load(tasks_yaml)
    if args.hw not in task_registry.load_hardware(tasks_yaml)['versions']:
        sys.exit(f'{args.hw} is not in hardware.yaml')

    with open(source / dataset_paths.LEROBOT_SUBDIR / 'meta' / 'episodes.jsonl') as f:
        rows = [json.loads(line) for line in f if line.strip()]
    demos = source / dataset_paths.DEMOS_SUBDIR

    by_day = collections.defaultdict(list)
    for row in rows:
        bag = row.get('bag') or row['source']
        if not (demos / bag).is_dir():
            sys.exit(f'bag {bag} of episode {row["episode_index"]} is not in {demos}')
        task = registry.by_text(row['tasks'][0]) if row.get('tasks') else None
        if task is None:
            sys.exit(f'episode {row["episode_index"]}: task {row.get("tasks")} is not in the registry')
        by_day[bag_time(bag).date()].append((bag, row, task))

    # A day already backfilled from this dataset is reused, so a rerun adds nothing.
    existing = {}
    if root.is_dir():
        for d in root.iterdir():
            info = dataset_paths.session_info(d)
            if info and info.get('backfilled_from') == source.name:
                existing[info.get('day')] = d.name

    for day in sorted(by_day):
        episodes = sorted(by_day[day], key=lambda e: e[0])
        name = existing.get(day.isoformat())
        print(f'{day}: {len(episodes)} bags -> {name or "new session"}')
        if args.dry_run:
            continue
        if name is None:
            name = dataset_paths.create_session(root, args.hw, day)
            info = dataset_paths.session_info(root / name)
            info.update({'backfilled_from': source.name, 'day': day.isoformat()})
            dataset_paths._write_session(root / name, info)
        out = root / name / dataset_paths.DEMOS_SUBDIR
        for bag, row, task in episodes:
            dst = out / bag
            if not dst.is_dir():
                tmp = out / ('.copy_' + bag)
                shutil.rmtree(tmp, ignore_errors=True)
                shutil.copytree(demos / bag, tmp, ignore=COPY_SKIP)
                os.replace(tmp, dst)
            takeover = row.get('origin') == 'takeover'
            picks = task.get('verb') == 'pick_up'
            record = {
                'bag': bag,
                'session': name,
                'hw': args.hw,
                'recorded_at': bag_time(bag).isoformat(timespec='seconds'),
                'task_id': task['id'],
                'source': 'correction' if takeover else 'demo',
                'round': 1 if takeover else 0,
                'policy_ckpt': None,
                'run_id': None,
                'instruction_sent': None,
                'takeover_s': None,
                'failure_mode': None,
                'operator': '',
                'objects': [o for o in (task.get('object'), task.get('destination')) if o],
                'surface': args.surface if picks else '',
                'quality': 'ok',
                'notes': '',
                'alfiebot_commit': None,
                'legacy_dataset': source.name,
                'legacy_episode_index': row['episode_index'],
            }
            with open(dst / dataset_paths.EPISODE_FILE, 'w') as f:
                json.dump(record, f, indent=2)
                f.write('\n')
        print(f'  {name}: {len(episodes)} bags in {out}')


if __name__ == '__main__':
    main()
