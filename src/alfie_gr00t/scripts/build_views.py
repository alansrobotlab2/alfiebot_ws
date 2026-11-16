#!/usr/bin/env python3
"""Build a training set from the closed recording sessions.

    build_views.py                      new build from every closed session
    build_views.py --from-manifest M    write the build described by manifest M
                                        again from the local sessions

Reads <data-root>/<session>/lerobot of every closed session and writes one
immutable folder (rules: alfie_gr00t/training_views.py):

    <views-root>/<YYYY-MM-DD>_bNN/
        manifest.json    which episodes went where, and from what code/registry
        mix.json         sampling share of each train view (frame-proportional
                         unless --ratio); paths relative to the build folder
        <family>__<source>/lerobot/     train views, e.g. pick_place__demo; --group-by
                                        splits on other episode fields instead
        heldout/lerobot/                the held-out episodes, never trained on
    <views-root>/latest -> the newest build
    <data-root>/../catalog/episodes.parquet   one row per episode of every
                                              closed session, rebuilt each run

Each view is a GR00T LeRobot v2.1 dataset: episodes renumbered from 0 in
(session, episode) order, task_index renumbered against the view's own
tasks.jsonl, parquet rewritten, videos hard-linked (copied across file
systems). No stats files are written; the GR00T loader makes them on first
use. Episodes rated bad are left out, as are those in <data-root>/exclusions.jsonl
(episodes found bad after their session was closed; the episode manager adds
them). Marginal ones are trained on but never held out.

A checkpoint is kept with the manifest of the build it was trained from.
--from-manifest reproduces that build on another machine (the robot, for
engine calibration on the train views and evaluation on heldout/) from its own
copy of the sessions, without copying any dataset.

Needs pyarrow (the Isaac-GR00T venv has it; system python on the Orin does not).
"""

import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pyarrow as pa
import pyarrow.parquet as pq

REPO_PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_PKG))
from alfie_gr00t import dataset_paths, task_registry, training_views as tv  # noqa: E402

CHUNK = 1000
STATS_FILES = ('stats.json', 'relative_stats.json')


def die(msg):
    sys.exit(f'build_views: {msg}')


def git(*args):
    try:
        out = subprocess.run(['git', '-C', str(REPO_PKG), *args], capture_output=True,
                             text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def read_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def load_sessions(root: Path):
    """Closed sessions under ``root``: {name: {'info', 'meta', 'episodes'}}."""
    sessions = {}
    for d in sorted(root.iterdir()) if root.is_dir() else []:
        info = dataset_paths.session_info(d)
        if not info or not info.get('closed'):
            continue
        meta = d / dataset_paths.LEROBOT_SUBDIR / 'meta'
        if not (meta / 'episodes.jsonl').is_file():
            die(f'closed session {d.name} has no converted episodes')
        episodes = read_jsonl(meta / 'episodes.jsonl')
        if info.get('episodes') is not None and info['episodes'] != len(episodes):
            die(f'session {d.name} holds {len(episodes)} episodes but was closed with '
                f'{info["episodes"]}: the copy is incomplete or was changed after closing')
        with open(meta / 'info.json') as f:
            ds_info = json.load(f)
        for ep in episodes:
            ep.setdefault('episode_uid', f'{d.name}/{int(ep["episode_index"]):06d}')
            ep['session'] = d.name
        sessions[d.name] = {'dir': d, 'info': info, 'ds_info': ds_info, 'episodes': episodes}
    return sessions


def check_schema(sessions):
    """Every session must convert to the same columns, cameras and frame rate."""
    ref_name, ref = None, None
    for name, s in sessions.items():
        shape = (s['ds_info'].get('fps'), s['ds_info'].get('robot_type'),
                 json.dumps(s['ds_info'].get('features'), sort_keys=True))
        if ref is None:
            ref_name, ref = name, shape
        elif shape != ref:
            die(f'sessions {ref_name} and {name} differ in fps, robot_type or features; '
                f'they cannot go into one view')


def write_catalog(path: Path, sessions):
    rows = []
    for name, s in sessions.items():
        for ep in s['episodes']:
            row = {k: v for k, v in ep.items() if not isinstance(v, (dict,))}
            row['session_closed'] = s['info'].get('closed')
            rows.append(row)
    keys = sorted({k for r in rows for k in r})
    table = pa.Table.from_pylist([{k: r.get(k) for k in keys} for r in rows])
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.tmp')
    pq.write_table(table, tmp)
    os.replace(tmp, path)
    return len(rows)


def link_or_copy(src: Path, dst: Path):
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def write_view(out: Path, episodes, sessions, wording_of):
    """One LeRobot dataset holding ``episodes`` in the given order."""
    meta = out / 'meta'
    meta.mkdir(parents=True)
    first = sessions[episodes[0]['session']]
    video_keys = [k for k, f in first['ds_info']['features'].items() if f.get('dtype') == 'video']
    tasks, rows, frames = {}, [], 0
    for new, ep in enumerate(episodes):
        src = sessions[ep['session']]['dir'] / dataset_paths.LEROBOT_SUBDIR
        old = int(ep['episode_index'])
        text = wording_of(ep)
        task_index = tasks.setdefault(text, len(tasks))
        chunk_old, chunk_new = f'chunk-{old // CHUNK:03d}', f'chunk-{new // CHUNK:03d}'

        table = pq.read_table(src / 'data' / chunk_old / f'episode_{old:06d}.parquet')
        if table.num_rows != int(ep['length']):
            die(f'{ep["episode_uid"]}: parquet has {table.num_rows} frames, the row says '
                f'{ep["length"]}')
        for column, value in (('episode_index', new), ('task_index', task_index)):
            i = table.schema.get_field_index(column)
            table = table.set_column(i, table.schema.field(i),
                                     pa.array([value] * table.num_rows, table.schema.field(i).type))
        dst = out / 'data' / chunk_new
        dst.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, dst / f'episode_{new:06d}.parquet')

        for key in video_keys:
            vsrc = src / 'videos' / chunk_old / key / f'episode_{old:06d}.mp4'
            if not vsrc.is_file():
                die(f'{ep["episode_uid"]}: video {vsrc} is missing')
            vdst = out / 'videos' / chunk_new / key
            vdst.mkdir(parents=True, exist_ok=True)
            link_or_copy(vsrc, vdst / f'episode_{new:06d}.mp4')

        row = dict(ep)
        row.update({'episode_index': new, 'task_index': task_index, 'tasks': [text],
                    'session_episode_index': old})
        rows.append(row)
        frames += table.num_rows

    with open(meta / 'episodes.jsonl', 'w') as f:
        for row in rows:
            f.write(json.dumps(row) + '\n')
    with open(meta / 'tasks.jsonl', 'w') as f:
        for text, index in tasks.items():
            f.write(json.dumps({'task_index': index, 'task': text}) + '\n')
    info = dict(first['ds_info'])
    info.update({'total_episodes': len(rows), 'total_frames': frames, 'total_tasks': len(tasks),
                 'total_videos': len(rows) * len(video_keys),
                 'total_chunks': (len(rows) - 1) // CHUNK + 1,
                 'splits': {'train': f'0:{len(rows)}'}})
    with open(meta / 'info.json', 'w') as f:
        json.dump(info, f, indent=4)
        f.write('\n')
    modality = first['dir'] / dataset_paths.LEROBOT_SUBDIR / 'meta' / 'modality.json'
    if modality.is_file():
        shutil.copyfile(modality, meta / 'modality.json')
    return {'episodes': len(rows), 'frames': frames, 'tasks': list(tasks)}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--data-root', default=None, help='sessions (default ~/alfie_data/raw)')
    ap.add_argument('--views-root', default=None, help='builds (default <data-root>/../views)')
    ap.add_argument('--tasks-yaml', default=None, help='task registry (default: this checkout)')
    ap.add_argument('--from-manifest', default=None,
                    help='reproduce the build of this manifest.json from the local sessions')
    ap.add_argument('--heldout-fraction', type=float, default=tv.DEFAULT_FRACTION)
    ap.add_argument('--heldout-min', type=int, default=tv.DEFAULT_MINIMUM)
    ap.add_argument('--heldout-min-group', type=int, default=tv.DEFAULT_MIN_GROUP,
                    help='a task/source group with fewer ok episodes than this holds none out')
    ap.add_argument('--salt', default=tv.DEFAULT_SALT)
    ap.add_argument('--previous', default=None,
                    help='build whose held-out episodes stay held out (default: the '
                         'newest build; "none" starts a new held-out set)')
    ap.add_argument('--group-by', default=','.join(tv.DEFAULT_GROUP_BY), metavar='FIELDS',
                    help='episode fields that split training into views, comma separated '
                         '(default %(default)s; e.g. task_id,source or family,source,round)')
    ap.add_argument('--exclude-marginal', action='store_true',
                    help='leave episodes rated marginal out of training too')
    ap.add_argument('--wording', choices=tv.WORDINGS, default='text',
                    help='registry string the episodes are labelled with: text = what the '
                         'deployed policy hears, instruction = the v6 wording')
    ap.add_argument('--ratio', action='append', default=[], metavar='VIEW=SHARE',
                    help='fix the sampling share of a train view (repeatable)')
    ap.add_argument('--dry-run', action='store_true', help='report the split, write nothing')
    args = ap.parse_args()

    root = dataset_paths.data_root(args.data_root)
    views_root = Path(args.views_root).expanduser() if args.views_root else root.parent / 'views'
    tasks_yaml = args.tasks_yaml or str(REPO_PKG / 'config' / 'tasks.yaml')
    registry = task_registry.load(tasks_yaml)
    sessions = load_sessions(root)
    if not sessions:
        die(f'no closed sessions under {root}')
    check_schema(sessions)
    by_uid = {ep['episode_uid']: ep for s in sessions.values() for ep in s['episodes']}

    source_manifest = None
    if args.from_manifest:
        with open(args.from_manifest) as f:
            source_manifest = json.load(f)
        name = source_manifest['build']
        wording = source_manifest['wording']
        lists = dict(source_manifest['views'])
        missing = sorted(u for uids in lists.values() for u in uids if u not in by_uid)
        if missing:
            die(f'{len(missing)} episodes of build {name} are not in a closed session under '
                f'{root} (first: {missing[0]}); nothing written')
        wrong = sorted(u for uids in lists.values() for u in uids
                       if by_uid[u]['length'] != source_manifest['frames'][u])
        if wrong:
            die(f'{len(wrong)} episodes differ in length from build {name} '
                f'(first: {wrong[0]}); nothing written')
        heldout_facts, ratios = source_manifest['heldout'], None
        group_by = source_manifest.get('group_by')
    else:
        name = tv.next_build_name(views_root)
        wording = args.wording
        group_by = [f.strip() for f in args.group_by.split(',') if f.strip()]
        known = {k for ep in by_uid.values() for k in ep}
        if not group_by or set(group_by) - known:
            die(f'--group-by: no episode has the field(s) '
                f'{", ".join(sorted(set(group_by) - known)) or "(none given)"}')
        usable, excluded = [], {}
        listed = tv.read_exclusions(root)
        for uid, ep in by_uid.items():
            if registry.by_id(ep.get('task_id') or '') is None:
                die(f'{uid}: task {ep.get("task_id")!r} is not in the task registry')
            if uid in listed:
                excluded[uid] = f'excluded: {listed[uid]}' if listed[uid] else 'excluded'
            elif ep.get('quality') == 'bad':
                excluded[uid] = 'bad'
            else:
                usable.append(ep)
        prev_name = args.previous or tv.latest_build(views_root)
        previous = []
        if prev_name and prev_name != 'none':
            try:
                with open(views_root / prev_name / 'manifest.json') as f:
                    previous = json.load(f)['views'].get(tv.HELDOUT_VIEW, [])
            except OSError:
                die(f'previous build {prev_name} has no manifest under {views_root}')
        else:
            prev_name = None
        heldout, counts = tv.select_heldout(usable, args.heldout_fraction, args.heldout_min,
                                            args.salt, previous, args.heldout_min_group)
        lists = {}
        for ep in sorted(usable, key=lambda e: (e['session'], int(e['episode_index']))):
            uid = ep['episode_uid']
            if uid in heldout:
                lists.setdefault(tv.HELDOUT_VIEW, []).append(uid)
            elif ep.get('quality') == 'marginal' and args.exclude_marginal:
                excluded[uid] = 'marginal'
            else:
                lists.setdefault(tv.view_name(ep, group_by), []).append(uid)
        heldout_facts = {'rule': 'per task_id x source: the lowest sha256(salt:episode_uid) '
                                 'among quality ok, ceil(fraction x eligible) of them, at '
                                 'least minimum, never all, none from a group smaller than '
                                 'min_group; plus everything the previous '
                                 'build held out',
                         'salt': args.salt, 'fraction': args.heldout_fraction,
                         'minimum': args.heldout_min, 'min_group': args.heldout_min_group,
                         'previous_build': prev_name,
                         'counts': counts, 'excluded': excluded}
        try:
            ratios = dict((r.split('=')[0], float(r.split('=')[1])) for r in args.ratio)
        except (IndexError, ValueError):
            die('--ratio takes VIEW=SHARE, e.g. pick_place__correction=0.12')

    def wording_of(ep):
        task = registry.by_id(ep['task_id'])
        if task is None or not task.get(wording):
            die(f'{ep["episode_uid"]}: task {ep["task_id"]!r} has no `{wording}` in the registry')
        return task[wording]

    train_views = [v for v in lists if v != tv.HELDOUT_VIEW]
    if not train_views:
        die('no training episodes')
    print(f'build {name}: {len(sessions)} closed sessions, {len(by_uid)} episodes')
    for view, uids in lists.items():
        print(f'  {view:28s} {len(uids):5d} episodes {sum(by_uid[u]["length"] for u in uids):8d} frames')
    if args.dry_run:
        for group, c in sorted(heldout_facts.get('counts', {}).items()):
            print(f'  held out {group}: {c["heldout"]} of {c["episodes"]}')
        return
    if (views_root / name).exists():
        die(f'{views_root / name} already exists; a build is never overwritten')

    tmp = views_root / f'.{name}.tmp'
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    written = {}
    for view, uids in lists.items():
        written[view] = write_view(tmp / view / 'lerobot', [by_uid[u] for u in uids],
                                   sessions, wording_of)

    frames = {v: written[v]['frames'] for v in train_views}
    if ratios is None:   # reproducing: the shares are part of the build
        shares = {m['view']: m['ratio'] for m in source_manifest['mix']}
    else:
        try:
            shares = tv.mix_ratios(frames, ratios)
        except ValueError as exc:
            shutil.rmtree(tmp)
            die(str(exc))
    final = views_root / name
    # Paths relative to the build folder, so the file works wherever the build is.
    mix = [{'view': v, 'path': f'{v}/lerobot', 'episodes': written[v]['episodes'],
            'frames': frames[v], 'ratio': round(shares[v], 6)} for v in train_views]
    with open(tmp / 'mix.json', 'w') as f:
        json.dump({'build': name, 'views': mix,
                   'heldout': f'{tv.HELDOUT_VIEW}/lerobot' if tv.HELDOUT_VIEW in lists else None},
                  f, indent=2)
        f.write('\n')
    commit = git('rev-parse', 'HEAD')
    manifest = {
        'build': name,
        'created': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'alfiebot_commit': commit,
        'alfiebot_dirty': bool(git('status', '--porcelain', '--', str(REPO_PKG))),
        'wording': wording,
        'group_by': group_by,
        'sessions': {n: {'closed': s['info'].get('closed'), 'hw': s['info'].get('hw'),
                         'episodes': len(s['episodes']),
                         'registry_commit': s['info'].get('registry_commit')}
                     for n, s in sessions.items()
                     if any(u.startswith(n + '/') for uids in lists.values() for u in uids)},
        'heldout': heldout_facts,
        'mix': mix,
        'tasks': {v: written[v]['tasks'] for v in written},
        'views': lists,
        'frames': {u: by_uid[u]['length'] for uids in lists.values() for u in uids},
    }
    if source_manifest:   # the same build, not a new one: keep what defined it
        for key in ('created', 'alfiebot_commit', 'alfiebot_dirty', 'sessions'):
            manifest[key] = source_manifest[key]
        manifest['reproduced'] = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
        manifest['reproduced_with_commit'] = commit
    with open(tmp / 'manifest.json', 'w') as f:
        json.dump(manifest, f, indent=1)
        f.write('\n')
    os.replace(tmp, final)
    latest = views_root / 'latest'
    if latest.is_symlink() or not latest.exists():
        if tv.latest_build(views_root) == name:
            latest.unlink(missing_ok=True)
            latest.symlink_to(name)
    rows = write_catalog(root.parent / 'catalog' / 'episodes.parquet', sessions)
    print(f'wrote {final}')
    print(f'catalog: {rows} episodes in {root.parent / "catalog" / "episodes.parquet"}')
    print('fine-tune with:')
    print(f'  --dataset-path {os.pathsep.join(str(final / m["path"]) for m in mix)} \\')
    print(f'  --dataset-mix-ratios {" ".join(str(m["ratio"]) for m in mix)}')


if __name__ == '__main__':
    main()
