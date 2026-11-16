#!/usr/bin/env python3
"""Copy closed recording sessions between the robot and the training machine.

    sync_sessions.py REMOTE              compare: what would be copied (nothing is)
    sync_sessions.py REMOTE --push       copy closed sessions from here to REMOTE
    sync_sessions.py REMOTE --pull       copy closed sessions from REMOTE to here

REMOTE is an rsync location of a data root, e.g.
``user@workstation:alfie_data/raw``. Sessions are recorded on the robot and
closed there; a closed session never changes, so a copy either matches or is
incomplete. Whether a session is "synced" is decided by comparing the two
sides (rsync dry run), never by a flag written into the session.

Only closed sessions are copied (--pull asks the remote which are closed).
Left out: the converter's decompressed bags (*.mcap; the *.mcap.zstd beside
them is the recording), stats files a GR00T loader may have written, and with
--no-bags the whole demonstrations/ folder (training reads lerobot/ only).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from alfie_gr00t import dataset_paths  # noqa: E402

EXCLUDES = ['*.mcap', 'stats.json', 'relative_stats.json', '.copy_*', '.trim']


def local_closed(root: Path):
    return sorted(d.name for d in root.iterdir()
                  if d.is_dir() and dataset_paths.is_closed(d)) if root.is_dir() else []


def remote_closed(remote: str):
    """Closed sessions of an rsync location, read through ssh (or locally)."""
    host, _, path = remote.rpartition(':')
    script = ('import json,os,sys\n'
              'root=os.path.expanduser(sys.argv[1])\n'
              'out=[]\n'
              'for n in sorted(os.listdir(root)) if os.path.isdir(root) else []:\n'
              '    try:\n'
              '        info=json.load(open(os.path.join(root,n,"session.json")))\n'
              '    except (OSError, ValueError):\n'
              '        continue\n'
              '    if info.get("closed"): out.append(n)\n'
              'print(json.dumps(out))\n')
    cmd = ['python3', '-c', script, path]
    if host:
        cmd = ['ssh', host, 'python3', '-', path]
    run = subprocess.run(cmd, input=script if host else None, capture_output=True, text=True)
    if run.returncode != 0:
        sys.exit(f'cannot list sessions at {remote}: {run.stderr.strip()}')
    return json.loads(run.stdout)


def rsync(src: str, dst: str, dry: bool, bags: bool):
    cmd = ['rsync', '-a', '--itemize-changes', '--mkpath']
    cmd += [f'--exclude={pattern}' for pattern in EXCLUDES]
    if not bags:
        cmd.append(f'--exclude=/{dataset_paths.DEMOS_SUBDIR}/')
    if dry:
        cmd.append('--dry-run')
    run = subprocess.run(cmd + [src.rstrip('/') + '/', dst.rstrip('/') + '/'],
                         capture_output=True, text=True)
    if run.returncode != 0:
        sys.exit(f'rsync failed for {src}: {run.stderr.strip()}')
    # Lines for files that would be / were transferred (not directory touch-ups).
    return [line for line in run.stdout.splitlines() if line[:2] in ('<f', '>f')]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('remote', help='rsync location of the other data root')
    ap.add_argument('--data-root', default=None, help='local sessions (default ~/alfie_data/raw)')
    direction = ap.add_mutually_exclusive_group()
    direction.add_argument('--push', action='store_true', help='copy from here to the remote')
    direction.add_argument('--pull', action='store_true', help='copy from the remote to here')
    ap.add_argument('--no-bags', action='store_true', help='leave demonstrations/ out')
    ap.add_argument('--only', nargs='+', default=[], metavar='SESSION',
                    help='only these sessions (default: every closed one)')
    args = ap.parse_args()

    root = dataset_paths.data_root(args.data_root)
    remote = args.remote.rstrip('/')
    names = remote_closed(remote) if args.pull else local_closed(root)
    if args.only:
        missing = sorted(set(args.only) - set(names))
        if missing:
            sys.exit(f'not a closed session at the source: {", ".join(missing)}')
        names = [n for n in names if n in args.only]
    if not names:
        print('no closed sessions at the source')
        return
    copy = args.push or args.pull
    pending = 0
    for name in names:
        src, dst = f'{root}/{name}', f'{remote}/{name}'
        if args.pull:
            src, dst = dst, src
        files = rsync(src, dst, dry=not copy, bags=not args.no_bags)
        pending += bool(files)
        if copy:
            left = rsync(src, dst, dry=True, bags=not args.no_bags)
            state = f'copied {len(files)} files' if files else 'already in sync'
            if left:
                sys.exit(f'{name}: {len(left)} files still differ after copying')
        else:
            state = f'{len(files)} files to copy' if files else 'in sync'
        print(f'{name}: {state}')
    if not copy and pending:
        print(f'{pending} of {len(names)} sessions differ; rerun with --push or --pull to copy')


if __name__ == '__main__':
    main()
