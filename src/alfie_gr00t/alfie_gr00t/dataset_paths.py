"""Where episode data lives on disk.

Kept at the package root (not under core/) so ROS nodes running on plain
system python can import it without pulling in the ZMQ/msgpack client stack.
Stdlib only.

Recordings are grouped by recording session, one folder per session under the
data root (``~/alfie_data/raw``):

    <data_root>/YYYY-MM-DD_sNN_hwN/
        session.json      {session, hw, created, closed}
        demonstrations/   raw MCAP bags written by data_recorder
                          (demo_YYYYMMDD_HHMMSS/, each with an episode.json of
                          record-time facts)
        lerobot/          GR00T-flavoured LeRobot v2.1 output of rosbag_to_groot.py
                          (meta/, data/, videos/) -- what training reads

``sNN`` counts the sessions of that day, ``hwN`` is the hardware version
(config/hardware.yaml). A session mixes tasks and sources. Once closed
(``closed`` set in session.json) nothing on the recording side changes it.

A folder with any other name is a plain dataset with the same two sub-folders
and no session.json. The recordings made before sessions existed
(CleanTheRoom) were copied into sessions by scripts/backfill_sessions.py; the
set checkpoint-40000 was trained on stays, converted only, in
~/alfiebot_ws/data/CleanTheRoom/lerobot. The recorder, the converter and the episode manager all
derive their paths from here. ``ALFIE_DATA_ROOT`` and ``ALFIE_DATASET``
override the defaults for scripts run outside ROS launch.
"""

import datetime
import json
import os
import re
from pathlib import Path

DEFAULT_DATA_ROOT = '~/alfie_data/raw'
DEFAULT_DATASET = 'default'

DEMOS_SUBDIR = 'demonstrations'
LEROBOT_SUBDIR = 'lerobot'
SESSION_FILE = 'session.json'
# Record-time facts of one episode, written by data_recorder into its bag folder.
EPISODE_FILE = 'episode.json'

SESSION_RE = re.compile(r'^(\d{4}-\d{2}-\d{2})_s(\d{2,})_(hw\d+)$')


def data_root(root: str | None = None) -> Path:
    """Expanded data root: explicit arg, then $ALFIE_DATA_ROOT, then the default."""
    return Path(os.path.expanduser(root or os.environ.get('ALFIE_DATA_ROOT') or DEFAULT_DATA_ROOT))


def dataset_name(name: str | None = None) -> str:
    """Dataset slug: explicit arg, then $ALFIE_DATASET, then the default."""
    return name or os.environ.get('ALFIE_DATASET') or DEFAULT_DATASET


def dataset_dir(dataset: str | None = None, root: str | None = None) -> Path:
    return data_root(root) / dataset_name(dataset)


def demos_dir(dataset: str | None = None, root: str | None = None) -> Path:
    """Raw rosbag demonstrations for a dataset."""
    return dataset_dir(dataset, root) / DEMOS_SUBDIR


def lerobot_dir(dataset: str | None = None, root: str | None = None) -> Path:
    """Converted GR00T LeRobot v2 dataset (the ``--dataset-path`` for Isaac-GR00T)."""
    return dataset_dir(dataset, root) / LEROBOT_SUBDIR


# -- recording sessions ------------------------------------------------------

def is_session_name(name: str) -> bool:
    return SESSION_RE.match(name) is not None


def session_info(path) -> dict | None:
    """session.json of a session folder, or None for a plain dataset."""
    try:
        with open(Path(path) / SESSION_FILE) as f:
            info = json.load(f)
    except (OSError, ValueError):
        return None
    return info if isinstance(info, dict) else None


def is_closed(path) -> bool:
    info = session_info(path)
    return bool(info and info.get('closed'))


def _now() -> str:
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def _write_session(path: Path, info: dict) -> None:
    tmp = path / (SESSION_FILE + '.tmp')
    with open(tmp, 'w') as f:
        json.dump(info, f, indent=2)
        f.write('\n')
    os.replace(tmp, path / SESSION_FILE)


def next_session_name(root, hw: str, day: datetime.date | None = None) -> str:
    """The next unused YYYY-MM-DD_sNN_hwN for ``day`` (default today)."""
    date = (day or datetime.date.today()).isoformat()
    used = [0]
    root = Path(root)
    if root.is_dir():
        for d in root.iterdir():
            m = SESSION_RE.match(d.name)
            if m and m.group(1) == date:
                used.append(int(m.group(2)))
    return f'{date}_s{max(used) + 1:02d}_{hw}'


def create_session(root, hw: str, day: datetime.date | None = None) -> str:
    """Make the next session folder for ``day`` and return its name."""
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    name = next_session_name(root, hw, day)
    path = root / name
    path.mkdir()
    (path / DEMOS_SUBDIR).mkdir()
    _write_session(path, {'session': name, 'hw': hw, 'created': _now(), 'closed': None})
    return name


def open_session(root, hw: str, day: datetime.date | None = None) -> str | None:
    """Today's latest session on ``hw`` that is not closed, if there is one."""
    date = (day or datetime.date.today()).isoformat()
    root = Path(root)
    best = None
    if root.is_dir():
        for d in root.iterdir():
            m = SESSION_RE.match(d.name)
            if (m and m.group(1) == date and m.group(3) == hw and d.is_dir()
                    and session_info(d) is not None and not is_closed(d)):
                if best is None or int(m.group(2)) > best[0]:
                    best = (int(m.group(2)), d.name)
    return best[1] if best else None


def close_session(path) -> dict:
    """Mark a session closed. Raises ValueError for a folder that is not a session."""
    path = Path(path)
    info = session_info(path)
    if info is None:
        raise ValueError(f'{path} is not a recording session (no {SESSION_FILE})')
    if not info.get('closed'):
        info['closed'] = _now()
        _write_session(path, info)
    return info
