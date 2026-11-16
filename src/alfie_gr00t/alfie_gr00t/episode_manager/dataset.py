"""Read and edit one converted GR00T LeRobot dataset on disk.

Everything here is stdlib so the node runs on plain system python. The one
operation that needs pyarrow (rewriting an episode's parquet ``task_index``)
runs ``set_task.py`` under the configured GR00T interpreter instead, the same
interpreter the converter needs.

Layout (see alfie_gr00t/dataset_paths.py):

    <data_root>/<session>/session.json                           recording sessions only
    <data_root>/<session>/demonstrations/demo_YYYYMMDD_HHMMSS/   raw bags
    <data_root>/<session>/lerobot/{meta,data,videos}/            converter output

A recording session (YYYY-MM-DD_sNN_hwN) can be edited until it is closed;
after that every edit here is refused. Plain datasets (recorded before
sessions) have no session.json and are never closed.

Episode indices are never renumbered. The converter leaves gaps when a source
bag disappears, and deleting here does the same, so an episode's index (and
therefore its file names) is stable for its whole life.
"""

import json
import os
import shutil
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from alfie_gr00t.dataset_paths import (
    DEMOS_SUBDIR, LEROBOT_SUBDIR, close_session, session_info,
)
from alfie_gr00t.recording_context import FAILURE_MODES, QUALITIES

CAMERAS = ['left_wide', 'right_wide', 'left_center', 'right_center']

DEFAULT_DATA_PATH = 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet'
DEFAULT_VIDEO_PATH = 'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4'

SET_TASK_SCRIPT = Path(__file__).resolve().parent / 'set_task.py'


class DatasetError(Exception):
    """A request the dataset cannot satisfy; the message goes to the UI."""


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows = []
    if path.exists():
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    rows.append(json.loads(line))
    return rows


def _write_atomic(path: Path, text: str) -> None:
    """Write via a temp file + rename so a crash never leaves half a file."""
    tmp = path.with_name(path.name + '.tmp')
    with open(tmp, 'w') as f:
        f.write(text)
    os.replace(tmp, path)


def recorded_at(source: str) -> Optional[str]:
    """demo_YYYYMMDD_HHMMSS -> 'YYYY-MM-DD HH:MM:SS', or None if it doesn't parse."""
    try:
        _, date_str, time_str = source.split('_')[:3]
        return datetime.strptime(date_str + time_str, '%Y%m%d%H%M%S').strftime('%Y-%m-%d %H:%M:%S')
    except ValueError:
        return None


def episode_bag(ep: Dict[str, Any]) -> str:
    """Bag folder a row came from (``bag``; rows from before sessions use ``source``)."""
    return ep.get('bag') or ep.get('source', '')


def episode_source(ep: Dict[str, Any]) -> str:
    """demo / correction / ... (rows from before sessions: ``origin: takeover``)."""
    if 'bag' in ep:
        return ep.get('source') or 'demo'
    return 'correction' if ep.get('origin') == 'takeover' else 'demo'


# Review-time fields the UI may set on an episode, with their allowed values
# (None = free text).
EDITABLE = {'quality': QUALITIES, 'failure_mode': FAILURE_MODES + (None,), 'notes': None}
# Row fields passed through to the UI as they are.
DETAIL_KEYS = ('episode_uid', 'task_id', 'family', 'round', 'policy_ckpt', 'run_id',
               'instruction_sent', 'takeover_s', 'failure_mode', 'objects', 'surface',
               'hw', 'operator', 'quality', 'notes', 'close_frame', 'done_frame',
               'legacy_dataset', 'legacy_episode_index')


# Written by the GR00T loader into any folder it reads; never part of a session.
STATS_FILES = ('stats.json', 'relative_stats.json')


class Dataset:
    """One dataset folder. Re-reads metadata on every call; it is small."""

    def __init__(self, data_root: Path, name: str):
        self.name = name
        self.root = Path(data_root).expanduser() / name
        self.lerobot = self.root / LEROBOT_SUBDIR
        self.meta = self.lerobot / 'meta'
        self.demos = self.root / DEMOS_SUBDIR

    def session(self) -> Optional[Dict[str, Any]]:
        """session.json, or None for a plain dataset."""
        return session_info(self.root)

    def closed(self) -> bool:
        info = self.session()
        return bool(info and info.get('closed'))

    def _check_open(self) -> None:
        if self.closed():
            raise DatasetError(f'session {self.name} is closed; raw sessions are not '
                               f'edited after closing')

    # -- reading -----------------------------------------------------------

    def info(self) -> Dict[str, Any]:
        path = self.meta / 'info.json'
        if not path.exists():
            return {}
        with open(path) as f:
            return json.load(f)

    def tasks(self) -> Dict[int, str]:
        return {int(t['task_index']): t['task'] for t in _read_jsonl(self.meta / 'tasks.jsonl')}

    def episodes(self) -> List[Dict[str, Any]]:
        return _read_jsonl(self.meta / 'episodes.jsonl')

    def _paths(self, info: Dict[str, Any], index: int) -> Dict[str, Path]:
        chunk = index // int(info.get('chunks_size', 1000))
        data_t = info.get('data_path', DEFAULT_DATA_PATH)
        video_t = info.get('video_path', DEFAULT_VIDEO_PATH)
        paths = {'parquet': self.lerobot / data_t.format(episode_chunk=chunk, episode_index=index)}
        for cam in CAMERAS:
            paths[cam] = self.lerobot / video_t.format(
                episode_chunk=chunk, episode_index=index, video_key=f'observation.images.{cam}')
        return paths

    def video_path(self, index: int, camera: str) -> Optional[Path]:
        if camera not in CAMERAS:
            return None
        path = self._paths(self.info(), index)[camera]
        return path if path.is_file() else None

    def demo_names(self) -> List[str]:
        if not self.demos.is_dir():
            return []
        return sorted(d.name for d in self.demos.iterdir()
                      if d.is_dir() and d.name.startswith('demo_'))

    def summary(self, registry=None) -> Dict[str, Any]:
        """Everything the UI needs in one payload.

        ``tasks`` lists the dataset's own tasks (meta/tasks.jsonl) with their
        registry ID where the string is a registry task, followed by the
        recordable registry tasks the dataset has no episodes of yet
        (``index`` None).
        """
        info = self.info()
        tasks = self.tasks()

        def entry(index, text, reg_task):
            reg_task = reg_task or {}
            return {'index': index, 'task': text, 'id': reg_task.get('id'),
                    'number': reg_task.get('number'),
                    'instruction': reg_task.get('instruction'),
                    'description': reg_task.get('description')}

        task_list = []
        seen = set()
        for i, text in sorted(tasks.items()):
            task_list.append(entry(i, text, registry.by_text(text) if registry else None))
            seen.add(text)
        for reg_task in (registry.recordable() if registry else ()):
            if reg_task['text'] not in seen:
                task_list.append(entry(None, reg_task['text'], reg_task))
        # Ordered by task ID, so the list reads the same in every session;
        # strings that are not registry tasks follow, by index.
        task_list.sort(key=lambda t: (t['id'] is None, t['id'] or '', t['index'] or 0))
        session = self.session()
        demos = set(self.demo_names())
        episodes = []
        for ep in self.episodes():
            index = int(ep['episode_index'])
            files = {key: path.is_file() for key, path in self._paths(info, index).items()}
            bag = episode_bag(ep)
            task_index = ep.get('task_index')
            row = {
                'index': index,
                # None = unlabelled (the converter writes -1 until it is tagged)
                'task_index': task_index if task_index is not None and task_index >= 0 else None,
                'length': ep.get('length'),
                'duration': ep.get('duration'),
                'bag': bag,
                'source': episode_source(ep),
                'recorded': recorded_at(bag),
                'has_source': bag in demos,
                'files': files,
            }
            row.update({k: ep.get(k) for k in DETAIL_KEYS})
            if row['quality'] is None:
                row['quality'] = 'ok'
            episodes.append(row)
        episodes.sort(key=lambda e: e['index'], reverse=True)
        sources = {e['bag'] for e in episodes}

        stats = self.meta / 'stats.json'
        episodes_file = self.meta / 'episodes.jsonl'
        stats_stale = (episodes_file.exists() and
                       (not stats.exists() or stats.stat().st_mtime < episodes_file.stat().st_mtime))

        return {
            'name': self.name,
            'path': str(self.lerobot),
            'demos_path': str(self.demos),
            'converted': (self.meta / 'info.json').is_file(),
            'is_session': session is not None,
            'closed': (session or {}).get('closed'),
            'hw': (session or {}).get('hw'),
            'fps': info.get('fps'),
            'tasks': task_list,
            'unlabelled': sum(1 for e in episodes if e['task_index'] is None),
            'episodes': episodes,
            'total_frames': sum(e['length'] or 0 for e in episodes),
            'pending': sorted(demos - sources),
            # Episodes whose bag is gone. The converter treats a missing bag as
            # "deleted", so converting now would drop every one of these.
            'orphans': sum(1 for e in episodes if not e['has_source']),
            'stats_stale': stats_stale,
        }

    # -- editing -----------------------------------------------------------

    def _find(self, episodes: List[Dict[str, Any]], index: int) -> Dict[str, Any]:
        for ep in episodes:
            if int(ep['episode_index']) == index:
                return ep
        raise DatasetError(f'episode {index} not found')

    def _write_episodes(self, episodes: List[Dict[str, Any]]) -> None:
        episodes.sort(key=lambda e: e['episode_index'])
        _write_atomic(self.meta / 'episodes.jsonl',
                      ''.join(json.dumps(ep) + '\n' for ep in episodes))

    def set_task(self, index: int, task_index: Optional[int], python: str,
                 registry=None, task_id: Optional[str] = None) -> List[str]:
        """Relabel an episode in episodes.jsonl and its parquet task_index column.

        By registry ``task_id`` (added to tasks.jsonl when the dataset has no
        episode of it yet), or by ``task_index`` for tasks outside the registry.
        """
        self._check_open()
        tasks = self.tasks()
        reg_task = None
        if task_id:
            reg_task = registry.by_id(task_id) if registry else None
            if reg_task is None:
                raise DatasetError(f'task "{task_id}" is not in the task registry')
            match = [i for i, text in tasks.items() if text == reg_task['text']]
            if match:
                task_index = match[0]
            else:
                task_index = max(tasks, default=-1) + 1
                tasks[task_index] = reg_task['text']
                _write_atomic(self.meta / 'tasks.jsonl', ''.join(
                    json.dumps({'task_index': i, 'task': tasks[i]}) + '\n'
                    for i in sorted(tasks)))
        elif task_index not in tasks:
            raise DatasetError(f'task {task_index} is not in tasks.jsonl')
        elif registry is not None:
            reg_task = registry.by_text(tasks[task_index])
        episodes = self.episodes()
        ep = self._find(episodes, index)
        log = [f'episode {index}: task {ep.get("task_index")} -> {task_index}'
               + (f' ({reg_task["id"]})' if reg_task else '')]

        info = self.info()
        parquet = self._paths(info, index)['parquet']
        frames = None
        # Parquet first: if it fails, the metadata still matches the data.
        if parquet.is_file():
            cmd = [python, str(SET_TASK_SCRIPT), str(parquet), str(task_index)]
            if reg_task is not None and info.get('fps'):
                # The grasp and done frames depend on the task: recompute them.
                cmd += [reg_task['id'], str(info['fps']), registry.source]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if result.returncode != 0:
                raise DatasetError(f'parquet update failed ({python}): '
                                   f'{(result.stderr or result.stdout).strip()}')
            for line in result.stdout.strip().splitlines():
                if line.startswith('FRAMES '):
                    frames = json.loads(line[len('FRAMES '):])
                else:
                    log.append(line)
        else:
            log.append(f'{parquet.name} not found, metadata only')

        ep['task_index'] = task_index
        if 'tasks' in ep:
            ep['tasks'] = [tasks[task_index]]
        if 'bag' in ep or reg_task is not None:
            # The structured task fields follow the task.
            fields = (registry.episode_fields(reg_task['id']) if reg_task else
                      {'task_id': None, 'family': None, 'verb': None, 'object': None,
                       'destination': None, 'start_state': None, 'end_state': None})
            ep.update(fields)
        if frames is not None:
            ep.update(frames)
            log.append(f'grasp frame {frames["close_frame"]}, done frame {frames["done_frame"]}')
        elif 'bag' in ep:
            ep.update(close_frame=None, done_frame=None)    # belonged to the old task
        self._write_episodes(episodes)
        log.append('episodes.jsonl written')
        return log

    def set_fields(self, index: int, fields: Dict[str, Any]) -> List[str]:
        """Set review fields (quality, failure_mode, notes) on an episode."""
        self._check_open()
        episodes = self.episodes()
        ep = self._find(episodes, index)
        log = []
        for key, value in fields.items():
            if key not in EDITABLE:
                raise DatasetError(f'{key} cannot be edited')
            allowed = EDITABLE[key]
            if allowed is None:
                value = str(value or '').strip()
            elif value not in allowed:
                raise DatasetError(f'{key} must be one of '
                                   f'{", ".join(str(a) for a in allowed if a)}')
            log.append(f'episode {index}: {key} {ep.get(key)!r} -> {value!r}')
            ep[key] = value
        self._write_episodes(episodes)
        return log

    def close(self, extra: Dict[str, Any]) -> List[str]:
        """Close the session: record ``extra`` in session.json, then freeze it."""
        info = self.session()
        if info is None:
            raise DatasetError(f'{self.name} is not a recording session')
        if info.get('closed'):
            raise DatasetError(f'session {self.name} is already closed')
        unlabelled = [int(ep['episode_index']) for ep in self.episodes()
                      if ep.get('task_index') is None or ep['task_index'] < 0]
        if unlabelled:
            raise DatasetError(f'{len(unlabelled)} episodes have no task yet; tag or delete '
                               f'them before closing')
        episodes = self.episodes()
        info.update(extra)
        info['schema'] = self.schema()
        # What a copy of the session must contain, so a sync can be checked.
        info['episodes'] = len(episodes)
        info['frames'] = sum(int(ep.get('length') or 0) for ep in episodes)
        # Normalisation stats belong to a training build, not to a session; a
        # loader pointed at this folder may have left some behind.
        for name in STATS_FILES:
            (self.meta / name).unlink(missing_ok=True)
        _write_atomic(self.root / 'session.json', json.dumps(info, indent=2) + '\n')
        closed = close_session(self.root)
        return [f'session {self.name} closed at {closed["closed"]}']

    def schema(self) -> Dict[str, Any]:
        """What the converted data looks like; training refuses a session
        whose schema differs from its hardware version's."""
        info = self.info()
        features = info.get('features') or {}
        videos = {k: f for k, f in features.items() if f.get('dtype') == 'video'}
        shape = next(iter(videos.values()), {}).get('shape') or [None, None]
        return {
            'fps': info.get('fps'),
            'codebase_version': info.get('codebase_version'),
            'robot_type': info.get('robot_type'),
            'features': {k: {'dtype': f.get('dtype'), 'shape': f.get('shape'),
                             'names': f.get('names')}
                         for k, f in features.items() if k not in videos},
            'video_keys': sorted(videos),
            'video_resolution': {'height': shape[0], 'width': shape[1]},
        }

    def delete(self, index: int) -> List[str]:
        """Remove an episode: its source bag, its output files and its metadata row.

        The bag goes too, otherwise the next conversion would bring it back.
        """
        self._check_open()
        info = self.info()
        episodes = self.episodes()
        ep = self._find(episodes, index)
        log = []

        source = episode_bag(ep)
        source_dir = self.demos / source
        if source and source_dir.is_dir():
            shutil.rmtree(source_dir)
            log.append(f'deleted bag {source_dir}')
        else:
            log.append(f'no bag at {source_dir}')

        for path in self._paths(info, index).values():
            if path.is_file():
                path.unlink()
                log.append(f'deleted {path.relative_to(self.lerobot)}')

        episodes.remove(ep)
        self._write_episodes(episodes)
        log.append(f'episodes.jsonl: {len(episodes)} episodes')

        # Same totals rosbag_to_groot.update_info_json keeps.
        if info:
            info['total_episodes'] = len(episodes)
            info['total_frames'] = sum(int(e.get('length', 0)) for e in episodes)
            info['total_videos'] = len(episodes) * len(CAMERAS)
            info['splits'] = {'train': f'0:{len(episodes)}'}
            _write_atomic(self.meta / 'info.json', json.dumps(info, indent=4))
            log.append('info.json updated')
        return log


class ConvertJob:
    """rosbag_to_groot.py running in the background, output kept for the UI."""

    MAX_LINES = 4000
    HEARTBEAT_S = 30.0

    def __init__(self, cmd: List[str], dataset: str,
                 on_done: Optional[Callable[[str], None]] = None,
                 on_line: Optional[Callable[[str], None]] = None,
                 on_heartbeat: Optional[Callable[[float], None]] = None):
        """``on_line`` sees every converter output line as it arrives;
        ``on_heartbeat(elapsed_s)`` fires every HEARTBEAT_S while it runs."""
        self.cmd = cmd
        self._on_done = on_done
        self._on_line = on_line
        self._on_heartbeat = on_heartbeat
        self._stopped = threading.Event()
        self.dataset = dataset
        self.lines: deque = deque(maxlen=self.MAX_LINES)
        self.started = time.time()
        self.finished: Optional[float] = None
        self.returncode: Optional[int] = None
        self.cancelled = False
        self._lock = threading.Lock()
        self.lines.append('$ ' + ' '.join(cmd))
        self._proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
            # Unbuffered child so progress shows up as it happens.
            env={**os.environ, 'PYTHONUNBUFFERED': '1'})
        threading.Thread(target=self._pump, daemon=True).start()
        if on_heartbeat is not None:
            threading.Thread(target=self._heartbeat, daemon=True).start()

    def _pump(self) -> None:
        for line in self._proc.stdout:
            line = line.rstrip('\n')
            with self._lock:
                self.lines.append(line)
            if self._on_line is not None:
                try:
                    self._on_line(line)
                except Exception:   # logging must never stop the pump draining stdout
                    pass
        self.returncode = self._proc.wait()
        self.finished = time.time()
        self._stopped.set()
        if self._on_done is not None:
            self._on_done(f'conversion of {self.dataset} {self.status()["state"]} '
                          f'after {self.finished - self.started:.0f}s')

    def _heartbeat(self) -> None:
        while not self._stopped.wait(self.HEARTBEAT_S):
            try:
                self._on_heartbeat(time.time() - self.started)
            except Exception:
                pass

    @property
    def running(self) -> bool:
        return self.finished is None

    def cancel(self) -> None:
        if self.running:
            self.cancelled = True
            self._proc.terminate()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            lines = list(self.lines)
        if self.running:
            state = 'running'
        elif self.cancelled:
            state = 'cancelled'
        else:
            state = 'ok' if self.returncode == 0 else 'failed'
        return {
            'state': state,
            'dataset': self.dataset,
            'returncode': self.returncode,
            'started': self.started,
            'elapsed': (self.finished or time.time()) - self.started,
            'log': lines,
        }


def default_python() -> str:
    """The Isaac-GR00T venv if present (it has pyarrow), else this interpreter."""
    venv = Path('~/Isaac-GR00T/.venv/bin/python').expanduser()
    return str(venv) if venv.exists() else sys.executable
