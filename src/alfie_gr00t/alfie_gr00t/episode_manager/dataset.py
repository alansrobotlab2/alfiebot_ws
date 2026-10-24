"""Read and edit one converted GR00T LeRobot dataset on disk.

Everything here is stdlib so the node runs on plain system python. The one
operation that needs pyarrow (rewriting an episode's parquet ``task_index``)
runs ``set_task.py`` under the configured GR00T interpreter instead, the same
interpreter the converter needs.

Layout (see alfie_gr00t/dataset_paths.py):

    <data_root>/<dataset>/demonstrations/demo_YYYYMMDD_HHMMSS/   raw bags
    <data_root>/<dataset>/lerobot/{meta,data,videos}/            converter output

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

from alfie_gr00t.dataset_paths import DEMOS_SUBDIR, LEROBOT_SUBDIR

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


class Dataset:
    """One dataset folder. Re-reads metadata on every call; it is small."""

    def __init__(self, data_root: Path, name: str):
        self.name = name
        self.root = Path(data_root).expanduser() / name
        self.lerobot = self.root / LEROBOT_SUBDIR
        self.meta = self.lerobot / 'meta'
        self.demos = self.root / DEMOS_SUBDIR

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

    def summary(self) -> Dict[str, Any]:
        """Everything the UI needs in one payload."""
        info = self.info()
        tasks = self.tasks()
        demos = set(self.demo_names())
        episodes = []
        for ep in self.episodes():
            index = int(ep['episode_index'])
            files = {key: path.is_file() for key, path in self._paths(info, index).items()}
            source = ep.get('source', '')
            episodes.append({
                'index': index,
                'task_index': ep.get('task_index'),
                'length': ep.get('length'),
                'duration': ep.get('duration'),
                'source': source,
                'recorded': recorded_at(source),
                'has_source': source in demos,
                'files': files,
            })
        episodes.sort(key=lambda e: e['index'], reverse=True)
        sources = {e['source'] for e in episodes}

        stats = self.meta / 'stats.json'
        episodes_file = self.meta / 'episodes.jsonl'
        stats_stale = (episodes_file.exists() and
                       (not stats.exists() or stats.stat().st_mtime < episodes_file.stat().st_mtime))

        return {
            'name': self.name,
            'path': str(self.lerobot),
            'demos_path': str(self.demos),
            'converted': self.meta.is_dir(),
            'fps': info.get('fps'),
            'tasks': [{'index': i, 'task': t} for i, t in sorted(tasks.items())],
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

    def set_task(self, index: int, task_index: int, python: str) -> List[str]:
        """Relabel an episode in episodes.jsonl and its parquet task_index column."""
        tasks = self.tasks()
        if task_index not in tasks:
            raise DatasetError(f'task {task_index} is not in tasks.jsonl')
        episodes = self.episodes()
        ep = self._find(episodes, index)
        log = [f'episode {index}: task {ep.get("task_index")} -> {task_index}']

        parquet = self._paths(self.info(), index)['parquet']
        # Parquet first: if it fails, the metadata still matches the data.
        if parquet.is_file():
            result = subprocess.run([python, str(SET_TASK_SCRIPT), str(parquet), str(task_index)],
                                    capture_output=True, text=True, timeout=60)
            if result.returncode != 0:
                raise DatasetError(f'parquet update failed ({python}): '
                                   f'{(result.stderr or result.stdout).strip()}')
            log.append(result.stdout.strip())
        else:
            log.append(f'{parquet.name} not found, metadata only')

        ep['task_index'] = task_index
        if 'tasks' in ep:
            ep['tasks'] = [tasks[task_index]]
        self._write_episodes(episodes)
        log.append('episodes.jsonl written')
        return log

    def delete(self, index: int) -> List[str]:
        """Remove an episode: its source bag, its output files and its metadata row.

        The bag goes too, otherwise the next conversion would bring it back.
        """
        info = self.info()
        episodes = self.episodes()
        ep = self._find(episodes, index)
        log = []

        source = ep.get('source', '')
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

    def __init__(self, cmd: List[str], dataset: str,
                 on_done: Optional[Callable[[str], None]] = None):
        self.cmd = cmd
        self._on_done = on_done
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

    def _pump(self) -> None:
        for line in self._proc.stdout:
            with self._lock:
                self.lines.append(line.rstrip('\n'))
        self.returncode = self._proc.wait()
        self.finished = time.time()
        if self._on_done is not None:
            self._on_done(f'conversion of {self.dataset} {self.status()["state"]} '
                          f'after {self.finished - self.started:.0f}s')

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
