"""State shared between the HTTP threads: which dataset is open, and the converter job.

Edits are serialized behind one lock, and refused while a conversion is
running, because the converter rewrites episodes.jsonl and info.json when it
finishes and would silently undo anything done in the meantime.
"""

import json
import re
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from alfie_gr00t import task_registry
from alfie_gr00t import training_views
from alfie_gr00t.dataset_paths import SESSION_RE, create_session, session_info
from alfie_gr00t.episode_manager.dataset import ConvertJob, Dataset, DatasetError
from alfie_gr00t.recording_context import (
    FAILURE_MODES, QUALITIES, SOURCES, merge_context,
)

REGISTRY_FILES = ('tasks.yaml', 'objects.yaml', 'hardware.yaml')


def _git(repo: Path, *args: str) -> Optional[str]:
    try:
        out = subprocess.run(['git', '-C', str(repo), *args], capture_output=True,
                             text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout if out.returncode == 0 else None


class ConvertProgressLog:
    """Turns rosbag_to_groot.py output into ROS log lines.

    The raw output interleaves 8 worker threads, and its "Starting X (queued n/m)"
    lines say when an episode was picked up, not finished. This logs each
    episode's start and finish, warnings at WARN, the sync plan and summary, and
    a heartbeat with what is still in flight so a long encode doesn't look hung.
    """

    _STARTING = re.compile(r'^Starting (\S+) -> (\S+) \(queued (\d+)/(\d+)\)')
    _FINISHED = re.compile(r'^(Done|Failed) (\S+): .*\((\d+)/(\d+) complete\)')
    _WARN = re.compile(r'warning|error|failed|traceback|exception', re.IGNORECASE)
    _INFO_PREFIXES = ('Found ', 'Sync plan', 'Skipping ', 'Removing episode',
                      'Registered task', 'Renaming task', 'Writing episode metadata',
                      'Saved ', 'Updated ', 'Sync complete', 'Skipped:', 'Removed:',
                      'Rebuilt:', 'Added:', 'Total episodes', 'Total frames',
                      'Delete or re-record')

    def __init__(self, logger, dataset: str):
        self.logger = logger
        self.tag = f'[convert {dataset}]'
        self.total = 0
        self.finished = 0
        self.failed: List[str] = []
        self.in_flight: Dict[str, float] = {}   # bag name -> monotonic start
        self._lock = threading.Lock()

    def line(self, raw: str) -> None:
        if self.logger is None:
            return
        text = raw.strip()
        if not text or set(text) <= {'='}:
            return
        m = self._STARTING.match(text)
        if m:
            name, episode, _, total = m.groups()
            with self._lock:
                self.total = int(total)
                self.in_flight[name] = time.monotonic()
                running = len(self.in_flight)
            self.logger.info(f'{self.tag} start {name} -> {episode} ({running} running)')
            return
        m = self._FINISHED.match(text)
        if m:
            verb, name, done, total = m.groups()
            with self._lock:
                self.in_flight.pop(name, None)
                self.finished, self.total = int(done), int(total)
                if verb == 'Failed':
                    self.failed.append(name)
            log = self.logger.warn if verb == 'Failed' else self.logger.info
            log(f'{self.tag} {text}')
            return
        if self._WARN.search(text) and not text.startswith(('Skipped:', 'Rebuilt:')):
            self.logger.warn(f'{self.tag} {text}')
        elif text.startswith(self._INFO_PREFIXES) and not text.startswith('Saved parquet'):
            self.logger.info(f'{self.tag} {text}')

    def heartbeat(self, elapsed: float) -> None:
        if self.logger is None:
            return
        with self._lock:
            now = time.monotonic()
            flight = sorted(self.in_flight.items(), key=lambda kv: kv[1])
            done, total, failed = self.finished, self.total, len(self.failed)
        if not total:
            self.logger.info(f'{self.tag} still running after {elapsed:.0f}s '
                             f'(scanning bags / planning sync)')
            return
        busy = ', '.join(f'{n} {now - t:.0f}s' for n, t in flight) or 'none'
        phase = 'encoding' if flight else 'writing metadata'
        self.logger.info(f'{self.tag} {elapsed:.0f}s: {done}/{total} finished'
                         f'{f" ({failed} failed)" if failed else ""}, {phase}; '
                         f'in progress: {busy}')


class EpisodeManager:

    def __init__(self, data_root: str, dataset: str, python: str, converter: str,
                 num_threads: int, use_gpu: bool, logger=None,
                 tasks_yaml: Optional[str] = None,
                 workspace_dir: str = '~/alfiebot_ws',
                 isaac_groot_dir: str = '~/Isaac-GR00T'):
        self.data_root = Path(data_root).expanduser()
        self.tasks_yaml = tasks_yaml or None
        self.workspace_dir = Path(workspace_dir).expanduser()
        self.isaac_groot_dir = Path(isaac_groot_dir).expanduser()
        # Recording context as data_recorder last published it (None = not seen),
        # and how to send it an update (set by the node).
        self.context: Optional[Dict[str, Any]] = None
        self.send_context: Optional[Callable[[Dict[str, Any]], None]] = None
        if not dataset:
            names = self.sessions()
            dataset = names[0] if names else ''
        self.dataset = Dataset(self._root_of(dataset), dataset)
        self.python = python
        self.converter = converter
        self.num_threads = num_threads
        self.use_gpu = use_gpu
        self.logger = logger
        self.job: Optional[ConvertJob] = None
        self.recorder_status = ''          # last /alfie/recording/status, '' if never seen
        self._lock = threading.Lock()

    def _log(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.info(msg)

    def _job_done(self, msg: str) -> None:
        if self.logger is None:
            return
        job = self.job
        if job is not None and job.returncode not in (0, None) and not job.cancelled:
            self.logger.error(f'{msg} (exit code {job.returncode}); full output in the '
                              f'UI converter panel')
        else:
            self.logger.info(msg)

    def _busy(self) -> bool:
        return self.job is not None and self.job.running

    # -- reads -------------------------------------------------------------

    @staticmethod
    def _folders(root: Optional[Path]) -> List[str]:
        if root is None or not root.is_dir():
            return []
        return sorted(d.name for d in root.iterdir()
                      if d.is_dir() and not d.name.startswith('.'))

    def sessions(self) -> List[str]:
        """Folders under the data root, newest session first."""
        return sorted(self._folders(self.data_root), reverse=True)

    def datasets(self) -> List[str]:
        """Every folder under the data root, newest session first."""
        return self.sessions()

    def _root_of(self, name: str) -> Path:
        return self.data_root

    def registry(self):
        """The task registry, re-read each time so edits show up (None if broken)."""
        try:
            return task_registry.load(self.tasks_yaml)
        except Exception as exc:   # noqa: BLE001
            if self.logger is not None:
                self.logger.error(f'cannot read the task registry: {exc}')
            return None

    def _hw(self) -> Optional[str]:
        try:
            return task_registry.current_hw(self.tasks_yaml)
        except Exception:   # noqa: BLE001
            return None

    def registry_info(self) -> Dict[str, Any]:
        reg = self.registry()
        tasks = [{'id': t['id'], 'number': t.get('number'), 'task': t['text'],
                  'instruction': t.get('instruction'), 'description': t.get('description'),
                  'family': t.get('family'), 'status': t['status'],
                  'recordable': t['status'] in task_registry.RECORDABLE}
                 for t in sorted(reg.tasks.values() if reg else (), key=lambda t: t['id'])]
        return {'source': reg.source if reg else None, 'tasks': tasks, 'hw': self._hw(),
                'sources': list(SOURCES), 'qualities': list(QUALITIES),
                'failure_modes': list(FAILURE_MODES)}

    def session_list(self) -> List[Dict[str, Any]]:
        """Every folder under the data root with whether it is still open."""
        out = []
        for name in self.datasets():
            info = session_info(self.data_root / name)
            match = SESSION_RE.match(name)
            out.append({'name': name, 'is_session': info is not None,
                        'closed': bool(info and info.get('closed')),
                        'day': match.group(1) if match else None})
        return out

    def surfaces(self) -> List[str]:
        """Surfaces already used in any session, for the recording bar to offer."""
        seen = set()
        for name in self.datasets():
            path = self.data_root / name / 'lerobot' / 'meta' / 'episodes.jsonl'
            try:
                with open(path) as f:
                    for line in f:
                        if '"surface"' in line:
                            seen.add(str(json.loads(line).get('surface') or ''))
            except (OSError, ValueError):
                continue
        return sorted(s for s in seen if s)

    def _training_facts(self, summary: Dict[str, Any]) -> None:
        """Mark each episode with what training does with it: excluded after
        closing (exclusions.jsonl) and held out by the newest build here."""
        excluded = training_views.read_exclusions(self.data_root)
        views_root = self.data_root.parent / 'views'
        build = training_views.latest_build(views_root)
        heldout: set = set()
        if build:
            try:
                with open(views_root / build / 'manifest.json') as f:
                    heldout = set(json.load(f)['views'].get(training_views.HELDOUT_VIEW, []))
            except (OSError, ValueError, KeyError):
                build = None
        summary['build'] = build
        for ep in summary['episodes']:
            uid = ep.get('episode_uid')
            ep['excluded'] = excluded.get(uid)
            ep['heldout'] = build if uid in heldout else None

    def exclude(self, index: int, excluded: bool, reason: str = '') -> Dict[str, Any]:
        """Leave an episode out of future training builds (or take that back).

        The way to drop an episode of a closed session, which cannot be edited.
        """
        reason = str(reason or '').strip()
        if excluded and not reason:
            return {'ok': False, 'error': 'say why the episode is excluded'}
        for ep in self.dataset.episodes():
            if int(ep['episode_index']) == index:
                uid = ep.get('episode_uid') or f'{self.dataset.name}/{index:06d}'
                training_views.add_exclusion(self.data_root, uid, excluded, reason)
                what = f'excluded from training: {reason}' if excluded else 'no longer excluded'
                self._log(f'[{self.dataset.name}] episode {index} {what}')
                return {'ok': True, 'log': [f'episode {index} {what}']}
        return {'ok': False, 'error': f'episode {index} not found'}

    def describe(self) -> Dict[str, Any]:
        summary = self.dataset.summary(self.registry())
        self._training_facts(summary)
        return {
            'data_root': str(self.data_root),
            'datasets': self.datasets(),
            'sessions': self.session_list(),
            'surfaces': self.surfaces(),
            'dataset': summary,
            'registry': self.registry_info(),
            'context': self.context,
            'python': self.python,
            'busy': self._busy(),
            'recorder': self.recorder_status,
        }

    def status(self) -> Dict[str, Any]:
        """Cheap poll: enough for the UI to know when to reload the full summary."""
        ds = self.dataset
        sources = {ep.get('bag') or ep.get('source') for ep in ds.episodes()}
        return {
            'dataset': ds.name,
            'closed': ds.closed(),
            'context': self.context,
            'busy': self._busy(),
            'recorder': self.recorder_status,
            'pending': sum(1 for name in ds.demo_names() if name not in sources),
        }

    def job_status(self) -> Dict[str, Any]:
        if self.job is None:
            return {'state': 'idle', 'log': []}
        return self.job.status()

    # -- edits -------------------------------------------------------------

    def _edit(self, what: str, fn: Callable[[], List[str]]) -> Dict[str, Any]:
        with self._lock:
            if self._busy():
                return {'ok': False, 'error': 'a conversion is running; wait for it to finish'}
            try:
                log = fn()
            except DatasetError as exc:
                return {'ok': False, 'error': str(exc)}
        self._log(f'[{self.dataset.name}] {what}: ' + '; '.join(log))
        return {'ok': True, 'log': log}

    def select_dataset(self, name: str) -> Dict[str, Any]:
        if name not in self.datasets():
            return {'ok': False, 'error': f'no dataset "{name}" under {self.data_root}'}
        with self._lock:
            if self._busy():
                return {'ok': False, 'error': 'a conversion is running; wait for it to finish'}
            self.dataset = Dataset(self._root_of(name), name)
        self._log(f'opened dataset {name}')
        return {'ok': True}

    def set_task(self, index: int, task_index: Optional[int] = None,
                 task_id: Optional[str] = None) -> Dict[str, Any]:
        return self._edit(f'episode {index} task',
                          lambda: self.dataset.set_task(index, task_index, self.python,
                                                        self.registry(), task_id))

    def set_fields(self, index: int, fields: Dict[str, Any]) -> Dict[str, Any]:
        return self._edit(f'episode {index} review',
                          lambda: self.dataset.set_fields(index, fields))

    # -- recording context and sessions ------------------------------------

    def set_context(self, update: Dict[str, Any]) -> Dict[str, Any]:
        """Change what the recorder writes with the next episodes."""
        if self.context is None or self.send_context is None:
            return {'ok': False, 'error': 'the data recorder is not running '
                                          '(no /alfie/recording/context yet)'}
        reg = self.registry()
        if 'session' in update:
            name = str(update['session'] or '')
            if name and (not (self.data_root / name).is_dir()
                         or Dataset(self.data_root, name).closed()):
                return {'ok': False, 'error': f'"{name}" is not an open session'}
            if self._recording():
                return {'ok': False, 'error': 'recording; stop before switching session'}
        merged, error = merge_context(self.context, update, reg.tasks if reg else {})
        if error:
            return {'ok': False, 'error': error}
        self.send_context(update)
        # Shown at once; the recorder's own (latched) answer replaces it.
        self.context = {**self.context, **merged,
                        **({'session': str(update['session'] or '')}
                           if 'session' in update else {})}
        return {'ok': True, 'context': self.context}

    def _recording(self) -> bool:
        return not (self.recorder_status or 'STOPPED').startswith('STOPPED')

    def new_session(self) -> Dict[str, Any]:
        """Start the next session of today, open it and point the recorder at it."""
        hw = self._hw()
        if hw is None:
            return {'ok': False, 'error': 'cannot read hardware.yaml for the hardware version'}
        with self._lock:
            if self._busy():
                return {'ok': False, 'error': 'a conversion is running; wait for it to finish'}
            if self._recording():
                return {'ok': False, 'error': 'recording; stop before starting a session'}
            try:
                name = create_session(self.data_root, hw)
            except OSError as exc:
                return {'ok': False, 'error': f'cannot create a session: {exc}'}
            self.dataset = Dataset(self.data_root, name)
        self._log(f'new session {name}')
        if self.context is not None and self.send_context is not None:
            self.send_context({'session': name})
            self.context = {**self.context, 'session': name}
        return {'ok': True, 'session': name}

    def _close_facts(self) -> Dict[str, Any]:
        """Registry snapshot into the session, and the commits in use."""
        ds = self.dataset
        src = task_registry.default_path() if self.tasks_yaml is None else Path(self.tasks_yaml)
        snapshot = ds.root / 'registry'
        snapshot.mkdir(exist_ok=True)
        dirty = False
        for name in REGISTRY_FILES:
            path = src.parent / name
            if not path.is_file():
                raise DatasetError(f'registry file {path} is missing')
            shutil.copyfile(path, snapshot / name)
            committed = _git(self.workspace_dir, 'show', f'HEAD:src/alfie_gr00t/config/{name}')
            dirty = dirty or committed != path.read_text()
        commit = (_git(self.workspace_dir, 'rev-parse', 'HEAD') or '').strip() or None
        return {
            # The registry copy in registry/ is what recording used; it equals
            # this commit's unless registry_dirty.
            'registry_commit': commit,
            'registry_dirty': dirty,
            'alfiebot_commit': commit,
            'isaac_groot_commit': (_git(self.isaac_groot_dir, 'rev-parse', 'HEAD') or '').strip()
            or None,
        }

    def close_session(self, force: bool = False) -> Dict[str, Any]:
        """Freeze the open session. Unconverted bags need ``force``."""
        ds = self.dataset
        if self._recording() and f':{ds.name}/' in self.recorder_status:
            return {'ok': False, 'error': 'the recorder is writing into this session; stop it'}
        summary = ds.summary()
        if not summary['episodes']:
            return {'ok': False, 'error': 'nothing converted in this session yet'}
        if summary['pending'] and not force:
            return {'ok': False, 'needs_force': True, 'pending': len(summary['pending']),
                    'error': f'{len(summary["pending"])} bags are not converted'}
        result = self._edit(f'close session {ds.name}', lambda: ds.close(self._close_facts()))
        if (result['ok'] and self.context is not None and self.send_context is not None
                and self.context.get('session') == ds.name):
            self.send_context({'session': ''})   # the recorder moves on to a new session
            self.context = {**self.context, 'session': ''}
        return result

    def delete(self, index: int) -> Dict[str, Any]:
        return self._edit(f'delete episode {index}', lambda: self.dataset.delete(index))

    def convert(self, task_index: Optional[int], force: bool, task_name: str = '',
                task_id: str = '') -> Dict[str, Any]:
        """Start rosbag_to_groot.py.

        A bag recorded with a task (its episode.json) keeps it; one without
        becomes an unlabelled episode to tag afterwards. ``task_id`` (registry)
        or ``task_index`` (tasks.jsonl) labels such bags instead. ``task_name``
        registers (or renames) ``task_index`` in tasks.jsonl first, for tasks
        outside the registry.
        """
        task_name = task_name.strip()
        with self._lock:
            if self._busy():
                return {'ok': False, 'error': 'a conversion is already running'}
            ds = self.dataset
            if ds.closed():
                return {'ok': False, 'error': f'session {ds.name} is closed'}
            summary = ds.summary()
            if (not task_id and not task_name and task_index is not None
                    and task_index not in {t['index'] for t in summary['tasks']}):
                return {'ok': False, 'error': f'task {task_index} is not in tasks.jsonl; name it'}
            if summary['orphans'] and not force:
                return {'ok': False, 'needs_force': True, 'orphans': summary['orphans'],
                        'error': f'{summary["orphans"]} episodes have no bag in '
                                 f'{ds.demos}; converting would delete them'}
            cmd = [self.python, self.converter,
                   '--demos-dir', str(ds.demos),
                   '--output-dir', str(ds.lerobot),
                   '--dataset', ds.name,
                   '--num-threads', str(self.num_threads)]
            if self.tasks_yaml:
                cmd += ['--tasks-yaml', self.tasks_yaml]
            if task_id:
                cmd += ['--task-id', task_id]
            elif task_index is not None:
                cmd += ['--task-index', str(task_index)]
                if task_name:
                    cmd += ['--task-name', task_name]
            if self.use_gpu:
                cmd.append('--use-gpu')
            try:
                progress = ConvertProgressLog(self.logger, ds.name)
                self.job = ConvertJob(cmd, ds.name, on_done=self._job_done,
                                      on_line=progress.line,
                                      on_heartbeat=progress.heartbeat)
            except OSError as exc:
                return {'ok': False, 'error': f'cannot start converter: {exc}'}
        self._log(f'converting {ds.name}: ' + ' '.join(cmd))
        return {'ok': True}

    def cancel(self) -> Dict[str, Any]:
        if not self._busy():
            return {'ok': False, 'error': 'nothing is running'}
        self.job.cancel()
        return {'ok': True}
