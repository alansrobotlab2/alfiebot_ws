"""State shared between the HTTP threads: which dataset is open, and the converter job.

Edits are serialized behind one lock, and refused while a conversion is
running, because the converter rewrites episodes.jsonl and info.json when it
finishes and would silently undo anything done in the meantime.
"""

import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from alfie_gr00t.episode_manager.dataset import ConvertJob, Dataset, DatasetError


class EpisodeManager:

    def __init__(self, data_root: str, dataset: str, python: str, converter: str,
                 num_threads: int, use_gpu: bool, logger=None):
        self.data_root = Path(data_root).expanduser()
        self.dataset = Dataset(self.data_root, dataset)
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

    def _busy(self) -> bool:
        return self.job is not None and self.job.running

    # -- reads -------------------------------------------------------------

    def datasets(self) -> List[str]:
        if not self.data_root.is_dir():
            return []
        return sorted(d.name for d in self.data_root.iterdir()
                      if d.is_dir() and not d.name.startswith('.'))

    def describe(self) -> Dict[str, Any]:
        return {
            'data_root': str(self.data_root),
            'datasets': self.datasets(),
            'dataset': self.dataset.summary(),
            'python': self.python,
            'busy': self._busy(),
            'recorder': self.recorder_status,
        }

    def status(self) -> Dict[str, Any]:
        """Cheap poll: enough for the UI to know when to reload the full summary."""
        ds = self.dataset
        sources = {ep.get('source') for ep in ds.episodes()}
        return {
            'dataset': ds.name,
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
            self.dataset = Dataset(self.data_root, name)
        self._log(f'opened dataset {name}')
        return {'ok': True}

    def set_task(self, index: int, task_index: int) -> Dict[str, Any]:
        return self._edit(f'episode {index} task',
                          lambda: self.dataset.set_task(index, task_index, self.python))

    def delete(self, index: int) -> Dict[str, Any]:
        return self._edit(f'delete episode {index}', lambda: self.dataset.delete(index))

    def convert(self, task_index: int, force: bool, task_name: str = '') -> Dict[str, Any]:
        """Start rosbag_to_groot.py; new bags get labelled ``task_index``.

        ``task_name`` registers (or renames) that task in tasks.jsonl first, which
        is the only way to give a brand-new dataset its first task.
        """
        task_name = task_name.strip()
        with self._lock:
            if self._busy():
                return {'ok': False, 'error': 'a conversion is already running'}
            ds = self.dataset
            summary = ds.summary()
            if not task_name and task_index not in {t['index'] for t in summary['tasks']}:
                return {'ok': False, 'error': f'task {task_index} is not in tasks.jsonl; name it'}
            if summary['orphans'] and not force:
                return {'ok': False, 'needs_force': True, 'orphans': summary['orphans'],
                        'error': f'{summary["orphans"]} episodes have no bag in '
                                 f'{ds.demos}; converting would delete them'}
            cmd = [self.python, self.converter,
                   '--demos-dir', str(ds.demos),
                   '--output-dir', str(ds.lerobot),
                   '--dataset', ds.name,
                   '--task-index', str(task_index),
                   '--num-threads', str(self.num_threads)]
            if task_name:
                cmd += ['--task-name', task_name]
            if self.use_gpu:
                cmd.append('--use-gpu')
            try:
                self.job = ConvertJob(cmd, ds.name, on_done=self._log)
            except OSError as exc:
                return {'ok': False, 'error': f'cannot start converter: {exc}'}
        self._log(f'converting {ds.name}: ' + ' '.join(cmd))
        return {'ok': True}

    def cancel(self) -> Dict[str, Any]:
        if not self._busy():
            return {'ok': False, 'error': 'nothing is running'}
        self.job.cancel()
        return {'ok': True}
