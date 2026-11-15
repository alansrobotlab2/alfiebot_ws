"""The GR00T task registry (config/tasks.yaml): one entry per trained task.

Kept at the package root (not under core/) so the agent and scripts can import
it without the ZMQ/msgpack client stack. Needs only PyYAML.

A task carries its exact training string, start pose, time cap and the
conditions under which it is finished. Conditions are clauses over the measured
22D state, evaluated through a ``get(name)`` callable so the policy client (22D
vector) and the agent's body (RobotLowState) share one evaluator.
"""

import os
from pathlib import Path

import yaml

_ARM = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll')

# Single joints -> index in the 22D state/action vector.
JOINTS = {'back': 6, 'left_gripper': 12, 'right_gripper': 18,
          'head_yaw': 19, 'head_pitch': 20, 'head_roll': 21}
JOINTS.update({f'left_{n}': 7 + i for i, n in enumerate(_ARM)})
JOINTS.update({f'right_{n}': 13 + i for i, n in enumerate(_ARM)})

# Joint groups -> (start, stop) in the 22D vector. Arms exclude the gripper.
GROUPS = {'left_arm': (7, 12), 'right_arm': (13, 18), 'head': (19, 22)}

_POSE_SIZES = {'left_arm': 6, 'right_arm': 6, 'head': 3}
_TASK_KEYS = {'text', 'dataset', 'description', 'start_pose', 'max_s', 'attempts',
              'holds_object', 'done', 'accept_on_timeout'}


class RegistryError(ValueError):
    """tasks.yaml is malformed."""


def default_path():
    """$ALFIE_TASKS_YAML, else the installed share copy, else the source tree."""
    env = os.environ.get('ALFIE_TASKS_YAML')
    if env:
        return Path(os.path.expanduser(env))
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory('alfie_gr00t')) / 'config' / 'tasks.yaml'
        if installed.is_file():
            return installed
    except Exception:   # noqa: BLE001 - not installed / no ament: use the source tree
        pass
    return Path(__file__).resolve().parents[1] / 'config' / 'tasks.yaml'


def state_getter(state):
    """``get(name)`` over a 22D state vector: a float for a joint, a list for a group."""
    def get(name):
        if name in JOINTS:
            return float(state[JOINTS[name]])
        a, b = GROUPS[name]
        return [float(v) for v in state[a:b]]
    return get


def _check_clause(clause, where):
    if not isinstance(clause, dict):
        raise RegistryError(f'{where}: a clause must be a mapping, got {clause!r}')
    if 'joint' in clause:
        if clause['joint'] not in JOINTS:
            raise RegistryError(f"{where}: unknown joint {clause['joint']!r}")
        if 'min' not in clause and 'max' not in clause:
            raise RegistryError(f'{where}: a joint clause needs min and/or max')
        extra = set(clause) - {'joint', 'min', 'max'}
    elif 'joints' in clause:
        group = clause['joints']
        if group not in GROUPS:
            raise RegistryError(f'{where}: unknown joint group {group!r}')
        a, b = GROUPS[group]
        near = clause.get('near')
        if not isinstance(near, list) or len(near) != b - a or 'tol' not in clause:
            raise RegistryError(f'{where}: {group} needs near: [{b - a} values] and tol')
        extra = set(clause) - {'joints', 'near', 'tol'}
    else:
        raise RegistryError(f'{where}: a clause needs joint: or joints:')
    if extra:
        raise RegistryError(f'{where}: unknown keys {sorted(extra)}')


def check(clauses, get):
    """True when every clause holds (an empty list holds)."""
    for c in clauses or ():
        if 'joint' in c:
            v = get(c['joint'])
            if 'min' in c and v < c['min']:
                return False
            if 'max' in c and v > c['max']:
                return False
        else:
            vals = get(c['joints'])
            if max(abs(v - t) for v, t in zip(vals, c['near'])) > c['tol']:
                return False
    return True


class DoneTracker:
    """Decides when a running task is finished (its ``done`` block).

    ``after`` clauses latch once seen true; then every ``all`` clause must hold
    continuously for ``hold_s``. A task without a ``done`` block never finishes.
    """

    def __init__(self, task):
        self.task = task
        done = task.get('done') or {}
        self._all = done.get('all') or []
        self._after = done.get('after') or []
        self._hold_s = float(done.get('hold_s', 0.0))
        self.reset()

    def reset(self):
        self._latched = [False] * len(self._after)
        self._since = None

    def update(self, now, get):
        if not self._all or get is None:
            self._since = None
            return False
        for i, c in enumerate(self._after):
            if not self._latched[i] and check([c], get):
                self._latched[i] = True
        if not (all(self._latched) and check(self._all, get)):
            self._since = None
            return False
        if self._since is None:
            self._since = now
        return now - self._since >= self._hold_s


class TaskRegistry:
    def __init__(self, data, source='<data>'):
        self.source = str(source)
        if not isinstance(data, dict):
            raise RegistryError(f'{self.source}: expected a mapping with poses: and tasks:')
        self.poses = data.get('poses') or {}
        for name, pose in self.poses.items():
            for part, n in _POSE_SIZES.items():
                if len(pose.get(part) or ()) != n:
                    raise RegistryError(f'pose {name}: {part} needs {n} values')
            if not isinstance(pose.get('back'), (int, float)):
                raise RegistryError(f'pose {name}: back (m) is required')
        self.tasks = {}
        self._by_text = {}
        for tid, raw in (data.get('tasks') or {}).items():
            task = self._validate(tid, raw)
            if task['text'] in self._by_text:
                other = self._by_text[task['text']]['id']
                raise RegistryError(f'task {tid}: same text as task {other}')
            self.tasks[tid] = task
            self._by_text[task['text']] = task

    def _validate(self, tid, raw):
        if not isinstance(raw, dict):
            raise RegistryError(f'task {tid}: expected a mapping')
        extra = set(raw) - _TASK_KEYS
        if extra:
            raise RegistryError(f'task {tid}: unknown keys {sorted(extra)}')
        text = raw.get('text')
        if not isinstance(text, str) or not text.strip():
            raise RegistryError(f'task {tid}: text (the training string) is required')
        if raw.get('start_pose') not in self.poses:
            raise RegistryError(f"task {tid}: unknown start_pose {raw.get('start_pose')!r}")
        max_s = raw.get('max_s')
        if not isinstance(max_s, (int, float)) or max_s <= 0:
            raise RegistryError(f'task {tid}: max_s must be a positive number')
        attempts = raw.get('attempts', 1)
        if not isinstance(attempts, int) or attempts < 1:
            raise RegistryError(f'task {tid}: attempts must be an integer >= 1')
        done = raw.get('done') or {}
        if set(done) - {'hold_s', 'all', 'after'}:
            raise RegistryError(f'task {tid}: done takes hold_s, all, after')
        for key in ('all', 'after'):
            for c in done.get(key) or ():
                _check_clause(c, f'task {tid} done.{key}')
        if done and not done.get('all'):
            raise RegistryError(f'task {tid}: done needs an all: list')
        for c in raw.get('accept_on_timeout') or ():
            _check_clause(c, f'task {tid} accept_on_timeout')
        task = dict(raw)
        task.update(id=tid, max_s=float(max_s), attempts=attempts,
                    holds_object=bool(raw.get('holds_object', False)),
                    accept_on_timeout=list(raw.get('accept_on_timeout') or ()))
        return task

    def by_id(self, tid):
        return self.tasks.get(tid)

    def by_text(self, text):
        return self._by_text.get(text)

    def pose(self, name):
        return self.poses[name]

    def start_pose(self, task):
        return self.poses[task['start_pose']]


def load(path=None):
    """Load and validate tasks.yaml; raises RegistryError / OSError on a bad file."""
    path = Path(path) if path else default_path()
    with open(path) as f:
        return TaskRegistry(yaml.safe_load(f), source=path)
