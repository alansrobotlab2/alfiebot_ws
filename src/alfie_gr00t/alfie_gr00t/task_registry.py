"""The task registry (config/tasks.yaml, objects.yaml, hardware.yaml).

Kept at the package root (not under core/) so the agent and scripts can import
it without the ZMQ/msgpack client stack. Needs only PyYAML.

One entry per task ID, used by recording and inference alike. A task carries
what an episode of it is (family, structured instruction fields, start and end
state), its exact training string, start pose, time cap and the conditions
under which it is finished. Conditions are clauses over the measured
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
              'holds_object', 'done', 'accept_on_timeout', 'close',
              'family', 'status', 'verb', 'object', 'destination', 'start_state',
              'end_state', 'instruction', 'paraphrases', 'former_ids', 'includes'}

STATUSES = ('proposed', 'collecting', 'active', 'retired')
# Statuses the episode manager offers for recording.
RECORDABLE = ('collecting', 'active')
# Structured task fields copied into every episode row (meta/episodes.jsonl).
EPISODE_TASK_FIELDS = ('family', 'verb', 'object', 'destination', 'start_state', 'end_state')


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


def _sibling(name, tasks_path=None):
    """objects.yaml / hardware.yaml live beside tasks.yaml."""
    return (Path(tasks_path) if tasks_path else default_path()).parent / name


def load_objects(tasks_path=None):
    """Object ID -> {name, aliases, notes} from objects.yaml ({} if the file is absent)."""
    path = _sibling('objects.yaml', tasks_path)
    if not path.is_file():
        return {}
    with open(path) as f:
        objects = (yaml.safe_load(f) or {}).get('objects') or {}
    names = {}
    for oid, obj in objects.items():
        name = (obj or {}).get('name')
        if not isinstance(name, str) or not name.strip():
            raise RegistryError(f'{path}: object {oid}: name is required')
        if name in names:
            raise RegistryError(f'{path}: objects {names[name]} and {oid} share the name {name!r}')
        names[name] = oid
    return objects


def load_hardware(tasks_path=None):
    """hardware.yaml as {'current': 'hwN', 'versions': {...}}."""
    path = _sibling('hardware.yaml', tasks_path)
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    versions = data.get('versions') or {}
    if data.get('current') not in versions:
        raise RegistryError(f"{path}: current: {data.get('current')!r} is not under versions")
    return {'current': data['current'], 'versions': versions}


def current_hw(tasks_path=None):
    """The hardware version recordings are tagged with, e.g. 'hw2'."""
    return load_hardware(tasks_path)['current']


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


def first_held(clauses, hold_s, getters, dt):
    """Index of the first frame of the first stretch where every clause holds
    for at least ``hold_s``; None if there is none.

    ``getters`` is one ``get(name)`` per frame, ``dt`` the frame period.
    """
    need = max(1, round(hold_s / dt))
    start = None
    for i, get in enumerate(getters):
        if check(clauses, get):
            if start is None:
                start = i
            if i - start + 1 >= need:
                return start
        else:
            start = None
    return None


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


def object_name(oid, objects=None):
    """The words an object ID stands for in an instruction (objects.yaml
    `name`; an ID that is not listed reads as itself with spaces)."""
    if oid is None:
        return None
    entry = (objects or {}).get(oid) or {}
    return entry.get('name') or str(oid).replace('_', ' ')


def fill_template(template, obj, destination, objects=None):
    """An instruction from a verb template and object / destination IDs."""
    return template.format(object=object_name(obj, objects),
                           destination=object_name(destination, objects))


class TaskRegistry:
    def __init__(self, data, source='<data>', objects=None):
        self.source = str(source)
        self.objects = objects or {}
        if not isinstance(data, dict):
            raise RegistryError(f'{self.source}: expected a mapping with poses: and tasks:')
        self.poses = data.get('poses') or {}
        self.templates = data.get('templates') or {}
        for name, pose in self.poses.items():
            for part, n in _POSE_SIZES.items():
                if len(pose.get(part) or ()) != n:
                    raise RegistryError(f'pose {name}: {part} needs {n} values')
            if not isinstance(pose.get('back'), (int, float)):
                raise RegistryError(f'pose {name}: back (m) is required')
        self.tasks = {}
        self._by_text = {}
        self._former = {}       # an ID a task used to have -> its current ID
        for tid, raw in (data.get('tasks') or {}).items():
            task = self._validate(tid, raw)
            if task['text'] in self._by_text:
                other = self._by_text[task['text']]['id']
                raise RegistryError(f'task {tid}: same text as task {other}')
            self.tasks[tid] = task
            self._by_text[task['text']] = task
            for old in task.get('former_ids') or ():
                if old in self._former:
                    raise RegistryError(f'task {tid}: former ID {old!r} also belongs to '
                                        f'task {self._former[old]}')
                self._former[old] = tid
        clash = set(self._former) & set(self.tasks)
        if clash:
            raise RegistryError(f'former IDs reused as task IDs: {sorted(clash)}')

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
        close = raw.get('close') or {}
        if set(close) - {'hold_s', 'all', 'keep_s'}:
            raise RegistryError(f'task {tid}: close takes hold_s, all, keep_s')
        keep_s = close.get('keep_s')
        if keep_s is not None and not (isinstance(keep_s, (int, float))
                                       and keep_s >= close.get('hold_s', 0.0)):
            raise RegistryError(f'task {tid}: close.keep_s must be a number >= hold_s')
        if close and not close.get('all'):
            raise RegistryError(f'task {tid}: close needs an all: list')
        for c in close.get('all') or ():
            _check_clause(c, f'task {tid} close.all')
        for c in raw.get('accept_on_timeout') or ():
            _check_clause(c, f'task {tid} accept_on_timeout')
        status = raw.get('status', 'active')
        if status not in STATUSES:
            raise RegistryError(f'task {tid}: status must be one of {", ".join(STATUSES)}')
        verb = raw.get('verb')
        if verb is not None:
            if verb not in self.templates:
                raise RegistryError(f'task {tid}: no template for verb {verb!r}')
            try:
                want = fill_template(self.templates[verb], raw.get('object'),
                                     raw.get('destination'), self.objects)
            except (KeyError, IndexError) as exc:
                raise RegistryError(f'task {tid}: template for {verb!r}: {exc}') from exc
            if raw.get('instruction') != want:
                raise RegistryError(
                    f'task {tid}: instruction {raw.get("instruction")!r} is not the '
                    f'{verb} template filled in ({want!r})')
        for key in ('paraphrases', 'former_ids', 'includes'):
            if not isinstance(raw.get(key) or [], list):
                raise RegistryError(f'task {tid}: {key} must be a list')
        task = dict(raw)
        task.update(status=status)
        task.update(id=tid, max_s=float(max_s), attempts=attempts,
                    holds_object=bool(raw.get('holds_object', False)),
                    accept_on_timeout=list(raw.get('accept_on_timeout') or ()))
        return task

    def by_id(self, tid):
        """The task with this ID, or the one that used to have it (former_ids)."""
        return self.tasks.get(tid) or self.tasks.get(self._former.get(tid))

    def by_text(self, text):
        return self._by_text.get(text)

    def recordable(self):
        """Tasks offered for recording (status collecting or active), in file order."""
        return [t for t in self.tasks.values() if t['status'] in RECORDABLE]

    def episode_fields(self, tid):
        """The task facts stored with every episode of ``tid`` (None when absent)."""
        task = self.by_id(tid)          # a former ID is stored as the current one
        fields = {'task_id': task['id']}
        fields.update({k: task.get(k) for k in EPISODE_TASK_FIELDS})
        return fields

    def pose(self, name):
        return self.poses[name]

    def start_pose(self, task):
        return self.poses[task['start_pose']]


def load(path=None):
    """Load and validate tasks.yaml; raises RegistryError / OSError on a bad file."""
    path = Path(path) if path else default_path()
    with open(path) as f:
        return TaskRegistry(yaml.safe_load(f), source=path, objects=load_objects(path))
