"""
skills — what Alfie can be asked to do, as data.

A skill is a named, described capability the LLM starts with the ``skill`` tool.
Three kinds:

  * ``task``      one trained GR00T task. Never written by hand: every entry of
                  the task registry (alfie_gr00t config/tasks.yaml) that has a
                  ``description`` becomes one, under its task id.
  * ``sequence``  task steps in order, optionally repeated (``clean_room``).
                  Runs in ``Body.run_skill`` on a worker thread; the LLM only
                  picks it and its arguments.
  * ``prompt``    instructions handed back to the LLM as the tool result, to be
                  carried out with its other tools. Moves nothing by itself.

Skill files are YAML, one per skill, in this directory (or ``skills_dir``):

    name: clean_room            # what the LLM calls
    title: clean the room       # how it is spoken / logged (default: the name)
    kind: sequence              # sequence | prompt
    description: one line saying what it does and when it applies
    triggers: [clean, tidy]     # extra words that should surface it (optional)
    params:                     # optional; integer | number | string | boolean
      rounds: {type: integer, default: 3, min: 1, max: 10}
    repeat: rounds              # sequence: a param name or a number (default 1)
    steps:                      # sequence: task ids, or {task:, failed:}
      - pick_up_soda_can
    say:                        # sequence: spoken lines; {count} rounds finished,
      progress: "That's {count}."   # {round}, {rounds}, {step}
      done: "Done."
      failed: "I couldn't finish {step}, so I stopped."
    instructions: |             # prompt: text returned to the LLM
      ...

Skills are not listed in the system prompt (it is pinned in the LLM server and
would grow with every skill). The per-turn prefetch injects the few that match
the user's words (``search``), and ``skill_list`` covers the rest. Files are
re-read when they change, so a new skill needs no restart.
"""
import difflib
import threading
from pathlib import Path

import yaml

from alfie_agent.prefetch import stems
from alfie_gr00t import task_registry

SKILL_DIR = Path(__file__).resolve().parent
_KINDS = ("sequence", "prompt")
_TYPES = {"integer": int, "number": (int, float), "string": str, "boolean": bool}
_ABBR = {"integer": "int", "number": "num", "string": "str", "boolean": "bool"}
_KEYS = {"name", "title", "kind", "description", "triggers", "params", "repeat",
         "steps", "say", "instructions"}


class SkillError(ValueError):
    """A skill file is malformed."""


def _params(name, raw):
    out = {}
    for pname, spec in (raw or {}).items():
        spec = dict(spec or {})
        if spec.get("type") not in _TYPES:
            raise SkillError(f"skill {name}: param {pname} needs type "
                             f"{' | '.join(_TYPES)}")
        out[pname] = spec
    return out


def _build(raw, tasks, source):
    """Validate one skill file's mapping against the task registry."""
    if not isinstance(raw, dict) or not raw.get("name"):
        raise SkillError(f"{source}: a skill needs a name")
    name = str(raw["name"])
    if set(raw) - _KEYS:
        raise SkillError(f"skill {name}: unknown keys {sorted(set(raw) - _KEYS)}")
    if raw.get("kind") not in _KINDS:
        raise SkillError(f"skill {name}: kind must be {' or '.join(_KINDS)}")
    if not str(raw.get("description") or "").strip():
        raise SkillError(f"skill {name}: description is required")
    skill = {"name": name, "kind": raw["kind"],
             "title": str(raw.get("title") or name.replace("_", " ")),
             "description": " ".join(str(raw["description"]).split()),
             "triggers": [str(t) for t in raw.get("triggers") or ()],
             "params": _params(name, raw.get("params"))}
    if skill["kind"] == "prompt":
        if not str(raw.get("instructions") or "").strip():
            raise SkillError(f"skill {name}: a prompt skill needs instructions")
        skill["instructions"] = str(raw["instructions"]).strip()
        return skill
    steps = []
    for entry in raw.get("steps") or ():
        entry = {"task": entry} if isinstance(entry, str) else dict(entry or {})
        if tasks.by_id(entry.get("task")) is None:
            raise SkillError(f"skill {name}: unknown task {entry.get('task')!r}")
        steps.append(entry)
    if not steps:
        raise SkillError(f"skill {name}: a sequence needs steps")
    repeat = raw.get("repeat", 1)
    if isinstance(repeat, str):
        if skill["params"].get(repeat, {}).get("type") != "integer":
            raise SkillError(f"skill {name}: repeat names no integer param {repeat!r}")
    elif not isinstance(repeat, int) or repeat < 1:
        raise SkillError(f"skill {name}: repeat must be a param name or a number >= 1")
    skill.update(steps=steps, repeat=repeat, say=dict(raw.get("say") or {}))
    return skill


def _task_skill(task):
    """The skill generated for one registered GR00T task."""
    return {"name": task["id"], "kind": "sequence",
            "title": task["id"].replace("_", " "),
            "description": " ".join(str(task["description"]).split()),
            "triggers": [], "params": {}, "steps": [{"task": task["id"]}],
            "repeat": 1, "say": {}}


def signature(skill):
    """``clean_room(rounds?:int)`` — '?' marks an argument with a default."""
    parts = [f"{p}{'?' if 'default' in s else ''}:{_ABBR[s['type']]}"
             for p, s in skill["params"].items()]
    return f"{skill['name']}({', '.join(parts)})"


def line(skill):
    return f"{signature(skill)} — {skill['description']}"


class SkillRegistry:
    """The skills on disk plus the task registry behind them.

    ``log(text)`` receives load problems; a file that fails to load is skipped
    (the others still work) and, on a reload, the last good set is kept.
    """

    def __init__(self, skill_dir=None, tasks_path=None, log=None):
        self.skill_dir = Path(skill_dir) if skill_dir else SKILL_DIR
        self.tasks_path = Path(tasks_path) if tasks_path else task_registry.default_path()
        self._log = log or (lambda text: None)
        self._lock = threading.Lock()
        self._stamp = None
        self._skills = {}
        self._stems = {}
        self._tasks = None
        self.refresh()

    def _files(self):
        return sorted(self.skill_dir.glob("*.yaml"))

    def _signature(self):
        out = []
        for path in [self.tasks_path] + self._files():
            try:
                out.append((str(path), path.stat().st_mtime_ns))
            except OSError:
                out.append((str(path), None))
        return out

    def refresh(self):
        """Re-read the files if any changed since the last load."""
        stamp = self._signature()
        with self._lock:
            if stamp == self._stamp:
                return
            self._stamp = stamp
        try:
            tasks = task_registry.load(self.tasks_path)
        except (OSError, ValueError, yaml.YAMLError) as e:
            self._log(f"task registry not loaded ({e}); keeping the previous skills")
            return
        skills = {}
        for path in self._files():
            try:
                skill = _build(yaml.safe_load(path.read_text()), tasks, path.name)
            except (OSError, ValueError, yaml.YAMLError) as e:
                self._log(f"skill file {path.name} skipped: {e}")
                continue
            skills[skill["name"]] = skill
        for task in tasks.tasks.values():
            if task.get("description") and task["id"] not in skills:
                skills[task["id"]] = _task_skill(task)
        index = {name: stems(" ".join([name.replace("_", " "), s["title"],
                                       s["description"]] + s["triggers"]))
                 for name, s in skills.items()}
        with self._lock:
            self._tasks, self._skills, self._stems = tasks, skills, index

    def names(self):
        self.refresh()
        return sorted(self._skills)

    def get(self, name):
        self.refresh()
        return self._skills.get(name)

    def search(self, words, limit=3):
        """Skills sharing stemmed words with ``words`` (a set of stems), best first."""
        self.refresh()
        words = set(words)
        # Most shared words first; on a tie the more specific (shorter) skill.
        hits = sorted((-len(words & kw), len(kw), name)
                      for name, kw in self._stems.items() if words & kw)
        return [self._skills[name] for _, _, name in hits[:limit]]

    def listing(self, query="", limit=20):
        """For the ``skill_list`` tool: every skill, or those matching ``query``."""
        self.refresh()
        if (query or "").strip():
            found = self.search(stems(query), limit=limit)
        else:
            found = [self._skills[n] for n in sorted(self._skills)][:limit]
        return {"skills": [line(s) for s in found], "total": len(self._skills)}

    def resolve(self, name, args=None):
        """Turn a ``skill`` call into something to run.

        Returns ``{"plan": ...}`` for Body.run_skill, ``{"instructions": ...}``
        for a prompt skill, or ``{"error": ...}``.
        """
        self.refresh()
        skill = self._skills.get(str(name or "").strip())
        if skill is None:
            close = difflib.get_close_matches(str(name or ""), list(self._skills), n=3,
                                              cutoff=0.5)
            return {"error": f"no skill named {name!r}; I can only run my listed "
                             "skills (skill_list shows them)",
                    **({"did_you_mean": close} if close else {})}
        args = dict(args or {})
        unknown = set(args) - set(skill["params"])
        if unknown:
            return {"error": f"{skill['name']} takes "
                             f"{sorted(skill['params']) or 'no arguments'}, "
                             f"not {sorted(unknown)}"}
        values = {}
        for pname, spec in skill["params"].items():
            if args.get(pname) is None:
                if "default" not in spec:
                    return {"error": f"{skill['name']} needs {pname}"}
                values[pname] = spec["default"]
                continue
            value = args[pname]
            if spec["type"] == "integer" and isinstance(value, (float, str)):
                try:
                    as_float = float(value)
                except ValueError:
                    as_float = None
                if as_float is not None and as_float == int(as_float):
                    value = int(as_float)
            if isinstance(value, bool) != (spec["type"] == "boolean") or \
                    not isinstance(value, _TYPES[spec["type"]]):
                return {"error": f"{pname} must be {spec['type']}"}
            lo, hi = spec.get("min"), spec.get("max")
            if (lo is not None and value < lo) or (hi is not None and value > hi):
                return {"error": f"{pname} must be {lo} to {hi}"}
            values[pname] = value
        if skill["kind"] == "prompt":
            return {"instructions": skill["instructions"], "args": values}
        repeat = skill["repeat"]
        steps = []
        for entry in skill["steps"]:
            task = self._tasks.by_id(entry["task"])
            step = dict(task, pose=self._tasks.start_pose(task))
            if entry.get("failed"):
                step["failed"] = entry["failed"]
            steps.append(step)
        return {"plan": {"name": skill["name"], "title": skill["title"],
                         "rounds": values[repeat] if isinstance(repeat, str) else repeat,
                         "steps": steps, "say": skill["say"]}}
