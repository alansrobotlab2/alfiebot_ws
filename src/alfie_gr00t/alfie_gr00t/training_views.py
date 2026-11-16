"""Rules for turning closed recording sessions into a training build.

A build (scripts/build_views.py) is one immutable folder of LeRobot datasets:
a train view per group of episodes (task family x source unless told
otherwise), one held-out view, the sampling weights
(mix.json) and a manifest of exactly which episodes went where. This module
holds the decisions, with no file access, so both machines and the tests share
them. Stdlib only.

Held-out rule: within each task x source group, episodes rated ``ok`` are
ranked by a hash of (salt, episode_uid) and the lowest are held out:
``fraction`` of the group rounded up, at least ``minimum``, never the whole group.
A group smaller than ``min_group`` holds none out.
An episode held out by the previous build stays held out, so no later
checkpoint trains on an earlier build's test episodes.
"""

import datetime
import hashlib
import math
import re
from pathlib import Path

DEFAULT_SALT = 'alfie-heldout-1'
DEFAULT_FRACTION = 0.1
DEFAULT_MINIMUM = 1
# A group with fewer usable episodes than this holds none out: the first few
# corrections of a task are worth more in training than as a test set of one.
DEFAULT_MIN_GROUP = 5
HELDOUT_VIEW = 'heldout'
# Episode fields that decide which train view an episode goes into.
DEFAULT_GROUP_BY = ('family', 'source')
BUILD_RE = re.compile(r'^(\d{4}-\d{2}-\d{2})_b(\d{2,})$')
# Which registry string a view's episodes are labelled with.
WORDINGS = ('text', 'instruction')


def uid_hash(salt: str, uid: str) -> str:
    return hashlib.sha256(f'{salt}:{uid}'.encode()).hexdigest()


def stratum(ep: dict) -> str:
    return f'{ep["task_id"]}/{ep["source"]}'


def view_name(ep: dict, group_by=DEFAULT_GROUP_BY) -> str:
    """Train view of an episode: its values of the ``group_by`` fields, joined by __.

    Any episode field works (family, source, task_id, round, object, hw, ...).
    A missing or empty value reads ``none``.
    """
    parts = []
    for field in group_by:
        value = ep.get(field)
        text = 'none' if value in (None, '', []) else str(value)
        parts.append(re.sub(r'[^A-Za-z0-9.-]+', '_', text).strip('_') or 'none')
    name = '__'.join(parts)
    if not name or name == HELDOUT_VIEW:
        raise ValueError(f'{name!r} cannot be a train view name')
    return name


def select_heldout(episodes, fraction=DEFAULT_FRACTION, minimum=DEFAULT_MINIMUM,
                   salt=DEFAULT_SALT, previous=(), min_group=DEFAULT_MIN_GROUP):
    """Choose the held-out episodes.

    episodes: dicts with episode_uid, task_id, source, quality (the usable
    ones: bad episodes already removed). previous: episode_uids held out by
    the build this one extends.

    Returns (uids, counts): the held-out set, and per task/source group
    {'episodes', 'eligible', 'target', 'carried', 'heldout'}.
    """
    if not 0 <= fraction < 1:
        raise ValueError('held-out fraction must be in [0, 1)')
    previous = set(previous)
    groups = {}
    for ep in episodes:
        groups.setdefault(stratum(ep), []).append(ep)
    heldout, counts = set(), {}
    for name in sorted(groups):
        eps = groups[name]
        eligible = [ep['episode_uid'] for ep in eps if ep.get('quality') == 'ok']
        target = 0
        if fraction > 0 and len(eligible) >= min_group:
            # Rounded up; at least `minimum`, but always leave one to train on.
            target = max(minimum, math.ceil(fraction * len(eligible) - 1e-9))
            target = max(0, min(target, len(eligible) - 1))
        carried = {ep['episode_uid'] for ep in eps if ep['episode_uid'] in previous}
        chosen = set(carried)
        for uid in sorted(eligible, key=lambda u: uid_hash(salt, u)):
            if len(chosen) >= target:
                break
            chosen.add(uid)
        heldout |= chosen
        counts[name] = {'episodes': len(eps), 'eligible': len(eligible), 'target': target,
                        'carried': len(carried), 'heldout': len(chosen)}
    return heldout, counts


def mix_ratios(frames, overrides=None):
    """Sampling share of each train view (sums to 1).

    frames: {view: frame count}. Without overrides every view's share is its
    share of frames, which is what pooling the folders would sample. overrides
    ({view: share}) fix some shares; the rest split what is left by frames.
    """
    overrides = dict(overrides or {})
    unknown = sorted(set(overrides) - set(frames))
    if unknown:
        raise ValueError(f'no such view: {", ".join(unknown)}')
    fixed = sum(overrides.values())
    rest = {v: n for v, n in frames.items() if v not in overrides}
    if any(share <= 0 for share in overrides.values()) or fixed > 1 + 1e-9:
        raise ValueError('view shares must be positive and sum to at most 1')
    if rest and fixed >= 1:
        raise ValueError('the given shares leave nothing for the other views')
    if not rest and abs(fixed - 1) > 1e-6:
        raise ValueError('shares for every view must sum to 1')
    total = sum(rest.values())
    out = {}
    for view in frames:
        if view in overrides:
            out[view] = float(overrides[view])
        else:
            out[view] = (1 - fixed) * rest[view] / total if total else 0.0
    return out


def next_build_name(views_root, day=None) -> str:
    """The next unused YYYY-MM-DD_bNN under ``views_root``."""
    date = (day or datetime.date.today()).isoformat()
    used = [0]
    root = Path(views_root)
    if root.is_dir():
        for d in root.iterdir():
            # A build still being written is .<name>.tmp; its number is taken.
            m = BUILD_RE.match(d.name.lstrip('.').removesuffix('.tmp'))
            if m and m.group(1) == date:
                used.append(int(m.group(2)))
    return f'{date}_b{max(used) + 1:02d}'


def latest_build(views_root):
    """Name of the newest build under ``views_root``, or None."""
    root = Path(views_root)
    if not root.is_dir():
        return None
    names = sorted(d.name for d in root.iterdir()
                   if BUILD_RE.match(d.name) and (d / 'manifest.json').is_file())
    return names[-1] if names else None
