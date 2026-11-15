"""The recording context: the facts an operator sets before recording episodes.

data_recorder owns it (persisted next to the sessions, republished latched on
/alfie/recording/context); the episode manager changes it by publishing a JSON
update on /alfie/recording/set_context. Every episode recorded gets a copy in
its bag folder (episode.json), which rosbag_to_groot.py writes into the
session's meta/episodes.jsonl.

Stdlib only; the task registry is passed in.
"""

# What produced an episode. Training folders are task family x source.
SOURCES = ('demo', 'correction', 'scripted_recovery', 'autonomous_success')
# Review verdict. 'bad' episodes stay in the raw session; the export decides.
QUALITIES = ('ok', 'marginal', 'bad')
# Why the operator took over, tagged on corrections during review.
FAILURE_MODES = ('miss', 'bleed_over', 'collision', 'stuck', 'other')

CONTEXT_FILE = '.recording_context.json'
SET_CONTEXT_TOPIC = '/alfie/recording/set_context'
CONTEXT_TOPIC = '/alfie/recording/context'

DEFAULT_CONTEXT = {
    'task_id': '',        # registry task ID; '' = not chosen (episode needs labelling)
    'source': 'demo',     # a VR takeover of a GR00T run is always a correction
    'round': 1,           # correction round, used for every source but demo
    'operator': '',
    'objects': [],        # object IDs in the scene; [] = the task's own objects
    'surface': '',        # what the object stands on (floor, table, ...)
}


def merge_context(current, update, task_ids):
    """Apply a partial ``update`` to ``current``.

    Returns (context, error): the new context and None, or the unchanged
    context and why the update was refused. Unknown keys are ignored.
    """
    new = dict(DEFAULT_CONTEXT)
    new.update({k: v for k, v in (current or {}).items() if k in DEFAULT_CONTEXT})
    if not isinstance(update, dict):
        return new, 'context update must be a JSON object'
    out = dict(new)
    try:
        if 'task_id' in update:
            tid = str(update['task_id'] or '')
            if tid and tid not in task_ids:
                return new, f'task "{tid}" is not in the task registry'
            out['task_id'] = tid
        if 'source' in update:
            if update['source'] not in SOURCES:
                return new, f'source must be one of {", ".join(SOURCES)}'
            out['source'] = update['source']
        if 'round' in update:
            rnd = int(update['round'])
            if rnd < 1:
                return new, 'round must be 1 or more'
            out['round'] = rnd
        for key in ('operator', 'surface'):
            if key in update:
                out[key] = str(update[key] or '').strip()
        if 'objects' in update:
            objects = update['objects'] or []
            if isinstance(objects, str):
                objects = objects.replace(',', ' ').split()
            out['objects'] = [str(o) for o in objects]
    except (TypeError, ValueError) as exc:
        return new, f'bad context value: {exc}'
    return out, None
