from pathlib import Path

import pytest

from alfie_gr00t import task_registry as tr

TASKS_YAML = Path(__file__).resolve().parents[1] / 'config' / 'tasks.yaml'


def state(gripper=0.0, back=0.34, right_arm=(0.0, 0.12, -1.48, -0.14, 0.0)):
    s = [0.0] * 22
    s[6], s[18] = back, gripper
    s[13:18] = right_arm
    return tr.state_getter(s)


@pytest.fixture
def reg():
    return tr.load(TASKS_YAML)


def test_loads_shipped_tasks(reg):
    assert reg.by_text('find the can and pick it up')['id'] == 'pick_can'
    assert reg.by_id('trash_can')['text'] == 'put the can in the trash can'
    assert reg.by_text('do a backflip') is None
    pick = reg.by_id('pick_can')
    assert pick['attempts'] == 2 and pick['holds_object'] and pick['max_s'] == 90.0
    assert reg.by_id('trash_can')['attempts'] == 1
    assert reg.start_pose(pick)['back'] == 0.34


def test_state_getter_names():
    s = list(range(22))
    get = tr.state_getter(s)
    assert get('back') == 6 and get('left_gripper') == 12 and get('right_gripper') == 18
    assert get('right_shoulder_pitch') == 14 and get('head_roll') == 21
    assert get('left_arm') == [7, 8, 9, 10, 11] and get('head') == [19, 20, 21]


def test_pick_done_needs_hold(reg):
    t = tr.DoneTracker(reg.by_id('pick_can'))
    assert not t.update(0.0, state(gripper=0.0))
    assert not t.update(1.0, state(gripper=0.40))          # met, hold starts
    assert not t.update(1.4, state(gripper=0.40))
    assert t.update(1.5, state(gripper=0.40))
    assert not t.update(2.0, state(gripper=0.40, back=0.2))  # back dropped: restart
    assert not t.update(2.1, state(gripper=0.40))
    assert t.update(2.6, state(gripper=0.40))
    assert not t.update(3.0, state(gripper=0.78))          # closed on nothing


def test_trash_done_needs_can_seen_first(reg):
    t = tr.DoneTracker(reg.by_id('trash_can'))
    # Open gripper at the tucked pose, but no can was ever held: not done.
    assert not t.update(0.0, state(gripper=0.0))
    assert not t.update(5.0, state(gripper=0.0))
    assert not t.update(6.0, state(gripper=0.40))          # can seen
    assert not t.update(7.0, state(gripper=0.0, right_arm=(0.5, 0.12, -1.0, -0.14, 0.0)))
    assert not t.update(8.0, state(gripper=0.0))
    assert t.update(8.5, state(gripper=0.0))
    t.reset()
    assert not t.update(9.0, state(gripper=0.0))
    assert not t.update(10.0, state(gripper=0.0))


def test_no_state_or_no_done_block(reg):
    t = tr.DoneTracker(reg.by_id('pick_can'))
    assert not t.update(0.0, None)
    assert not tr.DoneTracker({'id': 'x'}).update(0.0, state(gripper=0.4))


def test_accept_on_timeout(reg):
    assert tr.check(reg.by_id('pick_can')['accept_on_timeout'], state(gripper=0.4))
    assert not tr.check(reg.by_id('pick_can')['accept_on_timeout'], state(gripper=0.0))
    assert tr.check(reg.by_id('trash_can')['accept_on_timeout'], state(gripper=0.12))
    assert not tr.check(reg.by_id('trash_can')['accept_on_timeout'], state(gripper=0.4))


POSE = {'p': {'left_arm': [0] * 6, 'right_arm': [0] * 6, 'head': [0] * 3, 'back': 0.3}}


def _task(**over):
    t = {'text': 'do it', 'start_pose': 'p', 'max_s': 10}
    t.update(over)
    return {'poses': POSE, 'tasks': {'t': t}}


@pytest.mark.parametrize('bad', [
    _task(text=''),
    _task(start_pose='nope'),
    _task(max_s=0),
    _task(attempts=0),
    _task(colour='red'),
    _task(done={'all': [{'joint': 'elbow', 'min': 0}]}),
    _task(done={'all': [{'joint': 'back'}]}),
    _task(done={'all': [{'joints': 'right_arm', 'near': [0, 0], 'tol': 0.1}]}),
    _task(done={'after': [{'joint': 'back', 'min': 0}]}),
    _task(accept_on_timeout=[{'joint': 'back', 'min': 0, 'near': 1}]),
    {'poses': {'p': {'left_arm': [0] * 5, 'right_arm': [0] * 6, 'head': [0] * 3, 'back': 0}},
     'tasks': {}},
])
def test_rejects_bad_entries(bad):
    with pytest.raises(tr.RegistryError):
        tr.TaskRegistry(bad)


def test_rejects_duplicate_text():
    data = _task()
    data['tasks']['u'] = dict(data['tasks']['t'])
    with pytest.raises(tr.RegistryError):
        tr.TaskRegistry(data)
