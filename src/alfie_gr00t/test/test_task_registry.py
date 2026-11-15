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
    assert reg.by_text('find the can and pick it up')['id'] == 'pick_up_soda_can'
    assert reg.by_id('put_soda_can_in_trash')['text'] == 'put the can in the trash can'
    assert reg.by_text('do a backflip') is None
    pick = reg.by_id('pick_up_soda_can')
    assert pick['attempts'] == 2 and pick['holds_object'] and pick['max_s'] == 90.0
    assert reg.by_id('put_soda_can_in_trash')['attempts'] == 1
    assert reg.start_pose(pick)['back'] == 0.34


def test_state_getter_names():
    s = list(range(22))
    get = tr.state_getter(s)
    assert get('back') == 6 and get('left_gripper') == 12 and get('right_gripper') == 18
    assert get('right_shoulder_pitch') == 14 and get('head_roll') == 21
    assert get('left_arm') == [7, 8, 9, 10, 11] and get('head') == [19, 20, 21]


def test_pick_done_needs_hold(reg):
    t = tr.DoneTracker(reg.by_id('pick_up_soda_can'))
    assert not t.update(0.0, state(gripper=0.0))
    assert not t.update(1.0, state(gripper=0.40))          # met, hold starts
    assert not t.update(1.4, state(gripper=0.40))
    assert t.update(1.5, state(gripper=0.40))
    assert not t.update(2.0, state(gripper=0.40, back=0.2))  # back dropped: restart
    assert not t.update(2.1, state(gripper=0.40))
    assert t.update(2.6, state(gripper=0.40))
    assert not t.update(3.0, state(gripper=0.78))          # closed on nothing


def test_trash_done_needs_can_seen_first(reg):
    t = tr.DoneTracker(reg.by_id('put_soda_can_in_trash'))
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
    t = tr.DoneTracker(reg.by_id('pick_up_soda_can'))
    assert not t.update(0.0, None)
    assert not tr.DoneTracker({'id': 'x'}).update(0.0, state(gripper=0.4))


def test_accept_on_timeout(reg):
    assert tr.check(reg.by_id('pick_up_soda_can')['accept_on_timeout'], state(gripper=0.4))
    assert not tr.check(reg.by_id('pick_up_soda_can')['accept_on_timeout'], state(gripper=0.0))
    assert tr.check(reg.by_id('put_soda_can_in_trash')['accept_on_timeout'], state(gripper=0.12))
    assert not tr.check(reg.by_id('put_soda_can_in_trash')['accept_on_timeout'], state(gripper=0.4))


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


def test_shipped_tasks_carry_record_fields(reg):
    objects = tr.load_objects(TASKS_YAML)
    for tid, task in reg.tasks.items():
        fields = reg.episode_fields(tid)
        assert fields['task_id'] == tid
        for key in ('family', 'verb', 'object', 'start_state', 'end_state'):
            assert fields[key], f'{tid}: {key}'
        assert task['object'] in objects
        assert task.get('destination') is None or task['destination'] in objects
        assert task['instruction'] != task['text']      # wording switches in v6
    assert reg.by_id('pick_up_soda_can')['instruction'] == 'pick up the soda can'
    assert reg.by_id('put_soda_can_in_trash')['instruction'] == 'put the soda can in the trash'
    # the strings the deployed policy hears are unchanged until v6
    assert reg.by_id('pick_up_soda_can')['text'] == 'find the can and pick it up'
    assert [t['id'] for t in reg.recordable()] == ['pick_up_soda_can', 'put_soda_can_in_trash']


def test_instruction_must_match_template():
    data = {'templates': {'pick_up': 'pick up the {object}'},
            'poses': {'p': {'left_arm': [0] * 6, 'right_arm': [0] * 6, 'head': [0] * 3,
                            'back': 0.3}},
            'tasks': {'t': {'text': 'x', 'start_pose': 'p', 'max_s': 1, 'verb': 'pick_up',
                            'object': 'can', 'instruction': 'grab the can'}}}
    with pytest.raises(tr.RegistryError, match='template'):
        tr.TaskRegistry(data)
    data['tasks']['t']['instruction'] = 'pick up the can'
    data['tasks']['t']['status'] = 'retired'
    reg = tr.TaskRegistry(data)
    assert reg.recordable() == []


def test_hardware_and_objects():
    hw = tr.load_hardware(TASKS_YAML)
    assert hw['current'] == 'hw2' and 'hw1' in hw['versions']
    names = [o['name'] for o in tr.load_objects(TASKS_YAML).values()]
    assert len(names) == len(set(names))


def test_close_frame_skips_sweeps_and_released_attempts(reg):
    close = reg.by_id('pick_up_soda_can')['close']
    dt = 1 / 15

    def close_frame(spec):       # spec: [(gripper, back, n frames), ...]
        frames = [state(gripper=g, back=b) for g, b, n in spec for _ in range(n)]
        return tr.first_held(close['all'], close['hold_s'], frames, dt)

    reach = (0.0, 0.0, 30)
    # closing on nothing crosses the band in 0.3 s, then a real grasp and the lift
    assert close_frame([reach, (0.4, 0.0, 5), (0.78, 0.0, 10), (0.0, 0.0, 20),
                        (0.4, 0.0, 60), (0.4, 0.34, 30)]) == 65
    # an attempt released after 1.2 s does not count either
    assert close_frame([reach, (0.4, 0.0, 18), (0.0, 0.0, 20), (0.4, 0.0, 60)]) == 68
    # a miss, and a gripper in the band only once the back is up: no grasp
    assert close_frame([reach, (0.4, 0.0, 5), (0.78, 0.0, 40)]) is None
    assert close_frame([reach, (0.4, 0.34, 60)]) is None
    assert reg.by_id('put_soda_can_in_trash').get('close') is None


def test_former_ids_still_resolve(reg):
    assert reg.by_id('pick_can')['id'] == 'pick_up_soda_can'
    assert reg.by_id('trash_can')['id'] == 'put_soda_can_in_trash'
    assert 'pick_can' not in reg.tasks and reg.by_id('nope') is None
    assert tr.load_objects(TASKS_YAML)['soda_can']['former_ids'] == ['can']
    assert reg.episode_fields('pick_can')['task_id'] == 'pick_up_soda_can'


def test_pick_keeps_the_lift_and_ends_in_the_carry_pose(reg):
    pick = reg.by_id('pick_up_soda_can')
    assert 'keep_s' not in pick['close']                 # episodes are not cut at the grasp
    assert 'lift to the carry pose' in pick['includes']
    carry = reg.pose('carry')
    assert carry['back'] == reg.pose('tucked')['back'] == 0.34
    # the carry pose satisfies the pick's own done condition
    s = [0.0] * 22
    s[6], s[13:19] = carry['back'], carry['right_arm']
    assert tr.check(pick['done']['all'], tr.state_getter(s))


def test_task_numbers_are_unique(reg):
    numbers = [t['number'] for t in reg.tasks.values()]
    assert numbers == [1, 2]
    data = {'poses': {'p': {'left_arm': [0] * 6, 'right_arm': [0] * 6, 'head': [0] * 3,
                            'back': 0.3}},
            'tasks': {'a': {'text': 'x', 'start_pose': 'p', 'max_s': 1, 'number': 3},
                      'b': {'text': 'y', 'start_pose': 'p', 'max_s': 1, 'number': 3}}}
    with pytest.raises(tr.RegistryError, match='already task'):
        tr.TaskRegistry(data)
