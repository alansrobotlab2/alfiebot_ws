"""Held-out selection and mix weights of a training build."""

import pytest

from alfie_gr00t import training_views as tv


def eps(n, task='pick', source='demo', quality='ok', session='2026-11-20_s01_hw2', start=0):
    return [{'episode_uid': f'{session}/{i:06d}', 'task_id': task, 'source': source,
             'quality': quality, 'family': 'pick_place'} for i in range(start, start + n)]


def test_heldout_is_per_group_rounded_up_and_repeatable():
    episodes = eps(21) + eps(30, task='trash', session='2026-11-21_s01_hw2')
    held, counts = tv.select_heldout(episodes, fraction=0.1)
    assert counts['pick/demo']['heldout'] == 3 and counts['trash/demo']['heldout'] == 3
    assert held == tv.select_heldout(list(reversed(episodes)), fraction=0.1)[0]
    assert held != tv.select_heldout(episodes, fraction=0.1, salt='other')[0]


def test_small_groups_and_marginal_episodes_are_not_held_out():
    episodes = (eps(4, source='correction') + eps(20, quality='marginal', start=100)
                + eps(10, start=200))
    held, counts = tv.select_heldout(episodes, fraction=0.5)
    assert counts['pick/correction']['heldout'] == 0
    by_uid = {e['episode_uid']: e for e in episodes}
    assert held and all(by_uid[u]['quality'] == 'ok' for u in held)
    # never the whole group
    assert len(tv.select_heldout(eps(5), fraction=0.99)[0]) == 4


def test_once_held_out_always_held_out():
    first, _ = tv.select_heldout(eps(40), fraction=0.1)
    grown = eps(400)
    fresh, _ = tv.select_heldout(grown, fraction=0.1)
    carried, counts = tv.select_heldout(grown, fraction=0.1, previous=first)
    assert first <= carried and counts['pick/demo']['carried'] == len(first)
    assert len(carried) == len(fresh) == 40
    # a smaller fraction later never releases an episode
    assert first <= tv.select_heldout(eps(40), fraction=0.05, previous=first)[0]


def test_mix_is_frame_proportional_unless_overridden():
    frames = {'pick_place__demo': 6000, 'counter__demo': 2000, 'pick_place__correction': 2000}
    mix = tv.mix_ratios(frames)
    assert mix == pytest.approx({'pick_place__demo': 0.6, 'counter__demo': 0.2,
                                 'pick_place__correction': 0.2})
    mix = tv.mix_ratios(frames, {'pick_place__correction': 0.4})
    assert mix['pick_place__correction'] == 0.4 and sum(mix.values()) == pytest.approx(1)
    assert mix['pick_place__demo'] == pytest.approx(0.45)
    with pytest.raises(ValueError):
        tv.mix_ratios(frames, {'nope': 0.1})
    with pytest.raises(ValueError):
        tv.mix_ratios(frames, {'pick_place__correction': 1.0})


def test_build_names(tmp_path):
    import datetime
    day = datetime.date(2026, 11, 20)
    assert tv.next_build_name(tmp_path, day) == '2026-11-20_b01'
    (tmp_path / '2026-11-20_b01').mkdir()
    (tmp_path / '2026-11-20_b01' / 'manifest.json').write_text('{}')
    (tmp_path / '.2026-11-20_b02.tmp').mkdir()
    assert tv.next_build_name(tmp_path, day) == '2026-11-20_b03'
    assert tv.latest_build(tmp_path) == '2026-11-20_b01'


def test_view_names_follow_the_chosen_fields():
    ep = {'family': 'pick_place', 'source': 'correction', 'task_id': 'pick_up_soda_can',
          'round': 2, 'surface': '', 'objects': ['soda can']}
    assert tv.view_name(ep) == 'pick_place__correction'
    assert tv.view_name(ep, ['task_id', 'source', 'round']) == 'pick_up_soda_can__correction__2'
    assert tv.view_name(ep, ['surface']) == 'none'
    with pytest.raises(ValueError):
        tv.view_name({'source': 'heldout'}, ['source'])
