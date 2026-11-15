import datetime
import json

from alfie_gr00t import dataset_paths as dp
from alfie_gr00t.episode_manager.dataset import (
    Dataset, DatasetError, episode_bag, episode_source,
)
from alfie_gr00t.recording_context import DEFAULT_CONTEXT, merge_context

import pytest

DAY = datetime.date(2026, 11, 20)


def test_session_names_count_per_day(tmp_path):
    assert dp.create_session(tmp_path, 'hw2', DAY) == '2026-11-20_s01_hw2'
    assert dp.create_session(tmp_path, 'hw2', DAY) == '2026-11-20_s02_hw2'
    assert dp.create_session(tmp_path, 'hw2', DAY + datetime.timedelta(1)) == '2026-11-21_s01_hw2'
    assert dp.is_session_name('2026-11-20_s02_hw2') and not dp.is_session_name('CleanTheRoom')


def test_open_session_skips_closed_and_other_hardware(tmp_path):
    first = dp.create_session(tmp_path, 'hw2', DAY)
    second = dp.create_session(tmp_path, 'hw2', DAY)
    assert dp.open_session(tmp_path, 'hw2', DAY) == second
    assert dp.open_session(tmp_path, 'hw3', DAY) is None
    dp.close_session(tmp_path / second)
    assert dp.is_closed(tmp_path / second)
    assert dp.open_session(tmp_path, 'hw2', DAY) == first
    # a closed session keeps its number: the next one does not reuse it
    assert dp.next_session_name(tmp_path, 'hw2', DAY) == '2026-11-20_s03_hw2'


def test_merge_context_validates():
    tasks = {'pick_up_soda_can': {}}
    ctx, err = merge_context(None, {'task_id': 'pick_up_soda_can', 'round': '3', 'operator': ' alan '},
                             tasks)
    assert err is None and ctx['task_id'] == 'pick_up_soda_can' and ctx['round'] == 3
    assert ctx['operator'] == 'alan' and ctx['source'] == 'demo'
    for bad in ({'task_id': 'nope'}, {'source': 'teleop'}, {'round': 0}, {'round': 'x'}, 'x'):
        same, err = merge_context(ctx, bad, tasks)
        assert err and same == ctx
    assert merge_context(None, {}, tasks)[0] == DEFAULT_CONTEXT


def test_rows_from_before_sessions_still_read():
    old = {'source': 'demo_20261110_163035', 'origin': 'takeover'}
    new = {'bag': 'demo_20261120_100000', 'source': 'correction'}
    assert episode_bag(old) == 'demo_20261110_163035' and episode_source(old) == 'correction'
    assert episode_bag(new) == 'demo_20261120_100000' and episode_source(new) == 'correction'
    assert episode_source({'source': 'demo_20261110_163035'}) == 'demo'


def test_closed_session_refuses_edits(tmp_path):
    name = dp.create_session(tmp_path, 'hw2', DAY)
    ds = Dataset(tmp_path, name)
    ds.meta.mkdir(parents=True)
    (ds.meta / 'episodes.jsonl').write_text(json.dumps(
        {'episode_index': 0, 'task_index': 0, 'length': 20, 'bag': 'demo_x', 'source': 'demo'})
        + '\n')
    assert ds.set_fields(0, {'quality': 'marginal', 'notes': ' wobbly '})
    assert ds.episodes()[0]['quality'] == 'marginal' and ds.episodes()[0]['notes'] == 'wobbly'
    with pytest.raises(DatasetError):
        ds.set_fields(0, {'quality': 'great'})
    with pytest.raises(DatasetError):
        ds.set_fields(0, {'task_id': 'pick_up_soda_can'})
    ds.close({'registry_commit': 'abc'})
    info = dp.session_info(ds.root)
    assert info['closed'] and info['registry_commit'] == 'abc' and 'schema' in info
    for edit in (lambda: ds.set_fields(0, {'quality': 'ok'}), lambda: ds.delete(0),
                 lambda: ds.set_task(0, 0, 'python3')):
        with pytest.raises(DatasetError, match='closed'):
            edit()
