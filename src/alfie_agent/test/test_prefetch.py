"""Unit tests for the per-turn context prefetch (qmd mocked, tmp vault)."""
import json
import time
from datetime import datetime, timedelta, timezone
from unittest import mock

import pytest

from alfie_agent import prefetch
from alfie_agent.tools import facts, qmd_search, vault


@pytest.fixture
def v(tmp_path):
    vault.configure(tmp_path)
    qmd_search.configure(None)       # vault leg errors out cleanly by default
    return tmp_path


def test_keywords_drop_stopwords():
    assert prefetch.keywords("Hey Alfie, what is my favorite soda?") == [
        "favorite", "soda"]


def test_short_message_skipped(v):
    block, st = prefetch.Prefetcher().build("stop")
    assert block == "" and st["skipped"]


def test_facts_leg_by_name_and_first_person(v):
    facts.add_facts("Alan", "preference", ["Alan likes Diet Coke."])
    facts.add_facts("User", "general", ["The user has a dog named Sam."])
    p = prefetch.Prefetcher()
    block, st = p.build("what soda does alan like")
    assert "Alan (preference): Alan likes Diet Coke." in block
    assert "dog named Sam" not in block
    block, _ = p.build("what is my dog called again")
    assert "dog named Sam" in block


def test_focus_keeps_entity_in_play(v):
    facts.add_facts("Alan", "preference", ["Alan likes Diet Coke."])
    p = prefetch.Prefetcher()
    p.build("tell me about alan please")
    block, _ = p.build("what does he drink these days")
    assert "Alan likes Diet Coke." in block


def test_vault_leg_filters_and_renders(v):
    qmd_search.configure("http://x/query")
    now = datetime.now(timezone.utc)
    fresh = (now - timedelta(minutes=5)).strftime("%Y-%m-%d-%H%M%S")
    res = {"results": [
        {"file": "notes/soda.md", "title": "Soda", "snippet": "cans are red"},
        {"file": "alfie/MEMORY.md", "title": "mem", "snippet": "dup"},
        {"file": f"memory/episodes/{fresh}.md", "title": "e", "snippet": "dup"},
    ], "meta": {"ms": 3}}
    with mock.patch.object(qmd_search, "query", return_value=res) as q:
        block, st = prefetch.Prefetcher().build("where is the red soda")
    sent = q.call_args.args[0]
    assert sent == [{"type": "lex", "query": "red soda"}]
    assert st["vault"] == 1
    assert "Soda (notes/soda.md): cans are red" in block
    assert "dup" not in block


def test_late_vault_leg_carries_to_next_turn(v):
    def slow(*a, **k):
        time.sleep(0.3)
        return {"results": [{"file": "notes/a.md", "title": "A",
                             "snippet": "garage slow hit"}]}
    p = prefetch.Prefetcher(budget_ms=50)
    with mock.patch.object(qmd_search, "query", side_effect=slow):
        block, st = p.build("tell me about the garage")
        assert "vault" in st["late"] and "slow hit" not in block
        time.sleep(0.4)
        block, st = p.build("and the workshop door")
    assert st["carried"] >= 1 and "garage slow hit" in block


def test_earlier_leg_only_on_temporal_cue(v):
    old = (datetime.now(timezone.utc) - timedelta(hours=20)).isoformat()
    (v / "memory").mkdir()
    (v / "memory/recent.jsonl").write_text(
        json.dumps({"ts": old, "summary": "We fixed the gripper."}) + "\n")
    p = prefetch.Prefetcher()
    block, _ = p.build("how is the gripper working")
    assert "fixed the gripper" not in block
    block, _ = p.build("what did we fix yesterday")
    assert "We fixed the gripper." in block


def test_render_caps_whole_rows():
    rows = {"facts": ["a" * 100, "b" * 100], "vault": ["c" * 100]}
    out = prefetch.render(rows, max_chars=330)
    assert len(out) <= 330
    assert "a" * 100 in out and "b" * 100 not in out
    assert prefetch.render({}, 500) == ""


def test_earlier_leg_yesterday_means_yesterday(v):
    now = datetime.now(timezone.utc).astimezone()
    two_days = (now - timedelta(days=2)).isoformat()
    yday = (now - timedelta(days=1)).isoformat()
    (v / "memory").mkdir()
    (v / "memory/recent.jsonl").write_text(
        json.dumps({"ts": two_days, "summary": "Two days ago thing."}) + "\n"
        + json.dumps({"ts": yday, "summary": "Yesterday thing."}) + "\n")
    block, _ = prefetch.Prefetcher().build("what did we talk about yesterday")
    assert "Yesterday thing." in block and "Two days ago" not in block
    block, _ = prefetch.Prefetcher().build("what did we talk about last time")
    assert "Two days ago thing." in block


def test_primary_user_answers_first_person(v):
    facts.add_facts("Alan", "preference", ["Alan likes Diet Coke."])
    block, _ = prefetch.Prefetcher().build("what soda do I like best")
    assert "Diet Coke" not in block
    block, _ = prefetch.Prefetcher(primary_user="alan").build(
        "what soda do I like best")
    assert "Alan (preference): Alan likes Diet Coke." in block


def test_stem_matches_inflections():
    assert prefetch.stem("planning") == prefetch.stem("plan")
    assert prefetch.stem("batteries") == prefetch.stem("battery")
    assert prefetch.stem("trips") == "trip"


def test_second_person_loads_self_facts(v):
    facts.add_facts("Alfie", "hardware", ["Alfie's gripper has a silicone pad."])
    block, _ = prefetch.Prefetcher().build("what changed on your gripper")
    assert "silicone pad" in block
