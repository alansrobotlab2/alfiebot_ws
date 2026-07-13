"""Unit tests for the memory subsystem (no ROS / no LLM; a tmp vault only)."""
import json
from datetime import datetime, timezone

from alfie_agent import memory
from alfie_agent.tools import vault


def _dt(h, m):
    return datetime(2026, 7, 12, h, m, 0, tzinfo=timezone.utc)


def test_format_episode_note_has_frontmatter_and_facts():
    note = memory.format_episode_note(
        _dt(14, 32), "Added milk to the list.", ["keeps a shopping list"])
    assert note.startswith("---\n")
    assert "kind: episode" in note
    assert "# 2026-07-12 14:32" in note
    assert "Added milk to the list." in note
    assert "Durable: keeps a shopping list" in note


def test_select_recent_windows_by_time_and_caps_items():
    now = _dt(15, 0)
    entries = [
        {"ts": _dt(14, 0).isoformat(), "summary": "old, outside 30m"},
        {"ts": _dt(14, 40).isoformat(), "summary": "in window a"},
        {"ts": _dt(14, 50).isoformat(), "summary": "in window b"},
        {"ts": _dt(14, 58).isoformat(), "summary": "in window c"},
    ]
    chosen = memory.select_recent(entries, now, window_min=30, max_items=2)
    # 14:00 is outside the 30-minute window; only the last 2 in-window survive.
    assert [e["summary"] for e in chosen] == ["in window b", "in window c"]


def test_select_recent_skips_unparseable_ts():
    now = _dt(15, 0)
    entries = [{"ts": "not-a-date", "summary": "junk"},
               {"ts": _dt(14, 55).isoformat(), "summary": "ok"}]
    chosen = memory.select_recent(entries, now, window_min=30, max_items=5)
    assert [e["summary"] for e in chosen] == ["ok"]


def test_render_preamble_none_when_empty():
    assert memory.render_preamble([]) is None


def test_render_preamble_lists_times_and_summaries():
    out = memory.render_preamble([
        {"ts": _dt(14, 40).isoformat(), "summary": "chatted about rain"}])
    assert "vault_search" in out          # points the model at deeper recall
    assert "- 14:40 — chatted about rain" in out


def test_record_episode_writes_note_and_recent_log(tmp_path):
    vault.configure(tmp_path)

    def fake_summarize(turns):
        assert turns == [{"role": "user", "content": "hi"},
                         {"role": "assistant", "content": "hello"}]
        return {"summary": "greeted each other", "facts": ["fact one"]}

    res = memory.record_episode(
        [{"role": "user", "content": "hi"},
         {"role": "assistant", "content": "hello"}],
        summarize_fn=fake_summarize, now=_dt(14, 32))
    assert res == {"summary": "greeted each other", "facts": ["fact one"]}

    note = tmp_path / "memory" / "episodes" / "2026-07-12-143200.md"
    assert note.is_file()
    assert "greeted each other" in note.read_text()

    recent = tmp_path / "memory" / "recent.jsonl"
    lines = recent.read_text().strip().splitlines()
    assert len(lines) == 1
    entry = json.loads(lines[0])
    assert entry["summary"] == "greeted each other"
    assert entry["facts"] == ["fact one"]


def test_record_episode_empty_summary_stores_nothing(tmp_path):
    vault.configure(tmp_path)
    res = memory.record_episode(
        [{"role": "user", "content": "hi"}],
        summarize_fn=lambda t: {"summary": "", "facts": []}, now=_dt(14, 0))
    assert res is None
    assert not (tmp_path / "memory" / "recent.jsonl").exists()


def test_recent_preamble_roundtrip_through_vault(tmp_path):
    vault.configure(tmp_path)
    now = _dt(15, 0)
    memory.record_episode(
        [{"role": "user", "content": "a"}],
        summarize_fn=lambda t: {"summary": "did thing A", "facts": []},
        now=_dt(14, 50))
    memory.record_episode(
        [{"role": "user", "content": "b"}],
        summarize_fn=lambda t: {"summary": "did thing B", "facts": []},
        now=_dt(14, 55))

    preamble = memory.recent_preamble(now=now, window_min=30, max_items=5)
    assert "did thing A" in preamble
    assert "did thing B" in preamble
    # Ordering is oldest-first so the most recent interaction reads last.
    assert preamble.index("did thing A") < preamble.index("did thing B")


def test_append_recent_trims_to_size_guard(tmp_path, monkeypatch):
    vault.configure(tmp_path)
    monkeypatch.setattr(memory, "_RECENT_KEEP", 3)
    for i in range(5):
        memory._append_recent({"ts": _dt(14, i).isoformat(), "summary": f"s{i}"})
    lines = (tmp_path / "memory" / "recent.jsonl").read_text().strip().splitlines()
    assert len(lines) == 3
    assert [json.loads(ln)["summary"] for ln in lines] == ["s2", "s3", "s4"]
