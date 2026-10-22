"""Unit tests for the lloyd-format fact store and fact_get on a tmp vault."""
import pytest

from alfie_agent.tools import facts, vault


@pytest.fixture
def v(tmp_path):
    vault.configure(tmp_path)
    return tmp_path


def test_add_writes_lloyd_shaped_file(v):
    n = facts.add_facts("Alan", "preferences", ["Alan likes Diet Coke."],
                        source_doc="memory/episodes/x.md")
    assert n == 1
    path = v / "memory/facts/Alan/Alan-preference.md"
    data = facts.parse_fact_file(path.read_text())
    assert data["type"] == "facts"
    assert data["entity"] == "Alan" and data["category"] == "preference"
    f = data["facts"][0]
    assert f["id"] == "pref-001"
    assert f["fact"] == "Alan likes Diet Coke."
    assert f["provenance"] == "ALFIE_VOICE"
    assert f["source_doc"] == "memory/episodes/x.md"
    assert f["expired_at"] is None and f["invalid_at"] is None
    assert "### pref-001" in path.read_text()


def test_ids_continue_and_duplicates_refused_across_categories(v):
    facts.add_facts("Alan", "preference", ["Alan likes tea."])
    facts.add_facts("Alan", "preference", ["Alan likes cake."])
    assert facts.add_facts("Alan", "general", ["alan likes TEA"]) == 0
    ids = [f["id"] for f in facts.all_facts("Alan")]
    assert ids == ["pref-001", "pref-002"]


def test_unknown_category_and_bad_entity(v):
    facts.add_facts("Al/an!", "nonsense", ["x is y"])
    assert facts.entities() == ["Alan"]
    assert facts.all_facts("Alan")[0]["category"] == "general"
    assert facts.add_facts("///", "general", ["dropped"]) == 0


def test_invalidated_facts_hidden(v):
    facts.add_facts("Alan", "state", ["Alan is in Denver."])
    path = v / "memory/facts/Alan/Alan-state.md"
    path.write_text(path.read_text().replace("invalid_at: null",
                                             "invalid_at: '2026-10-01'"))
    assert facts.all_facts("Alan") == []
    assert len(facts.all_facts("Alan", include_invalid=True)) == 1


def test_fact_get_tool(v):
    facts.add_facts("Alan", "preference", ["Alan likes tea."])
    r = facts.call_tool("fact_get", {"entity": "alan"})
    assert r == {"entity": "Alan",
                 "facts": [{"category": "preference", "fact": "Alan likes tea."}]}
    r = facts.call_tool("fact_get", {"entity": "Bob"})
    assert r["facts"] == [] and "Alan" in r["note"]


def test_fact_get_falls_back_when_guessed_category_misses(v):
    facts.add_facts("Maria", "relationship", ["Maria lives in Portland."])
    r = facts.call_tool("fact_get", {"entity": "Maria", "category": "location"})
    assert r["facts"] == [{"category": "relationship",
                           "fact": "Maria lives in Portland."}]
