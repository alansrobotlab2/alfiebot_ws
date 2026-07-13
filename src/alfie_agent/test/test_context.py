"""Unit tests for harness context helpers: budget trim, summary parsing, injection."""
from alfie_agent import harness


def _pairs(n):
    """Build n user/assistant pairs, each ~40 chars (~10 approx tokens)."""
    turns = []
    for i in range(n):
        turns.append({"role": "user", "content": f"user message number {i:03d} ......"})
        turns.append({"role": "assistant", "content": f"assistant reply number {i:03d} ."})
    return turns


def test_budget_trim_keeps_last_pair_when_over_budget():
    turns = _pairs(10)
    trimmed = harness.budget_trim(turns, max_tokens=1)  # impossibly small
    assert len(trimmed) == 2                            # never drops below one pair
    assert trimmed == turns[-2:]                        # keeps the most recent


def test_budget_trim_noop_when_within_budget():
    turns = _pairs(3)
    trimmed = harness.budget_trim(turns, max_tokens=10_000)
    assert trimmed == turns


def test_budget_trim_drops_oldest_first_in_pairs():
    turns = _pairs(6)
    # Budget for ~roughly the last two pairs.
    total = sum(harness.approx_tokens(t["content"]) for t in turns)
    trimmed = harness.budget_trim(turns, max_tokens=total // 3)
    assert trimmed == turns[-len(trimmed):]            # a suffix of the original
    assert len(trimmed) % 2 == 0                       # pairs stay paired


def test_parse_summary_extracts_json():
    raw = '<think>hmm</think>{"summary": "did a thing", "facts": ["f1", "f2"]}'
    out = harness.parse_summary(raw)
    assert out == {"summary": "did a thing", "facts": ["f1", "f2"]}


def test_parse_summary_tolerates_non_list_facts():
    out = harness.parse_summary('{"summary": "x", "facts": "oops"}')
    assert out == {"summary": "x", "facts": []}


def test_parse_summary_falls_back_to_plain_text():
    out = harness.parse_summary("just some prose, no json here")
    assert out["summary"] == "just some prose, no json here"
    assert out["facts"] == []


def test_summarize_uses_llm_and_parses():
    class FakeLLM:
        def __init__(self):
            self.seen = None

        def stream_completion(self, messages, *, is_current, max_tokens=None):
            self.seen = messages
            return '{"summary": "greeted", "facts": ["likes tea"]}'

    llm = FakeLLM()
    out = harness.summarize([{"role": "user", "content": "hi"},
                             {"role": "assistant", "content": "hello"}], llm)
    assert out == {"summary": "greeted", "facts": ["likes tea"]}
    # The conversation is rendered into the summarizer's user turn.
    assert "User: hi" in llm.seen[-1]["content"]
    assert "Alfie: hello" in llm.seen[-1]["content"]


def test_recent_memory_injected_as_second_system_message():
    class FakeLLM:
        def __init__(self):
            self.calls = []

        def stream_completion(self, messages, *, is_current, max_tokens=None):
            self.calls.append([dict(m) for m in messages])
            return "ok"

    llm = FakeLLM()
    harness.run_turn("hello", [], system_prompt="SYS", llm=llm,
                     call_tool=lambda n, a: {}, is_current=lambda: True,
                     recent_memory="RECENT")
    msgs = llm.calls[0]
    assert msgs[0] == {"role": "system", "content": "SYS"}
    assert msgs[1] == {"role": "system", "content": "RECENT"}  # after cached prefix
    assert msgs[2]["role"] == "user"


def test_no_recent_memory_leaves_single_system_message():
    class FakeLLM:
        def __init__(self):
            self.calls = []

        def stream_completion(self, messages, *, is_current, max_tokens=None):
            self.calls.append([dict(m) for m in messages])
            return "ok"

    llm = FakeLLM()
    harness.run_turn("hello", [], system_prompt="SYS", llm=llm,
                     call_tool=lambda n, a: {}, is_current=lambda: True)
    msgs = llm.calls[0]
    assert [m["role"] for m in msgs] == ["system", "user"]
