"""Integration tests for harness.run_turn with a fake LLM and fake tools."""
from alfie_agent import harness


class FakeLLM:
    """Return scripted completions in order; record the messages seen."""

    def __init__(self, scripted):
        self._scripted = list(scripted)
        self.calls = []

    def stream_completion(self, messages, *, is_current, max_tokens=None):
        self.calls.append([dict(m) for m in messages])
        return self._scripted.pop(0)


def _always_current():
    return True


def test_plain_reply_no_tools():
    llm = FakeLLM(["Hi there!"])
    out = harness.run_turn("hello", [], system_prompt="SYS", llm=llm,
                           call_tool=lambda n, a: {}, is_current=_always_current)
    assert out == "Hi there!"
    assert len(llm.calls) == 1


def test_tool_call_then_answer():
    llm = FakeLLM([
        '<tool_call>{"name": "vault_read", "arguments": {"path": "a.md"}}</tool_call>',
        "It says hello.",
    ])
    seen = {}

    def call_tool(name, args):
        seen["name"], seen["args"] = name, args
        return {"path": "a.md", "text": "hello"}

    out = harness.run_turn("read a.md", [], system_prompt="SYS", llm=llm,
                           call_tool=call_tool, is_current=_always_current)
    assert out == "It says hello."
    assert seen == {"name": "vault_read", "args": {"path": "a.md"}}
    # Second LLM call must include the tool response fed back as a user turn.
    second = llm.calls[1]
    assert second[-1]["role"] == "user"
    assert "<tool_response>" in second[-1]["content"]
    assert "hello" in second[-1]["content"]


def test_cancelled_before_first_call_returns_none():
    llm = FakeLLM(["should not be used"])
    out = harness.run_turn("hi", [], system_prompt="SYS", llm=llm,
                           call_tool=lambda n, a: {}, is_current=lambda: False)
    assert out is None
    assert llm.calls == []


def test_cancel_mid_stream_returns_none():
    # stream_completion returning None models a mid-stream cancellation.
    class NoneLLM:
        def stream_completion(self, messages, *, is_current, max_tokens=None):
            return None

    out = harness.run_turn("hi", [], system_prompt="SYS", llm=NoneLLM(),
                           call_tool=lambda n, a: {}, is_current=_always_current)
    assert out is None


def test_max_iters_forces_final_answer():
    # LLM keeps emitting tool calls; after max_tool_iters it's asked to answer.
    tool_call = '<tool_call>{"name": "x", "arguments": {}}</tool_call>'
    llm = FakeLLM([tool_call, tool_call, "final answer"])
    out = harness.run_turn("go", [], system_prompt="SYS", llm=llm,
                           call_tool=lambda n, a: {"ok": True},
                           is_current=_always_current, max_tool_iters=2)
    assert out == "final answer"
    # Last call carries the "answer now" nudge.
    assert "without" in llm.calls[-1][-1]["content"].lower()


def test_tool_exception_is_fed_back_not_raised():
    llm = FakeLLM([
        '<tool_call>{"name": "boom", "arguments": {}}</tool_call>',
        "recovered.",
    ])

    def call_tool(name, args):
        raise RuntimeError("kaboom")

    out = harness.run_turn("go", [], system_prompt="SYS", llm=llm,
                           call_tool=call_tool, is_current=_always_current)
    assert out == "recovered."
    assert "kaboom" in llm.calls[1][-1]["content"]
