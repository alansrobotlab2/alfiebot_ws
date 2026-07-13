"""Unit tests for the harness tool-call parser and markup stripping."""
from alfie_agent import harness


def test_parses_single_tool_call():
    text = ('sure\n<tool_call>\n{"name": "vault_read", '
            '"arguments": {"path": "notes/todo.md"}}\n</tool_call>')
    calls = harness.parse_tool_calls(text)
    assert len(calls) == 1
    assert calls[0]["name"] == "vault_read"
    assert calls[0]["arguments"] == {"path": "notes/todo.md"}
    assert "error" not in calls[0]


def test_parses_multiple_tool_calls():
    text = ('<tool_call>{"name": "a", "arguments": {"x": 1}}</tool_call>'
            '<tool_call>{"name": "b", "arguments": {}}</tool_call>')
    calls = harness.parse_tool_calls(text)
    assert [c["name"] for c in calls] == ["a", "b"]


def test_no_tool_call_returns_empty():
    assert harness.parse_tool_calls("just a normal spoken reply.") == []


def test_tolerates_trailing_junk_after_json():
    text = ('<tool_call>{"name": "vault_read", "arguments": {"path": "a.md"}} '
            'please\n</tool_call>')
    calls = harness.parse_tool_calls(text)
    assert calls[0]["name"] == "vault_read"
    assert calls[0]["arguments"]["path"] == "a.md"


def test_unterminated_tool_call_is_recovered():
    text = '<tool_call>\n{"name": "vault_read", "arguments": {"path": "a.md"}}'
    calls = harness.parse_tool_calls(text)
    assert len(calls) == 1
    assert calls[0]["name"] == "vault_read"


def test_malformed_tool_call_reports_error():
    text = '<tool_call>this is not json at all</tool_call>'
    calls = harness.parse_tool_calls(text)
    assert len(calls) == 1
    assert calls[0].get("error")
    assert calls[0]["name"] is None


def test_non_dict_arguments_coerced_to_empty():
    text = '<tool_call>{"name": "x", "arguments": "oops"}</tool_call>'
    calls = harness.parse_tool_calls(text)
    assert calls[0]["arguments"] == {}


def test_strip_markup_removes_think_and_tool_calls():
    text = ('<think>deciding</think>Here you go.'
            '<tool_call>{"name": "x", "arguments": {}}</tool_call>')
    assert harness.strip_markup(text) == "Here you go."


def test_strip_markup_handles_unterminated_think():
    assert harness.strip_markup("stuff</think>the answer") == "the answer"


def test_strip_markup_drops_dangling_tool_call():
    assert harness.strip_markup("hello <tool_call>{partial") == "hello"
