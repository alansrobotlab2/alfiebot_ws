"""Unit tests for the vault read/write tools."""
import json

from alfie_agent.tools import vault


def test_write_then_read_roundtrip(tmp_path):
    vault.configure(tmp_path)
    w = vault.call_tool("vault_write", {"path": "notes/todo.md", "content": "buy milk"})
    assert w == {"success": True, "path": "notes/todo.md", "bytes": len(b"buy milk")}
    assert (tmp_path / "notes" / "todo.md").read_text() == "buy milk"

    r = vault.call_tool("vault_read", {"path": "notes/todo.md"})
    assert r == {"path": "notes/todo.md", "text": "buy milk"}


def test_write_creates_audit_log(tmp_path):
    vault.configure(tmp_path)
    vault.call_tool("vault_write", {"path": "a.md", "content": "x"})
    audit = tmp_path / "memory" / "audit" / "writes.jsonl"
    assert audit.is_file()
    rec = json.loads(audit.read_text().splitlines()[-1])
    assert rec["path"] == "a.md" and rec["action"] == "write" and rec["bytes"] == 1


def test_read_missing_file_returns_error(tmp_path):
    vault.configure(tmp_path)
    r = vault.call_tool("vault_read", {"path": "nope.md"})
    assert "error" in r and "not found" in r["error"]


def test_read_case_insensitive_fallback(tmp_path):
    vault.configure(tmp_path)
    vault.call_tool("vault_write", {"path": "Notes/Todo.md", "content": "hi"})
    r = vault.call_tool("vault_read", {"path": "notes/todo.md"})
    assert r.get("text") == "hi"


def test_path_escape_is_rejected_on_read(tmp_path):
    vault.configure(tmp_path)
    r = vault.call_tool("vault_read", {"path": "../secret.txt"})
    assert "error" in r and "escapes vault root" in r["error"]


def test_path_escape_is_rejected_on_write(tmp_path):
    vault.configure(tmp_path)
    r = vault.call_tool("vault_write", {"path": "../evil.md", "content": "x"})
    assert "error" in r and "escapes vault root" in r["error"]
    assert not (tmp_path.parent / "evil.md").exists()


def test_missing_required_params(tmp_path):
    vault.configure(tmp_path)
    assert "error" in vault.call_tool("vault_read", {})
    assert "error" in vault.call_tool("vault_write", {"path": "a.md"})


def test_read_line_slice(tmp_path):
    vault.configure(tmp_path)
    vault.call_tool("vault_write", {"path": "m.md", "content": "l1\nl2\nl3\nl4"})
    r = vault.call_tool("vault_read", {"path": "m.md", "start_line": 2, "num_lines": 2})
    assert r["text"] == "l2\nl3"


def test_empty_file_reads_placeholder(tmp_path):
    vault.configure(tmp_path)
    vault.call_tool("vault_write", {"path": "e.md", "content": ""})
    r = vault.call_tool("vault_read", {"path": "e.md"})
    assert r["text"] == "(empty file)"


def test_unknown_tool_returns_error(tmp_path):
    vault.configure(tmp_path)
    assert "error" in vault.call_tool("nope", {})


def test_list_tools_shape():
    names = {t["name"] for t in vault.list_tools()}
    assert names == {"vault_read", "vault_write"}
