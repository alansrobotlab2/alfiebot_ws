"""
vault — obsidian vault read/write tools.

Ported (minimal slice) from lloyd's agent_mcp/vault.py. Exposes two tools to the
agent, `vault_read` and `vault_write`, backed by a plain directory on disk (the
shared-with-lloyd obsidian vault, or a placeholder folder until it's hooked up).

The vault root is configured once via ``configure(vault_root)`` before the tools
are used (the ROS node passes its ``vault_root`` parameter). Every tool takes a
single ``params: dict`` and returns a plain ``dict`` — a ``{"error": ...}`` dict
on failure rather than raising, so the agent loop can feed the result straight
back to the model.

Security model (kept verbatim from lloyd):
  * Every path is resolved and checked to stay within the vault root, so the
    model can never read or write outside it (``..`` escapes are rejected).
  * Writes are logged, append-only, to ``<vault>/memory/audit/writes.jsonl``.
"""
import json
from datetime import datetime, timezone
from pathlib import Path

# Set by configure(); until then the tools return a clear "not configured" error
# instead of silently touching some default location.
_VAULT = None


def configure(vault_root):
    """Point the vault tools at ``vault_root`` (str or Path). Expands ~."""
    global _VAULT
    _VAULT = Path(vault_root).expanduser()


def _err(msg):
    return {"error": msg}


def _resolve_case_insensitive(rel_path):
    """
    Best-effort case-insensitive lookup of ``rel_path`` under the vault.

    Obsidian links are often typed with the wrong case; walk the path segment by
    segment matching case-insensitively. Returns a resolved Path or None.
    """
    current = _VAULT
    for part in Path(rel_path).parts:
        if not current.is_dir():
            return None
        match = None
        for child in current.iterdir():
            if child.name.lower() == part.lower():
                match = child
                break
        if match is None:
            return None
        current = match
    return current


def _in_vault(target):
    """Return True if ``target`` resolves inside the vault root."""
    try:
        return target.resolve().is_relative_to(_VAULT.resolve())
    except (OSError, ValueError):
        return False


def _audit_write(rel_path, byte_count):
    """Append a write record to the vault's audit log (best effort)."""
    try:
        audit_dir = _VAULT / "memory" / "audit"
        audit_dir.mkdir(parents=True, exist_ok=True)
        record = {
            "ts": datetime.now(timezone.utc).isoformat(),
            "action": "write",
            "path": rel_path,
            "bytes": byte_count,
        }
        with (audit_dir / "writes.jsonl").open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")
    except OSError:
        pass  # auditing must never break a write


def _vault_read(params):
    if _VAULT is None:
        return _err("vault not configured")
    rel = (params.get("path") or "").strip()
    if not rel:
        return _err("path is required")
    target = _VAULT / rel
    if not _in_vault(target):
        return _err("path escapes vault root")
    if not target.exists():
        resolved = _resolve_case_insensitive(rel)
        if resolved is None:
            return _err(f"file not found: {rel}")
        target = resolved
    if target.is_dir():
        return _err(f"path is a directory: {rel}")
    try:
        text = target.read_text(encoding="utf-8", errors="replace")
    except OSError as e:
        return _err(f"could not read {rel}: {e}")

    start = params.get("start_line")
    num = params.get("num_lines")
    if start is not None or num is not None:
        lines = text.splitlines()
        s = max(int(start or 1), 1) - 1  # 1-indexed inclusive
        e = s + int(num) if num is not None else len(lines)
        text = "\n".join(lines[s:e])

    return {"path": rel, "text": text or "(empty file)"}


def _vault_write(params):
    if _VAULT is None:
        return _err("vault not configured")
    rel = (params.get("path") or "").strip()
    if not rel:
        return _err("path is required")
    content = params.get("content")
    if content is None:
        return _err("content is required")
    target = _VAULT / rel
    if not _in_vault(target):
        return _err("path escapes vault root")
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        data = str(content)
        target.write_text(data, encoding="utf-8")
    except OSError as e:
        return _err(f"could not write {rel}: {e}")
    byte_count = len(data.encode("utf-8"))
    _audit_write(rel, byte_count)
    return {"success": True, "path": rel, "bytes": byte_count}


# --- MCP-shaped module interface (sync, in-process) ---
# Mirrors lloyd's agent_mcp modules so these tools can be lifted behind an MCP
# server later without a rewrite: same list_tools()/call_tool(name, args) shape.

_TOOLS = [
    {
        "name": "vault_read",
        "description": (
            "Read a note from the obsidian vault by its path relative to the "
            "vault root (e.g. 'notes/todo.md'). Optionally pass start_line and "
            "num_lines to read a slice."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "path": {"type": "string",
                         "description": "Path relative to the vault root."},
                "start_line": {"type": "integer",
                               "description": "1-indexed first line (optional)."},
                "num_lines": {"type": "integer",
                              "description": "Number of lines to read (optional)."},
            },
            "required": ["path"],
        },
    },
    {
        "name": "vault_write",
        "description": (
            "Create or overwrite a note in the obsidian vault. Give the path "
            "relative to the vault root and the full file content. Parent "
            "folders are created as needed."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "path": {"type": "string",
                         "description": "Path relative to the vault root."},
                "content": {"type": "string",
                            "description": "Full text content to write."},
            },
            "required": ["path", "content"],
        },
    },
]

_HANDLERS = {
    "vault_read": _vault_read,
    "vault_write": _vault_write,
}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
