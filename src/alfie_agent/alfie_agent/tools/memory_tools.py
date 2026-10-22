"""
memory_tools — Alfie's curated long-term memory (``alfie/MEMORY.md``).

Ported (minimal slice) from lloyd's agent_mcp/session.py memory tools. One small
markdown file of bullet entries — who the people are, what they like, standing
instructions — that the model curates itself with three tools:

  * ``memory_add(entry)``          append one entry,
  * ``memory_replace(old, new)``   rewrite the entry containing ``old``,
  * ``memory_remove(match)``       drop the entry containing ``match``.

The whole file is injected at the start of every episode (``memory.profile_text``),
so it is capped at ``max_bytes`` (``configure``); an add that would cross the cap
is refused with a hint to replace or remove something first — lloyd's
``memory_ceiling`` rule: refuse, never silently truncate.

Writes are screened with lloyd's memory-gate injection patterns
(agent_mcp/_injection_patterns.py, the ``INJECTION_GATE_FAMILIES`` subset). That
matters more here than on lloyd: anyone in the room can talk to Alfie, so a
spoken "remember: ignore previous instructions..." must not become part of every
future system prompt.

All file I/O goes through the audited ``vault`` tools.
"""
import re
from datetime import date

from . import vault

MEMORY_PATH = "alfie/MEMORY.md"
_HEADER = "# Alfie's memory\n\n"

_max_bytes = 1500
# The " (YYYY-MM-DD)" stamp each entry carries.
_DATED_RE = re.compile(r" \(\d{4}-\d{2}-\d{2}\)$")

# lloyd's memory-gate shapes, verbatim (agent_mcp/_injection_patterns.py).
_INJECTION_PATTERNS = [
    re.compile(r"ignore\s+(all\s+)?previous\s+instructions", re.I),
    re.compile(r"disregard\s+(your\s+)?(previous\s+)?instructions", re.I),
    re.compile(r"you\s+are\s+now\s+a", re.I),
    re.compile(r"pretend\s+you\s+are", re.I),
    re.compile(r"new\s+system\s+prompt", re.I),
    re.compile("[\x00​‌‍⁠﻿]"),
]


def configure(max_bytes=1500):
    """Set the byte ceiling for MEMORY.md."""
    global _max_bytes
    _max_bytes = int(max_bytes)


def _err(msg):
    return {"error": msg}


def check_injection(text):
    """Return an error string if ``text`` matches an injection shape, else None."""
    for pat in _INJECTION_PATTERNS:
        if pat.search(text or ""):
            return "refused: entry looks like a prompt injection"
    return None


def read_entries():
    """Return the current entries (bullet text, without the ``- `` marker)."""
    res = vault.call_tool("vault_read", {"path": MEMORY_PATH})
    if not isinstance(res, dict) or "error" in res:
        return []
    return [line[2:].strip() for line in (res.get("text") or "").splitlines()
            if line.startswith("- ") and line[2:].strip()]


def render(entries):
    """Render entries back into the MEMORY.md file text."""
    return _HEADER + "".join(f"- {e}\n" for e in entries)


def _write(entries):
    text = render(entries)
    size = len(text.encode("utf-8"))
    if size > _max_bytes:
        return _err(f"memory is full ({size} > {_max_bytes} bytes); use "
                    "memory_replace to shorten or memory_remove to drop an "
                    "entry first")
    res = vault.call_tool("vault_write", {"path": MEMORY_PATH, "content": text})
    if "error" in res:
        return res
    return {"success": True, "entries": len(entries), "bytes": size}


def _clean_entry(text):
    # One entry is one line: collapse newlines so a multi-line add can't forge
    # extra entries or headings.
    return " ".join((text or "").split())


def _find(entries, match):
    """Index of the single entry containing ``match`` (case-insensitive)."""
    needle = match.lower()
    hits = [i for i, e in enumerate(entries) if needle in e.lower()]
    if not hits:
        return None, _err(f"no memory entry contains: {match}")
    if len(hits) > 1:
        return None, _err(f"{len(hits)} entries contain '{match}'; be more "
                          "specific")
    return hits[0], None


def _memory_add(params):
    entry = _clean_entry(params.get("entry"))
    if not entry:
        return _err("entry is required")
    bad = check_injection(entry)
    if bad:
        return _err(bad)
    entries = read_entries()
    if any(entry.lower() == _DATED_RE.sub("", e).lower() for e in entries):
        return {"success": True, "note": "already remembered"}
    entries.append(f"{entry} ({date.today().isoformat()})")
    return _write(entries)


def _memory_replace(params):
    old = (params.get("old") or "").strip()
    new = _clean_entry(params.get("new"))
    if not old or not new:
        return _err("old and new are required")
    bad = check_injection(new)
    if bad:
        return _err(bad)
    entries = read_entries()
    i, err = _find(entries, old)
    if err:
        return err
    entries[i] = f"{new} ({date.today().isoformat()})"
    return _write(entries)


def _memory_remove(params):
    match = (params.get("match") or "").strip()
    if not match:
        return _err("match is required")
    entries = read_entries()
    i, err = _find(entries, match)
    if err:
        return err
    del entries[i]
    return _write(entries)


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "memory_add",
        "description": (
            "Remember something durable for future conversations (a person's "
            "name, a preference, a standing request). One short third-person "
            "sentence with names, e.g. 'Alan's favorite soda is Diet Coke.'"
        ),
        "parameters": {
            "type": "object",
            "properties": {"entry": {"type": "string"}},
            "required": ["entry"],
        },
    },
    {
        "name": "memory_replace",
        "description": "Correct a remembered entry: the one containing 'old' "
                       "becomes 'new'.",
        "parameters": {
            "type": "object",
            "properties": {"old": {"type": "string"},
                           "new": {"type": "string"}},
            "required": ["old", "new"],
        },
    },
    {
        "name": "memory_remove",
        "description": "Forget the remembered entry containing 'match'.",
        "parameters": {
            "type": "object",
            "properties": {"match": {"type": "string"}},
            "required": ["match"],
        },
    },
]

_HANDLERS = {
    "memory_add": _memory_add,
    "memory_replace": _memory_replace,
    "memory_remove": _memory_remove,
}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
