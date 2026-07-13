"""
memory — the actively-managed memory subsystem (Phase 1).

Backs Alfie's cross-episode memory with plain files under the obsidian vault, so
the existing QMD watcher indexes them for free. Two tiers ship in Phase 1:

  * **Working / recent** — ``memory/recent.jsonl``: one line per interaction
    (``{ts, summary, facts}``). A time+size-windowed view of this is injected
    verbatim at the start of every episode (see ``recent_preamble``).
  * **Episodic** — ``memory/episodes/<ts>.md``: a permanent, human-readable note
    per interaction, reachable later via the ``vault_search`` tool.

All writes go through the audited, path-guarded ``vault_write`` tool (so
``memory/audit/writes.jsonl`` still records them). Reads go through ``vault_read``.
The module therefore relies on ``tools.configure(vault_root)`` having been called
by the node — no separate configuration step of its own.

(Phase 2 — a curated ``facts.md`` profile, recent-buffer pruning/consolidation,
and daily digests — is intentionally deferred.)
"""
import json
from datetime import datetime, timezone

from alfie_agent.tools import vault

_RECENT_PATH = "memory/recent.jsonl"
_EPISODES_DIR = "memory/episodes"
# File-size guard for the rolling recent log (NOT phase-2 consolidation — just a
# bound so the injected-view source can't grow without limit).
_RECENT_KEEP = 200


# --- pure helpers (unit-tested without a vault) -------------------------------

def format_episode_note(ts, summary, facts):
    """Render a permanent, QMD-indexable episode note."""
    human = ts.strftime("%Y-%m-%d %H:%M")
    lines = ["---", f"ts: {ts.isoformat()}", "kind: episode", "---",
             f"# {human}", summary.strip() or "(no summary)"]
    for fact in facts:
        lines.append(f"Durable: {fact}")
    return "\n".join(lines) + "\n"


def select_recent(entries, now, window_min, max_items):
    """Return entries newer than ``window_min`` minutes, capped to the last N."""
    cutoff = now.timestamp() - window_min * 60
    fresh = []
    for entry in entries:
        try:
            ts = datetime.fromisoformat(entry.get("ts", "")).timestamp()
        except (ValueError, TypeError):
            continue
        if ts >= cutoff:
            fresh.append(entry)
    return fresh[-max_items:] if max_items else fresh


def render_preamble(chosen):
    """Render selected recent entries into the recent-memory window.

    Folded into the system prompt as a trailing section by the prompt builder.
    """
    if not chosen:
        return None
    lines = [
        "Recent memory — things that happened earlier, oldest first. Use these "
        "to stay consistent; for anything older or more detailed, use the "
        "vault_search tool.",
    ]
    for entry in chosen:
        try:
            hm = datetime.fromisoformat(entry.get("ts", "")).strftime("%H:%M")
        except (ValueError, TypeError):
            hm = "?"
        lines.append(f"- {hm} — {(entry.get('summary') or '').strip()}")
    return "\n".join(lines)


# --- vault-backed I/O ---------------------------------------------------------

def _read_recent_entries():
    """Load ``recent.jsonl`` as a list of dicts (empty on any error)."""
    res = vault.call_tool("vault_read", {"path": _RECENT_PATH})
    if not isinstance(res, dict) or "error" in res:
        return []
    entries = []
    for line in (res.get("text") or "").splitlines():
        line = line.strip()
        if not line or line == "(empty file)":
            continue
        try:
            entries.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return entries


def _append_recent(entry):
    """Append one entry to ``recent.jsonl``, trimming to the size guard."""
    entries = _read_recent_entries()
    entries.append(entry)
    entries = entries[-_RECENT_KEEP:]
    text = "\n".join(json.dumps(e, separators=(",", ":")) for e in entries) + "\n"
    vault.call_tool("vault_write", {"path": _RECENT_PATH, "content": text})


def recent_preamble(now=None, window_min=30, max_items=5):
    """Build the recent-memory preamble to inject at episode start (or None)."""
    now = now or datetime.now(timezone.utc)
    chosen = select_recent(_read_recent_entries(), now, window_min, max_items)
    return render_preamble(chosen)


def record_episode(turns, summarize_fn, now=None):
    """
    Compact one episode into memory.

    Summarizes the turns, writes the permanent episode note, and appends to the
    recent log. Returns ``{"summary", "facts"}``, or None if there was nothing
    worth storing (empty summary / cancelled summarizer).
    """
    now = now or datetime.now(timezone.utc)
    result = summarize_fn(turns) or {}
    summary = (result.get("summary") or "").strip()
    facts = result.get("facts") or []
    if not isinstance(facts, list):
        facts = []
    if not summary:
        return None
    stamp = now.strftime("%Y-%m-%d-%H%M%S")
    vault.call_tool("vault_write", {
        "path": f"{_EPISODES_DIR}/{stamp}.md",
        "content": format_episode_note(now, summary, facts),
    })
    _append_recent({"ts": now.isoformat(), "summary": summary, "facts": facts})
    return {"summary": summary, "facts": facts}
