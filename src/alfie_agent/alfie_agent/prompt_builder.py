"""
prompt_builder — assemble Alfie's system prompt.

Ported (minimal slice) from lloyd's prompt_builder.py. Assembles, in order:
  1. an anti-compliance directive (first, for primacy),
  2. the SOUL identity block (Alfie's personality),
  3. the tool-use protocol + the available tools' schemas,
  4. spoken-output rules.

Kept prefix-cache friendly like lloyd's: no per-turn timestamps or other volatile
content, so the served model can reuse the KV cache for the whole system prompt
across turns. SOUL is loaded from ``<vault_root>/alfie/SOUL.md`` if present, else
the bundled ``prompts/SOUL.default.md`` — so the agent runs even against an empty
placeholder vault.
"""
import json
from pathlib import Path

_ANTI_COMPLIANCE = (
    "You are NOT a compliance engine. Your job is to be genuinely useful, not "
    "merely agreeable. Challenge bad ideas plainly and kindly, offer better "
    "ones, and say 'I don't know' rather than inventing an answer."
)

_SPOKEN_RULES = (
    "Everything you say is spoken aloud by a text-to-speech voice. Keep replies "
    "short and conversational — usually one or two sentences. Use plain spoken "
    "language: no markdown, no emoji, no stage directions, no bullet lists. "
    "/no_think"
)

_BUNDLED_SOUL = Path(__file__).parent / "prompts" / "SOUL.default.md"


def load_soul(vault_root=None):
    """Return the SOUL text: the vault copy if present, else the bundled default."""
    if vault_root:
        vault_soul = Path(vault_root).expanduser() / "alfie" / "SOUL.md"
        try:
            if vault_soul.is_file():
                return vault_soul.read_text(encoding="utf-8", errors="replace")
        except OSError:
            pass
    try:
        return _BUNDLED_SOUL.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return "You are Alfie, a friendly desktop robot."


def _render_tools(tools):
    """Render the tool-use protocol and the tool schemas into a prompt block."""
    if not tools:
        return ""
    lines = [
        "# Tools",
        "",
        "You can call tools to look things up or take actions. To call a tool, "
        "emit exactly one tool call, on its own, in this format:",
        "",
        '<tool_call>',
        '{"name": "<tool_name>", "arguments": {<json arguments>}}',
        '</tool_call>',
        "",
        "Then stop. You will receive the result wrapped in "
        "<tool_response>...</tool_response>, and may then call another tool or "
        "answer. When you have what you need, reply in plain spoken language and "
        "do NOT emit a tool call. Only ever call the tools listed below, with "
        "valid JSON arguments.",
        "",
        "Available tools:",
    ]
    for tool in tools:
        schema = tool.get("parameters", {})
        lines.append(
            f"\n- {tool['name']}: {tool.get('description', '').strip()}"
        )
        lines.append(f"  arguments schema: {json.dumps(schema, separators=(',', ':'))}")
    return "\n".join(lines)


def build_system_prompt(soul_text, tools=None):
    """Assemble the full system prompt from the SOUL text and tool specs."""
    sections = [_ANTI_COMPLIANCE, soul_text.strip()]
    tool_block = _render_tools(tools or [])
    if tool_block:
        sections.append(tool_block)
    sections.append(_SPOKEN_RULES)
    return "\n\n".join(s for s in sections if s)
