"""
prompt_builder — assemble Alfie's system prompt.

Ported (minimal slice) from lloyd's prompt_builder.py. Assembles, in order:
  1. an anti-compliance directive (first, for primacy),
  2. the SOUL identity block (Alfie's personality),
  3. the tool-use protocol + the available tools' schemas,
  4. spoken-output rules.

Kept prefix-cache friendly like lloyd's: no per-turn timestamps or other volatile
content, so the served model can reuse the KV cache for the whole system prompt
across turns. The optional recent-memory window is appended *last*, so the static
sections ahead of it stay byte-identical and cached; the window itself only
changes when a new conversation (episode) starts. SOUL is loaded from
``<vault_root>/alfie/SOUL.md`` if present, else the bundled
``prompts/SOUL.default.md`` — so the agent runs even against an empty placeholder
vault.
"""
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


# Short type names for compact tool signatures (full JSON-schema envelopes cost
# ~300 prefill tokens for no behavioural gain; a typed signature carries the same
# information the model needs — arg names, types, and which are required).
_TYPE_ABBR = {"string": "str", "integer": "int", "number": "num",
              "boolean": "bool", "array": "list", "object": "obj"}


def _tool_signature(tool):
    """Render a tool as ``name(arg:type, optional?:type)`` from its JSON schema.

    ``?`` marks an optional (non-required) argument.
    """
    schema = tool.get("parameters", {}) or {}
    props = schema.get("properties", {}) or {}
    required = set(schema.get("required", []) or [])
    parts = []
    for name, spec in props.items():
        typ = _TYPE_ABBR.get(spec.get("type"), spec.get("type") or "any")
        opt = "" if name in required else "?"
        parts.append(f"{name}{opt}:{typ}")
    return f"{tool['name']}({', '.join(parts)})"


def _render_tools(tools):
    """Render the tool-use protocol and compact tool signatures into a block.

    Kept deliberately terse: this text is prefilled on every cache miss, so each
    token here is latency. Tools are rendered as typed signatures rather than full
    JSON schemas, and the "answer directly" guidance discourages the speculative
    tool calls that double a turn's latency.
    """
    if not tools:
        return ""
    lines = [
        "# Tools",
        "",
        "Most questions need NO tool — just answer directly. But when a listed tool "
        "fits the request, USE it rather than guessing or working it out yourself; "
        "each tool's description says when it applies. Never tell the user you lack "
        "a capability that a listed tool provides.",
        "",
        "To call a tool, emit exactly one tool call on its own and then stop:",
        '<tool_call>{"name": "<tool_name>", "arguments": {<json args>}}</tool_call>',
        "You then get a <tool_response>; after that call another tool or answer in "
        "plain spoken language (no tool call). '?' marks an optional argument.",
        "",
        "Tools:",
    ]
    for tool in tools:
        lines.append(
            f"- {_tool_signature(tool)} — {tool.get('description', '').strip()}")
    return "\n".join(lines)


def _render_memory(recent_memory):
    """Render the recent-memory window as a trailing system-prompt section.

    The window (from ``memory.render_preamble``) carries its own framing, so this
    only adds a heading to delimit it from the static sections above.
    """
    return "# Recent context\n\n" + recent_memory.strip()


def build_system_prompt(soul_text, tools=None, recent_memory=None):
    """
    Assemble the full system prompt from the SOUL text and tool specs.

    ``recent_memory``, if given, is appended as a trailing section so the static
    sections ahead of it keep a cacheable prefix (see module docstring).
    """
    sections = [_ANTI_COMPLIANCE, soul_text.strip()]
    tool_block = _render_tools(tools or [])
    if tool_block:
        sections.append(tool_block)
    sections.append(_SPOKEN_RULES)
    if recent_memory and recent_memory.strip():
        sections.append(_render_memory(recent_memory))
    return "\n\n".join(s for s in sections if s)
