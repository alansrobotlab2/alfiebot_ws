"""
tools — in-process tool registry.

Aggregates the tool modules (currently just ``vault``) behind lloyd's
``list_tools()`` / ``call_tool(name, arguments)`` interface. Each module exposes
the same two functions, so adding a capability is a one-line append to
``_MODULES`` and lifting the whole registry behind an MCP server later is a
mechanical wrap rather than a rewrite.

Tool specs are plain dicts (name / description / JSON-schema parameters). The
prompt builder renders them into the system prompt, and the harness dispatches
parsed ``<tool_call>`` blocks back through ``call_tool``.
"""
from . import vault
from . import qmd_search

# One entry per tool module. Order is stable so the system prompt (and the
# LLM's prefix cache) stays consistent across turns.
_MODULES = [vault, qmd_search]

# Built lazily on first list_tools(): tool name -> owning module.
_dispatch = {}


def configure(vault_root, qmd_url=None, qmd_skip_rerank=False):
    """Configure the underlying tool modules (vault root + QMD search URL)."""
    vault.configure(vault_root)
    qmd_search.configure(qmd_url, skip_rerank=qmd_skip_rerank)


def list_tools():
    """Return the aggregated tool specs, rebuilding the dispatch map."""
    _dispatch.clear()
    all_tools = []
    for mod in _MODULES:
        for tool in mod.list_tools():
            _dispatch[tool["name"]] = mod
            all_tools.append(tool)
    return all_tools


def call_tool(name, arguments):
    """Route a tool call to its owning module; returns the module's dict result."""
    if not _dispatch:
        list_tools()
    mod = _dispatch.get(name)
    if mod is None:
        return {"error": f"unknown tool: {name}"}
    return mod.call_tool(name, arguments)
