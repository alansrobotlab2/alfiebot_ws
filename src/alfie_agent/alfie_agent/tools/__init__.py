"""
tools — in-process tool registry.

Aggregates the tool modules behind lloyd's
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
from . import clock
from . import calc
from . import room
from . import see
from . import estop
from . import memory_tools
from . import facts
from . import robot
from . import weather
from . import web
from . import lloyd
from . import voice

# One entry per tool module. Order is stable so the system prompt (and the
# LLM's prefix cache) stays consistent across turns.
_MODULES = [vault, qmd_search, clock, calc, room, see, estop, memory_tools, facts,
            robot, weather, web, lloyd, voice]

# Tools that move the robot, and tools whose results are untrusted outside
# content. The harness refuses the first for the rest of a turn after the second.
MOTION_TOOLS = robot.MOTION_TOOLS
TAINTING_TOOLS = web.TAINTING_TOOLS | lloyd.TAINTING_TOOLS

# Built lazily on first list_tools(): tool name -> owning module.
_dispatch = {}


def configure(vault_root, qmd_url=None, qmd_skip_rerank=False, room_url=None,
              see_detect=None, estop_reset=None, memory_max_bytes=1500,
              body=None, weather_location="", weather_units="fahrenheit",
              lloyd_url="", lloyd_timeout=90.0, enroll_voice=None):
    """Configure the underlying tool modules (vault root, QMD + room service URLs,
    the on-demand vision `detect` callback for `look`/`turn_head`, the
    `estop_reset` callback for `clear_estop`, the MEMORY.md byte cap, the
    ROS-side `body` for the robot tools, weather defaults and lloyd's URL)."""
    vault.configure(vault_root)
    memory_tools.configure(memory_max_bytes)
    robot.configure(body=body, detect=see_detect)
    weather.configure(home=weather_location, units=weather_units)
    lloyd.configure(url=lloyd_url, timeout=lloyd_timeout)
    voice.configure(enroll=enroll_voice)
    qmd_search.configure(qmd_url, skip_rerank=qmd_skip_rerank)
    room.configure(room_url)
    see.configure(detect=see_detect)
    estop.configure(reset=estop_reset)


def warmup_vault_search():
    """
    Warm the QMD daemon so the first real vault search isn't cold.

    Loads the daemon's embedding model up front (a cold search is ~1.2 s vs
    ~75 ms warm). Best effort; safe to call when qmd is down.
    """
    qmd_search.warmup()


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
