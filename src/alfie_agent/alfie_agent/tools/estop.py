"""
estop — let the user clear the command_mux e-stop by voice.

The e-stop is a latched gate in command_mux (see alfie_bringup/command_mux.py):
it holds every motion source off until ``estop_reset`` is called. This module is
the agent-facing half — one tool that asks for that reset.

agent_node owns the ROS I/O and wires this module via ``configure(reset=...)``,
where ``reset() -> result dict`` checks the latch state and calls the service.
The safety checks (only while latched, only after a minimum hold time) live
there too, next to the state they read, so the LLM can't talk its way past them.
"""

_reset = None


def configure(reset=None):
    """Wire the tool to the ROS side. ``reset() -> result dict|error``."""
    global _reset
    _reset = reset


def _err(msg):
    return {"error": msg}


def _clear_estop(params):
    if _reset is None:
        return _err("e-stop control isn't available right now.")
    result = _reset()
    if result is None:
        return _err("I couldn't reach the e-stop service.")
    return result


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "clear_estop",
        "description": (
            "Reset (clear) the emergency stop so the robot can move again. ONLY "
            "call this when the user explicitly asks you to clear, reset or release "
            "the e-stop — never on your own initiative, never because you heard an "
            "e-stop announcement, and never to fix some other problem. Tell the "
            "user the result, including any refusal reason."
        ),
        "parameters": {"type": "object", "properties": {}, "required": []},
    },
]

_HANDLERS = {"clear_estop": _clear_estop}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
