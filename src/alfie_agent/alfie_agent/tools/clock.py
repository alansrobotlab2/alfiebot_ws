"""
clock — current date and time.

A tiny stateless tool that answers "what time/day is it?" from the robot's local
clock. The LLM has no wall-clock of its own (and its training cutoff is stale), so
without this it either guesses or refuses; with it, time-relative questions
("what's today's date?", "is it morning?") resolve against reality.

Returns the local time by default. Pass ``utc: true`` for UTC. The reply carries
both a machine-readable ISO-8601 string and pre-formatted human fields so the
model doesn't have to reformat (and mis-say) a timestamp out loud.
"""
from datetime import datetime, timezone


def _now(params):
    use_utc = bool((params or {}).get("utc"))
    now = datetime.now(timezone.utc) if use_utc else datetime.now().astimezone()
    return {
        "iso": now.isoformat(timespec="seconds"),
        "date": now.strftime("%Y-%m-%d"),
        "time": now.strftime("%H:%M:%S"),
        "weekday": now.strftime("%A"),
        "spoken": now.strftime("%A, %B %-d, %Y at %-I:%M %p"),
        "timezone": now.strftime("%Z") or ("UTC" if use_utc else "local"),
    }


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "current_datetime",
        "description": (
            "Current date, day, and local time from the robot's clock — use for "
            "any date/day/time question, never guess. utc=true for UTC."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "utc": {"type": "boolean",
                        "description": "Return UTC instead of local time (default false)."},
            },
            "required": [],
        },
    },
]

_HANDLERS = {"current_datetime": _now}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return {"error": f"unknown tool: {name}"}
    return handler(arguments or {})
