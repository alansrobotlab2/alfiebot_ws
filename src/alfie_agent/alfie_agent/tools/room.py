"""
room — "which room am I in?" via the alfie_room room service.

Alfie has no map, so room-awareness is appearance-based: the alfie_room
``room_node`` embeds the current camera view and matches it against rooms it has
been *taught*. This module is the agent-facing half — two tools the LLM can call,
talking to that node over its local HTTP endpoint (the same shape as
``qmd_search`` talking to QMD):

  * ``identify_room``  -> which known room the cameras are looking at now.
  * ``learn_room(name)`` -> remember the current view under a name (teaching).

Configured via ``configure(room_url)`` (the ROS node passes its ``room_url``
param). If the service is down every tool returns a clear error dict instead of
raising, so a missing vision node never crashes a conversation turn.
"""
import requests

_ROOM_URL = None
# (connect, read). The first request lazily loads the vision model on the node,
# so the read timeout is generous; later requests are fast.
_TIMEOUT = (5, 30)


def configure(room_url):
    """Point the room tools at the room_node HTTP base URL.

    E.g. ``http://localhost:8182/``.
    """
    global _ROOM_URL
    _ROOM_URL = (room_url or "").rstrip("/")


def _err(msg):
    return {"error": msg}


def _post(path, payload=None):
    if not _ROOM_URL:
        return None, _err("room recognition not configured (no room_url)")
    try:
        r = requests.post(_ROOM_URL + path, json=payload or {}, timeout=_TIMEOUT)
        r.raise_for_status()
        return r.json(), None
    except requests.exceptions.ConnectionError:
        return None, _err("room recognition unavailable (vision node not running)")
    except requests.exceptions.Timeout:
        return None, _err("room recognition timed out")
    except Exception as e:
        return None, _err(f"room recognition failed: {e}")


def _identify_room(params):
    data, err = _post("/classify")
    if err:
        return err
    if data.get("error"):
        return {"error": data["error"]}
    known = data.get("known_rooms", [])
    if data.get("unknown") or not data.get("room"):
        return {
            "room": None,
            "message": "This doesn't look like a room I've been taught yet.",
            "closest_guess": data.get("best_guess"),
            "known_rooms": known,
        }
    return {
        "room": data["room"],
        "confidence": data.get("score"),
        "known_rooms": known,
    }


def _learn_room(params):
    name = (params.get("name") or "").strip()
    if not name:
        return _err("a room name is required")
    data, err = _post("/teach", {"name": name})
    if err:
        return err
    if data.get("error"):
        return {"error": data["error"]}
    return {
        "learned": data.get("taught", name),
        "views_learned": data.get("exemplars"),
        "known_rooms": data.get("known_rooms", []),
    }


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "identify_room",
        "description": (
            "Look through the cameras and identify which room you're in. Use for "
            "any 'where are you / what room is this?' question — never guess. "
            "Returns the room name (or null if it's a place you haven't learned)."
        ),
        "parameters": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "learn_room",
        "description": (
            "Memorize the room you're currently looking at under a name, when the "
            "user teaches you a place (e.g. 'this is the kitchen'). Call again "
            "from different spots in the same room to recognize it more reliably."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "name": {"type": "string",
                         "description": "The room's name, e.g. 'kitchen'."},
            },
            "required": ["name"],
        },
    },
]

_HANDLERS = {"identify_room": _identify_room, "learn_room": _learn_room}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
