"""
see — "what do you see?" open-vocabulary vision via the NanoOWL detector.

The alfie_nanoowl node runs OWL-ViT over the camera *on demand* (no continuous
GPU load): its ``nanoowl/detect`` service runs one detection and returns the
list. This module is the agent-facing half — one tool the LLM can call to report
what's currently visible, and, because the detector is open-vocabulary, to look
for *anything* the user names.

agent_node owns the ROS I/O and wires this module via ``configure(detect=...)``,
where ``detect(prompt) -> snapshot dict`` performs the service call. If the
detector isn't running, ``detect`` returns an ``{"error": ...}`` dict (or None)
and this tool surfaces it cleanly, so a missing vision node never crashes a turn.
"""

_detect = None

_MAX_OBJECTS = 12      # cap the list handed back to the LLM


def configure(detect=None):
    """Wire the tool to the ROS side. ``detect(prompt) -> snapshot dict|error``."""
    global _detect
    _detect = detect


def _err(msg):
    return {"error": msg}


def _position(x0, x1):
    cx = (x0 + x1) / 2.0
    if cx < 0.34:
        return "to your left"
    if cx > 0.66:
        return "to your right"
    return "ahead of you"


def _distance(x0, y0, x1, y1):
    # Rough distance cue from how much of the frame the box fills.
    frac = max(0.0, (x1 - x0)) * max(0.0, (y1 - y0))
    if frac > 0.25:
        return "close"
    if frac > 0.05:
        return "mid-range"
    return "far"


def _snapshot_to_result(snap, looked_for=None):
    dets = snap.get("detections", [])
    objects = []
    for d in sorted(dets, key=lambda d: d.get("score", 0.0), reverse=True):
        objects.append({
            "object": d.get("label"),
            "confidence": round(float(d.get("score", 0.0)), 2),
            "position": _position(d.get("x0", 0.0), d.get("x1", 0.0)),
            "distance": _distance(d.get("x0", 0.0), d.get("y0", 0.0),
                                  d.get("x1", 0.0), d.get("y1", 0.0)),
        })
    objects = objects[:_MAX_OBJECTS]

    result = {
        "seeing": bool(objects),
        "objects": objects,
        "looked_for": snap.get("prompt"),
        "count": len(objects),
    }
    if looked_for:
        result["requested"] = looked_for
    if objects:
        result["summary"] = "; ".join(
            f"{o['object']} ({o['position']}, {o['distance']})" for o in objects)
    else:
        result["summary"] = "Nothing matching in view right now."
    return result


def _look(params):
    params = params or {}
    if _detect is None:
        return _err("vision isn't available right now (detector not running).")

    look_for = params.get("look_for")
    if isinstance(look_for, (list, tuple)):
        look_for = ", ".join(str(x) for x in look_for)
    look_for = (look_for or "").strip() or None

    snap = _detect(look_for)   # one on-demand detection via the ROS service
    if snap is None:
        return _err("I couldn't reach the vision detector.")
    if isinstance(snap, dict) and snap.get("error"):
        return snap
    return _snapshot_to_result(snap, looked_for=look_for)


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "look",
        "description": (
            "Look through the cameras and report what you actually see right now — "
            "use for any 'what/who do you see?', 'is there a ...?', 'what am I "
            "holding?' question; never guess. Open-vocabulary: pass look_for to "
            "search for specific things by name (e.g. 'a red mug, a laptop'). "
            "Returns each object with a rough position (left/ahead/right) and "
            "distance (close/mid-range/far). Takes a moment (the camera look)."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "look_for": {
                    "type": "string",
                    "description": (
                        "Optional comma-separated things to look for, e.g. "
                        "'a person, a phone'. Omit to report whatever is in view."),
                },
            },
            "required": [],
        },
    },
]

_HANDLERS = {"look": _look}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
