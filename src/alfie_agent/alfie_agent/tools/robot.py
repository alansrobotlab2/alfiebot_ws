"""
robot — tools that read or move Alfie's body: robot_status, turn_head, gesture,
timer, task.

agent_node wires this module via ``configure(body=..., detect=...)``: ``body`` is
the ROS-side ``alfie_agent.body.Body``; ``detect(prompt)`` is the same nanoowl
call the `look` tool uses, so ``turn_head`` can turn toward a named object. All
safety checks (e-stop, a running task owning the head, travel limits) live in
Body, next to the state they read, so the model can't talk its way past them.

``MOTION_TOOLS`` lists the tools that move the robot; the harness refuses them
for the rest of a turn once web content has entered it (see harness.run_turn).
"""
from alfie_agent import head_motion as hm

_body = None
_detect = None

MOTION_TOOLS = frozenset({"turn_head", "gesture", "task"})

# Horizontal FOV per detector camera stream. left_wide is idle_behavior's
# measured 104 deg; left_center is a 1:1 352-px crop of the 800-px eye, so
# tan(h/2) scales by 352/800 (~59 deg).
_HFOV = {"wide": 104.0, "center": 58.9}


def configure(body=None, detect=None):
    global _body, _detect
    _body = body
    _detect = detect


def _err(msg):
    return {"error": msg}


def _need_body():
    return None if _body is not None else _err("my body isn't connected right now.")


def _hfov(source_topic):
    return _HFOV["wide"] if "wide" in (source_topic or "") else _HFOV["center"]


# --- handlers -------------------------------------------------------------------

def _robot_status(params):
    return _need_body() or _body.status()


def _turn_head(params):
    if (e := _need_body()):
        return e
    target = " ".join(str(params.get("target") or "").lower().split())
    if not target:
        return _err("say where or what to look at")
    hold = params.get("seconds")
    hold = None if hold is None else max(1.0, min(30.0, float(hold)))
    if target in hm.DIRECTIONS:
        bearing, elev = hm.DIRECTIONS[target]
        absolute = target in ("forward", "ahead", "center", "straight")
        res = _body.look(bearing, elev, relative=not absolute, hold_s=hold)
        return {**res, "target": target} if "error" not in res else res
    # Anything else is an object to find with the detector, then turn toward.
    if _detect is None:
        return _err("vision isn't available right now.")
    snap = _detect(target)
    if not snap or snap.get("error"):
        return snap or _err("I couldn't reach the vision detector.")
    dets = sorted(snap.get("detections", []), key=lambda d: -d.get("score", 0.0))
    if not dets:
        return {"found": False, "target": target,
                "result": f"I don't see {target} in front of me."}
    d = dets[0]
    cx, cy = (d["x0"] + d["x1"]) / 2.0, (d["y0"] + d["y1"]) / 2.0
    bearing, elev = hm.image_offset_to_angles(cx, cy, _hfov(snap.get("source_topic")))
    res = _body.look(bearing, elev, relative=True, hold_s=hold)
    if "error" in res:
        return res
    side = "left" if bearing > 3 else "right" if bearing < -3 else "straight ahead"
    return {**res, "found": True, "target": d.get("label") or target,
            "bearing": f"{abs(round(bearing))} degrees {side}"
            if side != "straight ahead" else side}


def _gesture(params):
    if (e := _need_body()):
        return e
    name = str(params.get("name") or "").strip().lower().replace(" ", "_")
    if name not in hm.GESTURES:
        return _err(f"gestures are: {', '.join(hm.GESTURES)}")
    return _body.gesture(name)


def _timer(params):
    if (e := _need_body()):
        return e
    action = str(params.get("action") or "set").lower()
    if action == "set":
        secs = float(params.get("minutes") or 0) * 60 + float(params.get("seconds") or 0)
        return _body.timer_set(secs, params.get("label"))
    if action == "list":
        return _body.timer_list()
    if action == "cancel":
        return _body.timer_cancel(params.get("label"))
    return _err("action must be set, list or cancel")


def _task(params):
    if (e := _need_body()):
        return e
    action = str(params.get("action") or "").lower()
    if action == "start":
        return _body.task_start(params.get("description"))
    if action == "stop":
        return _body.task_stop()
    if action == "status":
        return _body.task_status()
    return _err("action must be start, status or stop")


# --- MCP-shaped module interface ---

_TOOLS = [
    {
        "name": "robot_status",
        "description": "Your own health: e-stop, motors, servo temperatures and "
                       "voltage, back, computer temperatures, wifi, running task.",
        "parameters": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "turn_head",
        "description": "Physically turn your head: 'look left/up/at me', 'face "
                       "the door', 'look at the red cup'. Target is a direction "
                       "(left, right, up, down, forward, behind) or an object.",
        "parameters": {
            "type": "object",
            "properties": {"target": {"type": "string"},
                           "seconds": {"type": "number"}},
            "required": ["target"],
        },
    },
    {
        "name": "gesture",
        "description": "Head gesture: nod, shake, tilt or look_around.",
        "parameters": {
            "type": "object",
            "properties": {"name": {"type": "string"}},
            "required": ["name"],
        },
    },
    {
        "name": "timer",
        "description": "Countdown timers: action set (minutes/seconds, label), "
                       "list, or cancel (label). You announce when one finishes.",
        "parameters": {
            "type": "object",
            "properties": {"action": {"type": "string"},
                           "minutes": {"type": "number"},
                           "seconds": {"type": "number"},
                           "label": {"type": "string"}},
            "required": ["action"],
        },
    },
    {
        "name": "task",
        "description": "Physical tasks with your arms via your learned policy: "
                       "action start (description, e.g. 'pick up the soda can'), "
                       "status, or stop. Start ONLY when the user explicitly asks "
                       "you to do a physical task.",
        "parameters": {
            "type": "object",
            "properties": {"action": {"type": "string"},
                           "description": {"type": "string"}},
            "required": ["action"],
        },
    },
]

_HANDLERS = {"robot_status": _robot_status, "turn_head": _turn_head,
             "gesture": _gesture, "timer": _timer, "task": _task}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    try:
        return handler(arguments or {})
    except (TypeError, ValueError) as e:
        return _err(f"bad arguments: {e}")
