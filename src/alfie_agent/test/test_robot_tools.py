"""Unit tests for the robot tools (fake Body) and Body itself (fake node)."""
import time

import pytest

from alfie_agent.tools import robot


class FakeBody:
    def __init__(self):
        self.calls = []

    def look(self, bearing, elev, relative=True, hold_s=None):
        self.calls.append(("look", bearing, elev, relative, hold_s))
        return {"ok": True}

    def gesture(self, name):
        self.calls.append(("gesture", name))
        return {"ok": True}

    def status(self):
        return {"estop": "clear"}

    def timer_set(self, secs, label):
        self.calls.append(("timer", secs, label))
        return {"ok": True}

    def run_skill(self, plan):
        self.calls.append(("skill", plan["name"], plan["rounds"],
                           [s["id"] for s in plan["steps"]]))
        return {"ok": True}

    def task_status(self):
        return {"state": "idle"}

    def task_stop(self):
        self.calls.append(("stop",))
        return {"ok": True}


@pytest.fixture
def body():
    b = FakeBody()
    robot.configure(body=b, detect=None)
    yield b
    robot.configure()


def test_no_body_is_a_clean_error():
    robot.configure()
    assert "isn't connected" in robot.call_tool("robot_status", {})["error"]


def test_turn_head_direction(body):
    robot.call_tool("turn_head", {"target": "Left"})
    assert body.calls[-1][:4] == ("look", 60.0, None, True)
    robot.call_tool("turn_head", {"target": "forward", "seconds": 99})
    assert body.calls[-1] == ("look", 0.0, 0.0, False, 30.0)


def test_turn_head_object_uses_detection_geometry(body):
    robot.configure(body=body, detect=lambda p: {
        "source_topic": "stereo_camera/left_center/image_raw/compressed",
        "detections": [{"label": "can", "score": 0.4, "x0": 0.0, "x1": 0.2,
                        "y0": 0.4, "y1": 0.6}]})
    r = robot.call_tool("turn_head", {"target": "the soda can"})
    _, bearing, elev, rel, _ = body.calls[-1]
    assert rel and bearing > 15 and abs(elev) < 1
    assert r["found"] and "left" in r["bearing"]
    robot.configure(body=body, detect=lambda p: {"detections": []})
    assert robot.call_tool("turn_head", {"target": "a cat"})["found"] is False


def test_gesture_validation(body):
    assert "gestures are" in robot.call_tool("gesture", {"name": "dance"})["error"]
    robot.call_tool("gesture", {"name": "Look Around"})
    assert body.calls[-1] == ("gesture", "look_around")


def test_timer_minutes_and_seconds(body):
    robot.call_tool("timer", {"action": "set", "minutes": 2, "seconds": 5,
                              "label": "tea"})
    assert body.calls[-1] == ("timer", 125.0, "tea")
    assert "bad arguments" in robot.call_tool(
        "timer", {"action": "set", "minutes": "soon"})["error"]


def test_task_actions(body):
    assert "action must be" in robot.call_tool(
        "task", {"action": "start", "description": "pick up the can"})["error"]
    assert body.calls == []
    assert "action must be" in robot.call_tool("task", {"action": "dance"})["error"]


# --- Body with a fake node --------------------------------------------------------

alfie_msgs = pytest.importorskip("alfie_msgs.msg")


class FakeNode:
    def __init__(self):
        self.pubs, self.subs, self.timers = {}, {}, []

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = cb

    def create_publisher(self, typ, topic, qos):
        pub = type("P", (), {"msgs": []})()
        pub.publish = pub.msgs.append
        self.pubs[topic] = pub
        return pub

    def create_timer(self, period, cb):
        self.timers.append(cb)

    def get_logger(self):
        import logging
        return logging.getLogger("fake")


def _head_state(yaw=0.0):
    from alfie_msgs.msg import HeadState
    m = HeadState()
    m.servos[0].current_location = yaw
    return m


def _make(estop=False, **kw):
    from alfie_agent.body import Body
    said = []
    node = FakeNode()
    b = Body(node, announce=said.append, estopped=lambda: estop, **kw)
    return b, node, said


def test_body_head_refuses_without_state_or_with_estop():
    b, node, _ = _make()
    assert "isn't reporting" in b.look(30, None)["error"]
    b, node, _ = _make(estop=True)
    node.subs["low/headstate"](_head_state())
    assert "e-stop" in b.gesture("nod")["error"]


def test_body_plays_plan_then_goes_quiet():
    b, node, _ = _make()
    node.subs["low/headstate"](_head_state(0.1))
    assert b.look(30, None, hold_s=0.0)["ok"]
    head_tick = node.timers[0]
    head_tick()
    msgs = node.pubs["cmd/head/agent"].msgs
    assert len(msgs) == 1 and msgs[0].servos[0].target_location < 0.1
    b._plan = (time.monotonic() - 100, b._plan[1], b._plan[2], "look")
    head_tick()
    assert len(msgs) == 1 and not b.head_moving()


def _plan(name="clean_room", **args):
    from alfie_agent.skills import SkillRegistry
    return SkillRegistry().resolve(name, args)["plan"]


def test_body_task_lifecycle_and_stop_all():
    b, node, said = _make(pose_timeout_s=0.1)
    assert "isn't running" in b.run_skill(_plan("pick_can"))["error"]
    _idle(node)
    started = b.run_skill(_plan("pick_can"))
    assert started["ok"] and started["started"] == "pick can"
    assert "already doing" in b.run_skill(_plan("pick_can"))["error"]
    assert b.task_status()["task"] == "pick can"
    assert b.stop_all() == ["task"]
    assert node.pubs["groot_client/activate"].msgs[-1].data is False
    assert b.task_status()["last"]["result"] == "stopped by voice"
    # The run cap stops it and announces why.
    b.run_skill(_plan("pick_can"))
    b._task["limit_s"] = 0.0
    time.sleep(0.01)
    node.timers[1]()
    assert not b.task_running() and "stopped" in said[-1]


def test_body_timers_fire_and_cancel():
    b, node, said = _make()
    assert b.timer_set(0.01, "tea timer")["ok"]
    b.timer_set(100, "pasta")
    assert [t["label"] for t in b.timer_list()["timers"]] == ["tea timer", "pasta"]
    time.sleep(0.02)
    node.timers[2]()
    assert said == ["Your tea timer is done."]
    assert b.timer_cancel()["cancelled"] == "pasta"
    assert "no timers" in b.timer_cancel()["error"]


def test_learn_voice_tool():
    from alfie_agent.tools import voice
    voice.configure()
    assert "isn't available" in voice.call_tool("learn_voice", {"name": "Alan"})["error"]
    got = []
    voice.configure(enroll=lambda n, f: got.append((n, f)) or {"ok": True})
    voice.call_tool("learn_voice", {"name": " Alan ", "forget": True})
    assert got == [("Alan", True)]
    assert "name" in voice.call_tool("learn_voice", {})["error"]


# --- clean the room ----------------------------------------------------------------

class FakeClient:
    """Plays groot_client: ACTIVE once activated, task_done ``done_after`` s
    after each task text, status at 5 Hz."""

    def __init__(self, node, body, done_after=2.6):
        import threading
        self.node, self.body, self.done_after = node, body, done_after
        self.texts = []
        self.stop = threading.Event()
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def _run(self):
        from std_msgs.msg import String
        seen, since = 0, time.monotonic()
        while not self.stop.wait(0.2):
            tasks = self.node.pubs["groot_client/task_description"].msgs
            acts = self.node.pubs["groot_client/activate"].msgs
            if len(tasks) != seen:
                seen, since = len(tasks), time.monotonic()
                self.texts.append(tasks[-1].data)
            active = bool(acts) and acts[-1].data
            done = active and time.monotonic() - since > self.done_after
            self.node.subs["groot_client/status"](String(data=str({
                "state": "ACTIVE" if active else "IDLE", "active": active,
                "task": tasks[-1].data if tasks else "", "task_done": done})))


def _idle(node):
    from std_msgs.msg import String
    node.subs["groot_client/status"](String(data=str({"state": "IDLE", "active": False})))


def _wait(pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    return False


PICK_TASK, TRASH_TASK = "find the can and pick it up", "put the can in the trash can"


def test_clean_room_runs_pick_then_trash_without_deactivating():
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    assert b.run_skill(_plan(rounds=1))["ok"]
    assert "already doing" in b.run_skill(_plan(rounds=1))["error"]
    client = FakeClient(node, b)
    try:
        assert _wait(lambda: not b.task_running(), 15.0)
    finally:
        client.stop.set()
    assert client.texts == [PICK_TASK, TRASH_TASK]
    acts = [m.data for m in node.pubs["groot_client/activate"].msgs]
    # pose reset deactivates, one activation for the whole chain, off at the end
    assert acts == [False, True, False]
    assert node.pubs["cmd/head/groot"].msgs, "start pose was not held"
    assert node.pubs["cmd/back/groot"].msgs[-1].position == pytest.approx(0.34)
    assert "1 cans in the trash" in said[-1]
    assert "1 cans in the trash" in b.task_status()["last"]["result"]


def test_stop_cancels_clean_room_at_once():
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    assert b.run_skill(_plan(rounds=3))["ok"]
    client = FakeClient(node, b, done_after=100)
    try:
        assert _wait(lambda: node.pubs["groot_client/task_description"].msgs, 3.0)
        assert b.stop_all() == ["task"]
        assert not b.task_running()
        n_task = len(node.pubs["groot_client/task_description"].msgs)
        n_act = len(node.pubs["groot_client/activate"].msgs)
        n_pose = len(node.pubs["cmd/head/groot"].msgs)
        time.sleep(0.5)
        # nothing published after the stop but its own deactivate
        assert node.pubs["groot_client/activate"].msgs[n_act - 1].data is False
        assert len(node.pubs["groot_client/activate"].msgs) == n_act
        assert len(node.pubs["groot_client/task_description"].msgs) == n_task
        assert len(node.pubs["cmd/head/groot"].msgs) == n_pose
        assert said == []
    finally:
        client.stop.set()


def test_stop_deactivates_a_run_the_agent_did_not_start():
    from std_msgs.msg import String
    b, node, _ = _make()
    node.subs["groot_client/status"](String(data=str({"state": "ACTIVE", "active": True})))
    assert b.stop_all() == ["policy"]
    assert node.pubs["groot_client/activate"].msgs[-1].data is False


def _short(plan, max_s=0.5):
    for step in plan["steps"]:
        step["max_s"] = max_s
    return plan


def test_clean_room_gives_up_after_pick_timeouts():
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    assert b.run_skill(_short(_plan(rounds=2)))["ok"]
    client = FakeClient(node, b, done_after=100)
    try:
        assert _wait(lambda: not b.task_running(), 10.0)
    finally:
        client.stop.set()
    assert client.texts == [PICK_TASK, PICK_TASK]      # two attempts, then gave up
    assert said[-1] == ("I couldn't find and pick up can 1, so I stopped. "
                        "I put 0 in the trash.")
    assert node.pubs["groot_client/activate"].msgs[-1].data is False


def _low_state(right_gripper):
    from alfie_msgs.msg import RobotLowState
    m = RobotLowState()
    m.servo_state[11].current_location = right_gripper
    return m


def test_timeout_counts_when_the_gripper_already_holds_the_can():
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    node.subs["robotlowstate"](_low_state(0.40))
    assert b._accepts_timeout(_plan()["steps"][0])       # pick: holding
    assert not b._accepts_timeout(_plan()["steps"][1])   # trash: not released
    assert b.run_skill(_short(_plan(rounds=1)))["ok"]
    client = FakeClient(node, b, done_after=100)
    try:
        assert _wait(lambda: not b.task_running(), 10.0)
    finally:
        client.stop.set()
    # The pick timeout was accepted (one attempt, client left active), then
    # the trash step timed out with the can still in the gripper.
    assert client.texts == [PICK_TASK, TRASH_TASK]
    assert [m.data for m in node.pubs["groot_client/activate"].msgs] == [False, True, False]
    assert "couldn't get can 1 into the trash" in said[-1]


@pytest.fixture
def skill_tools(body):
    from alfie_agent.skills import SkillRegistry
    robot.configure(body=body, detect=None, skills=SkillRegistry())
    return body


def test_skill_tool_runs_registered_skills_only(skill_tools):
    assert robot.call_tool("skill", {"name": "clean_room"})["ok"]
    assert robot.call_tool("skill", {"name": "clean_room", "args": {"rounds": 2}})["ok"]
    assert robot.call_tool("skill", {"name": "clean_room", "rounds": 4})["ok"]
    assert robot.call_tool("skill", {"name": "pick_can"})["ok"]
    assert skill_tools.calls == [
        ("skill", "clean_room", 3, ["pick_can", "trash_can"]),
        ("skill", "clean_room", 2, ["pick_can", "trash_can"]),
        ("skill", "clean_room", 4, ["pick_can", "trash_can"]),
        ("skill", "pick_can", 1, ["pick_can"])]
    res = robot.call_tool("skill", {"name": "do a backflip"})
    assert "no skill named" in res["error"]
    assert robot.call_tool("skill", {"name": "clean_rooms"})["did_you_mean"] == ["clean_room"]
    assert "1 to 10" in robot.call_tool("skill", {"name": "clean_room",
                                                  "args": {"rounds": 99}})["error"]
    assert len(skill_tools.calls) == 4


def test_task_tool_no_longer_starts_anything(skill_tools):
    assert "status or stop" in robot.call_tool(
        "task", {"action": "start", "description": "wave"})["error"]
    assert "status or stop" in robot.call_tool("task", {"action": "clean_room"})["error"]
    assert robot.call_tool("task", {"action": "status"}) == {"state": "idle"}
    assert robot.call_tool("task", {"action": "stop"})["ok"]
    assert skill_tools.calls == [("stop",)]


def test_skill_list_tool(skill_tools):
    listed = robot.call_tool("skill_list", {})
    assert listed["total"] == 3 and len(listed["skills"]) == 3
    found = robot.call_tool("skill_list", {"query": "tidy up"})["skills"]
    assert len(found) == 1 and found[0].startswith("clean_room(rounds?:int)")
    robot.configure()
    assert "aren't loaded" in robot.call_tool("skill", {"name": "clean_room"})["error"]


def test_skill_is_a_motion_tool_and_stop_phrases():
    from alfie_agent import conversation as convo
    assert "skill" in robot.MOTION_TOOLS
    assert not hasattr(convo, "is_clean_room")
    assert convo.is_motion_stop("cancel that")
    assert convo.is_motion_stop("Alfie stop")
