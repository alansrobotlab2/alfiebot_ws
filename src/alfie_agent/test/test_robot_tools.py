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

    def task_start(self, d):
        self.calls.append(("start", d))
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
    robot.call_tool("task", {"action": "start", "description": "pick up the can"})
    assert body.calls[-1] == ("start", "pick up the can")
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


def test_body_task_lifecycle_and_stop_all():
    from std_msgs.msg import String
    b, node, said = _make(task_max_s=0.0)
    assert "isn't running" in b.task_start("pick up the can")["error"]
    node.subs["groot_client/status"](String(data=str({"state": "IDLE", "active": False})))
    assert b.task_start("pick up the can")["ok"]
    assert node.pubs["groot_client/task_description"].msgs[-1].data == "pick up the can"
    assert node.pubs["groot_client/activate"].msgs[-1].data is True
    assert "already doing" in b.task_start("again")["error"]
    assert b.stop_all() == ["task"]
    assert node.pubs["groot_client/activate"].msgs[-1].data is False
    # The run cap stops it and announces why.
    b.task_start("pick up the can")
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


def test_clean_room_runs_pick_then_trash_without_deactivating():
    from alfie_agent.body import PICK_TASK, TRASH_TASK
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    assert b.clean_room(1)["ok"]
    assert "already doing" in b.clean_room(1)["error"]
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
    assert "1 cans in the trash" in said[-1]


def test_stop_cancels_clean_room_at_once():
    b, node, said = _make(pose_timeout_s=0.1)
    _idle(node)
    assert b.clean_room(3)["ok"]
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


def test_clean_room_gives_up_after_pick_timeouts():
    b, node, said = _make(pose_timeout_s=0.1, pick_max_s=0.5, pick_attempts=2)
    _idle(node)
    assert b.clean_room(2)["ok"]
    client = FakeClient(node, b, done_after=100)
    try:
        assert _wait(lambda: not b.task_running(), 10.0)
    finally:
        client.stop.set()
    assert "couldn't find" in said[-1]
    assert node.pubs["groot_client/activate"].msgs[-1].data is False


def test_task_clean_room_action():
    calls = []
    fb = type("B", (), {"clean_room": lambda self, n: calls.append(n) or {"ok": True}})()
    robot.configure(body=fb)
    try:
        robot.call_tool("task", {"action": "clean_room"})
        robot.call_tool("task", {"action": "clean_room", "rounds": 2})
    finally:
        robot.configure()
    assert calls == [3, 2]


def test_clean_room_and_stop_phrases():
    from alfie_agent import conversation as convo
    assert convo.is_clean_room("clean the room")
    assert convo.is_clean_room("Alfie, please clean up this room.")
    assert not convo.is_clean_room("the room is clean")
    assert convo.is_motion_stop("cancel that")
    assert convo.is_motion_stop("Alfie stop")
