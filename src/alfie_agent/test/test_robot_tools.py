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
