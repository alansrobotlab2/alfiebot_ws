"""
body — the agent's ROS-side handle on the robot: head, telemetry, GR00T, timers.

agent_node creates one ``Body`` and wires its methods into the tool modules
(tools/robot.py), the same way the `look` and
`clear_estop` tools are wired to ROS callbacks. Tool calls arrive on worker
threads; every method here only touches state under ``_lock`` and publishes,
never spins.

  * **Head** — plays head_motion plans on ``cmd/head/agent`` (HeadCmd), the
    mux source reserved for the agent (priority 30: above idle, below GR00T, VR
    and tools). Republished at 20 Hz while a plan lasts; when it ends the agent
    goes quiet and the idle behavior takes the head back.
  * **Telemetry** — latest ``robotlowstate`` (servos, back), ``low/headstate``
    and ``low/jetsonstate`` for ``robot_status``.
  * **GR00T** — drives the groot_client's existing control topics:
    ``groot_client/task_description`` (String) then ``groot_client/activate``
    (Bool), and reads ``groot_client/status``. A run is capped at
    ``task_max_s``; ``stop_all`` (the hard-wired stop path) deactivates it.
  * **Timers** — ``timer`` tool countdowns; ``announce(text)`` speaks when one
    fires.
"""
import ast
import math
import threading
import time

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, String

from alfie_msgs.msg import HeadCmd, HeadState, JetsonState, RobotLowState, ServoCmd

from alfie_agent import head_motion as hm

HEAD_HZ = 20.0
STATE_STALE_S = 3.0


class Body:
    def __init__(self, node, *, announce, estopped, task_max_s=120.0,
                 head_enabled=True):
        """``announce(text)`` speaks outside a turn; ``estopped()`` -> bool|None."""
        self.node = node
        self._announce = announce
        self._estopped = estopped
        self.task_max_s = float(task_max_s)
        self.head_enabled = head_enabled
        self._lock = threading.Lock()

        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._head_state = None       # (monotonic, [yaw, pitch, roll])
        self._low_state = None        # (monotonic, RobotLowState)
        self._jetson = None           # (monotonic, JetsonState)
        node.create_subscription(HeadState, 'low/headstate', self._on_head, state_qos)
        node.create_subscription(RobotLowState, 'robotlowstate', self._on_low, state_qos)
        node.create_subscription(JetsonState, 'low/jetsonstate', self._on_jetson, state_qos)

        # --- head ---
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.head_pub = node.create_publisher(HeadCmd, 'cmd/head/agent', cmd_qos)
        self._plan = None             # (t0 monotonic, frames, end_s, label)
        node.create_timer(1.0 / HEAD_HZ, self._head_tick)

        # --- GR00T ---
        ctl_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.task_pub = node.create_publisher(String, 'groot_client/task_description', ctl_qos)
        self.activate_pub = node.create_publisher(Bool, 'groot_client/activate', ctl_qos)
        node.create_subscription(String, 'groot_client/status', self._on_groot, ctl_qos)
        self._groot = None            # (monotonic, status dict)
        self._task = None             # {"description", "t0"} while the agent runs one
        node.create_timer(1.0, self._task_tick)

        # --- timers ---
        self._timers = {}             # label -> monotonic deadline
        node.create_timer(0.5, self._timer_tick)

    # --- state ------------------------------------------------------------------

    def _on_head(self, msg):
        pose = [float(s.current_location) for s in msg.servos[:3]]
        with self._lock:
            self._head_state = (time.monotonic(), pose)

    def _on_low(self, msg):
        with self._lock:
            self._low_state = (time.monotonic(), msg)

    def _on_jetson(self, msg):
        with self._lock:
            self._jetson = (time.monotonic(), msg)

    @staticmethod
    def _fresh(entry):
        return entry is not None and time.monotonic() - entry[0] < STATE_STALE_S

    def head_pose(self):
        """Current [yaw, pitch, roll], or None if the head isn't reporting."""
        with self._lock:
            return list(self._head_state[1]) if self._fresh(self._head_state) else None

    # --- head ---------------------------------------------------------------------

    def _head_ready(self):
        if not self.head_enabled:
            return "head movement is disabled"
        if self._estopped():
            return "the e-stop is engaged"
        if self.task_running():
            return "a task is running and it controls the head"
        if self.head_pose() is None:
            return "the head isn't reporting its position"
        return None

    def look(self, bearing_deg=None, elev_deg=None, relative=True, hold_s=None):
        """Turn the head by/to a bearing and elevation (degrees; + = left / up)."""
        why = self._head_ready()
        if why:
            return {"error": f"can't move my head: {why}"}
        cur = self.head_pose()
        yaw = cur[0] if bearing_deg is None else hm.bearing_to_yaw(bearing_deg, cur[0], relative)
        pitch = cur[1] if elev_deg is None else hm.elevation_to_pitch(elev_deg, cur[1], relative)
        frames, end = hm.look_plan(cur, (yaw, pitch, 0.0),
                                   hold_s if hold_s is not None else hm.DEFAULT_HOLD_S)
        self._start_plan(frames, end, "look")
        return {"ok": True, "yaw_deg": round(math.degrees(frames[0][1])),
                "pitch_deg": round(math.degrees(frames[0][2])), "hold_s": round(end, 1)}

    def gesture(self, name):
        why = self._head_ready()
        if why:
            return {"error": f"can't move my head: {why}"}
        try:
            frames, end = hm.gesture_plan(name, self.head_pose())
        except ValueError as e:
            return {"error": str(e)}
        self._start_plan(frames, end, name)
        return {"ok": True, "gesture": name}

    def _start_plan(self, frames, end, label):
        with self._lock:
            self._plan = (time.monotonic(), frames, end, label)
        self.node.get_logger().info(f'head: {label} ({end:.1f}s)')

    def stop_head(self):
        with self._lock:
            had = self._plan is not None
            self._plan = None
        return had

    def head_moving(self):
        with self._lock:
            return self._plan is not None

    def _head_tick(self):
        with self._lock:
            plan = self._plan
            if plan is None:
                return
            t = time.monotonic() - plan[0]
            if t >= plan[2] or self._estopped():
                self._plan = None
                return
        _, yaw, pitch, roll, speed = hm.frame_at(plan[1], t)
        cmd = HeadCmd()
        cmd.servos = [self._servo(yaw, speed), self._servo(pitch, speed),
                      self._servo(roll, speed)]
        cmd.eye_pwm = [0, 0]
        self.head_pub.publish(cmd)

    @staticmethod
    def _servo(location, speed):
        s = ServoCmd()
        s.enabled = True
        s.target_location = float(location)
        s.target_speed = float(speed)
        s.target_acceleration = 4.0
        s.target_torque = 0.0
        return s

    # --- status -----------------------------------------------------------------

    def status(self):
        """A compact, speakable snapshot of the robot's health."""
        out = {}
        est = self._estopped()
        out["estop"] = "unknown" if est is None else ("engaged" if est else "clear")
        with self._lock:
            low = self._low_state[1] if self._fresh(self._low_state) else None
            jet = self._jetson[1] if self._fresh(self._jetson) else None
        if low is not None:
            servos = [s for s in low.servo_state if s.current_voltage > 0.5]
            if servos:
                hot = max(servos, key=lambda s: s.current_temperature)
                out["hottest_servo_c"] = int(hot.current_temperature)
                out["servo_voltage_min"] = round(min(s.current_voltage for s in servos), 1)
                out["servos_reporting"] = f"{len(servos)}/{len(low.servo_state)}"
            out["back"] = ("calibrated" if low.back_state.is_calibrated
                           else "not calibrated")
            out["back_height_m"] = round(float(low.back_state.current_position), 3)
        else:
            out["motor_boards"] = "not reporting"
        if jet is not None:
            out["cpu_c"] = round(jet.cpu_temp)
            out["gpu_c"] = round(jet.gpu_temp)
            out["ram_used_pct"] = round(jet.ram_usage_percent)
            out["power_w"] = round(jet.power_draw_watts, 1)
            out["wifi"] = (f"{jet.wifi_ssid} {jet.wifi_signal_percent}%"
                           if jet.wifi_connected else "disconnected")
        out["head"] = "reporting" if self.head_pose() is not None else "not reporting"
        out["task"] = self.task_status().get("state", "unknown")
        return out

    # --- GR00T task -------------------------------------------------------------

    def _on_groot(self, msg):
        try:
            data = ast.literal_eval(msg.data)   # groot_client publishes str(dict)
        except (ValueError, SyntaxError):
            return
        if isinstance(data, dict):
            with self._lock:
                self._groot = (time.monotonic(), data)

    def task_running(self):
        with self._lock:
            return self._task is not None

    def task_start(self, description):
        description = " ".join((description or "").split())
        if not description:
            return {"error": "say what the task is, e.g. 'pick up the soda can'"}
        if self._estopped():
            return {"error": "the e-stop is engaged; it has to be cleared first"}
        with self._lock:
            groot = self._groot[1] if self._fresh(self._groot) else None
            running = self._task
        if groot is None:
            return {"error": "the GR00T policy client isn't running "
                             "(launch with launch_groot:=true)"}
        if running is not None:
            return {"error": f"already doing a task: {running['description']}"}
        if groot.get("state") not in ("IDLE",):
            return {"error": f"the policy client is {groot.get('state')}, not idle"}
        self.stop_head()
        self.task_pub.publish(String(data=description))
        self.activate_pub.publish(Bool(data=True))
        with self._lock:
            self._task = {"description": description, "t0": time.monotonic()}
        self.node.get_logger().warn(f'task started: "{description}"')
        return {"ok": True, "started": description,
                "limit_s": round(self.task_max_s)}

    def task_stop(self, why="stopped"):
        with self._lock:
            task, self._task = self._task, None
        self.activate_pub.publish(Bool(data=False))
        if task is None:
            return {"ok": True, "result": "no task was running"}
        self.node.get_logger().warn(f'task {why}: "{task["description"]}"')
        return {"ok": True, "result": f"stopped: {task['description']}"}

    def task_status(self):
        with self._lock:
            groot = self._groot[1] if self._fresh(self._groot) else None
            task = self._task
        if groot is None:
            return {"state": "policy client not running"}
        out = {"state": groot.get("state", "unknown").lower(),
               "active": bool(groot.get("active"))}
        if task is not None:
            out["task"] = task["description"]
            out["elapsed_s"] = round(time.monotonic() - task["t0"])
            out["limit_s"] = round(self.task_max_s)
        return out

    def _task_tick(self):
        with self._lock:
            task = self._task
        if task is None:
            return
        if time.monotonic() - task["t0"] > self.task_max_s:
            self.task_stop("timed out")
            self._announce(f"I stopped {task['description']}; it took longer than "
                           f"{round(self.task_max_s)} seconds.")
        elif self._estopped():
            self.task_stop("e-stopped")

    def stop_all(self):
        """The hard-wired stop path: stop the task and any head motion. No LLM."""
        stopped = []
        if self.task_running():
            self.task_stop("stopped by voice")
            stopped.append("task")
        if self.stop_head():
            stopped.append("head")
        return stopped

    # --- timers -----------------------------------------------------------------

    def timer_set(self, seconds, label):
        label = " ".join((label or "").split()) or "timer"
        seconds = float(seconds)
        if seconds <= 0 or seconds > 24 * 3600:
            return {"error": "timers can run from 1 second to 24 hours"}
        with self._lock:
            if label in self._timers:
                label = f"{label} {len(self._timers) + 1}"
            self._timers[label] = time.monotonic() + seconds
        return {"ok": True, "label": label, "seconds": round(seconds)}

    def timer_list(self):
        now = time.monotonic()
        with self._lock:
            items = sorted(self._timers.items(), key=lambda kv: kv[1])
        return {"timers": [{"label": k, "remaining_s": round(v - now)} for k, v in items]}

    def timer_cancel(self, label=None):
        with self._lock:
            if not self._timers:
                return {"error": "no timers are running"}
            if label:
                match = [k for k in self._timers if label.lower() in k.lower()]
                if len(match) != 1:
                    return {"error": f"{len(match)} timers match '{label}'"}
                key = match[0]
            elif len(self._timers) == 1:
                key = next(iter(self._timers))
            else:
                return {"error": "more than one timer; say which"}
            del self._timers[key]
        return {"ok": True, "cancelled": key}

    def _timer_tick(self):
        now = time.monotonic()
        with self._lock:
            due = [k for k, v in self._timers.items() if v <= now]
            for k in due:
                del self._timers[k]
        for label in due:
            self._announce(f"Your {label} is done." if "timer" in label
                           else f"Your {label} timer is done.")
