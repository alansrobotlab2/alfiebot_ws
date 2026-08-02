"""
command_mux — per-subsystem priority arbitration for Alfie's command path.

Replaces the single monolithic RobotLowCmd interface with per-subsystem command
inputs. Multiple sources publish to `cmd/<subsystem>/<source>`; the mux forwards
the highest-priority FRESH source to each firmware topic, at forward_rate_hz.

Subsystems and their firmware outputs (gen2 tier-2 topics, unchanged):
    left_arm   ArmCmd  -> low/left_arm/armcmd
    right_arm  ArmCmd  -> low/right_arm/armcmd
    head       HeadCmd -> low/headcmd   (servos)
    back       BackCmd -> low/backcmd
    base       Twist   -> low/mecanumdrive
    eyes       EyeCmd  -> merged into low/headcmd (eye_pwm)

E-stop is a LATCHED GATE, not a priority source (a timeout-based source would
fail *resumed* when its node dies; e-stop must fail *stopped*):
    - set   via topic  `estop`        (std_msgs/Empty, RELIABLE + TRANSIENT_LOCAL)
    - clear via service `estop_reset`  (std_srvs/Trigger) — the only way to clear
    - state on topic   `estop_state`   (std_msgs/Bool, latched)
While latched, all cmd/* inputs are ignored and the mux actively drives a stop
(hold at current pose, or limp) every tick. After reset, the resume fence ignores
any source whose last message predates the reset, so commanders must publish fresh
setpoints (prevents snap-to-stale-target).

Config: alfie_bringup/config/command_mux.yaml (overrides the built-in defaults).
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from alfie_msgs.msg import ArmCmd, HeadCmd, BackCmd, EyeCmd, ServoCmd, RobotLowState
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool
from std_srvs.srv import Trigger


# ============================================================================
# Fixed subsystem layout (message types + firmware output topics)
# ============================================================================

NUM_ARM_SERVOS = 6
NUM_HEAD_SERVOS = 3

# Servo slices in RobotLowState.servo_state (for e-stop hold-at-current-pose)
LEFT_ARM_SLICE = slice(0, NUM_ARM_SERVOS)                                  # 0-5
RIGHT_ARM_SLICE = slice(NUM_ARM_SERVOS, 2 * NUM_ARM_SERVOS)                # 6-11
HEAD_SLICE = slice(2 * NUM_ARM_SERVOS, 2 * NUM_ARM_SERVOS + NUM_HEAD_SERVOS)  # 12-14

# subsystem -> message type
SUB_TYPES = {
    'left_arm': ArmCmd,
    'right_arm': ArmCmd,
    'head': HeadCmd,
    'back': BackCmd,
    'base': Twist,
    'eyes': EyeCmd,
}

# subsystem -> firmware output topic (eyes has no own topic: merged into head)
SUB_OUTPUTS = {
    'left_arm': 'low/left_arm/armcmd',
    'right_arm': 'low/right_arm/armcmd',
    'head': 'low/headcmd',
    'back': 'low/backcmd',
    'base': 'low/mecanumdrive',
}

# Built-in default source map (priority, timeout_sec). Overridden by YAML params.
# "groot" is the GR00T policy (full-body: arms+head+back+base; no eyes).
# "tool" is servotool3, the bench bring-up UI. It outranks everything including
# vr: it has an explicit per-subsystem take/release plus an operator deadman, so
# the person standing at the robot with it open wins. It publishes only while a
# subsystem is held. It never drives the base.
DEFAULT_SOURCES: Dict[str, Dict[str, dict]] = {
    'left_arm': {'tool': {'priority': 110, 'timeout': 0.5},
                 'vr': {'priority': 100, 'timeout': 0.2},
                 'groot': {'priority': 50, 'timeout': 0.2}},
    'right_arm': {'tool': {'priority': 110, 'timeout': 0.5},
                  'vr': {'priority': 100, 'timeout': 0.2},
                  'groot': {'priority': 50, 'timeout': 0.2}},
    'head': {'tool': {'priority': 110, 'timeout': 0.5},
             'vr': {'priority': 100, 'timeout': 0.2},
             'groot': {'priority': 50, 'timeout': 0.2},
             'agent': {'priority': 30, 'timeout': 0.5},
             'idle': {'priority': 10, 'timeout': 0.5}},
    'back': {'tool': {'priority': 110, 'timeout': 0.5},
             'vr': {'priority': 100, 'timeout': 0.2},
             'groot': {'priority': 50, 'timeout': 0.2},
             'idle': {'priority': 10, 'timeout': 0.5}},
    'base': {'vr': {'priority': 100, 'timeout': 0.2},
             'joy': {'priority': 90, 'timeout': 0.2},
             'groot': {'priority': 50, 'timeout': 0.2},
             'nav': {'priority': 40, 'timeout': 0.5}},
    'eyes': {'tool': {'priority': 110, 'timeout': 0.5},
             'joy': {'priority': 90, 'timeout': 0.2},
             'agent': {'priority': 30, 'timeout': 0.5},
             'idle': {'priority': 10, 'timeout': 0.5}},
}


@dataclass
class Source:
    """One prioritized input topic feeding a subsystem."""
    name: str
    priority: int
    timeout: float
    msg: object = None
    stamp_ns: int = 0  # receipt time of the latest message


class CommandMuxNode(Node):

    def __init__(self):
        # auto-declare the nested source params (head.vr.priority, ...) from YAML
        super().__init__('command_mux_node',
                         automatically_declare_parameters_from_overrides=True)

        # ---- Global config --------------------------------------------------
        self.forward_rate = float(self._p('forward_rate_hz', 50.0))
        self.max_servo_speed = float(self._p('max_servo_speed', 6.0))
        self.max_servo_accel = float(self._p('max_servo_accel', 0.0))
        self.clamp_zero_speed = bool(self._p('clamp_zero_speed', False))
        self.estop_hold = bool(self._p('estop_hold', True))
        self.estop_eye_pwm = int(self._p('estop_eye_pwm', 1))
        self.estop_eye_flash_hz = float(self._p('estop_eye_flash_hz', 1.0))
        self.estop_hold_speed = float(self._p('estop_hold_speed', 1.0))
        self.estop_back_accel = float(self._p('estop_back_accel', 0.05))
        self.resume_fence = bool(self._p('resume_fence', True))

        # ---- E-stop state ---------------------------------------------------
        self.estopped = False
        self.reset_ns = 0            # resume-fence cutoff
        self.state: Optional[RobotLowState] = None  # latest pose (for hold)

        # ---- QoS profiles ---------------------------------------------------
        self.qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        # ---- Sources + subscriptions ---------------------------------------
        self.sources: Dict[str, Dict[str, Source]] = {}
        for sub in SUB_TYPES:
            self.sources[sub] = self._build_sources(sub)

        for sub, srcs in self.sources.items():
            msg_type = SUB_TYPES[sub]
            for src in srcs.values():
                topic = f'cmd/{sub}/{src.name}'
                self.create_subscription(
                    msg_type, topic, self._make_cb(src), self.qos_cmd)

        # ---- Firmware output publishers ------------------------------------
        self.pub: Dict[str, object] = {}
        for sub, topic in SUB_OUTPUTS.items():
            self.pub[sub] = self.create_publisher(SUB_TYPES[sub], topic, self.qos_cmd)

        # ---- E-stop wiring --------------------------------------------------
        self.create_subscription(Empty, 'estop', self._on_estop, qos_latched)
        self.estop_state_pub = self.create_publisher(Bool, 'estop_state', qos_latched)
        self.create_service(Trigger, 'estop_reset', self._on_reset)
        self.create_subscription(
            RobotLowState, 'robotlowstate', self._on_state, self.qos_cmd)
        self._publish_estop_state()  # latched initial "false"

        # ---- Forward loop ---------------------------------------------------
        self.create_timer(1.0 / self.forward_rate, self._forward)

        total_sources = sum(len(s) for s in self.sources.values())
        self.get_logger().info(
            f'command_mux started: {len(self.sources)} subsystems, '
            f'{total_sources} sources, forwarding at {self.forward_rate:.0f} Hz '
            f'(estop_hold={self.estop_hold}, resume_fence={self.resume_fence})')

    # ========================================================================
    # Parameter helpers
    # ========================================================================

    def _p(self, name: str, default):
        """Read a declared param (present iff in the YAML overrides), else default."""
        if self.has_parameter(name):
            val = self.get_parameter(name).value
            if val is not None:
                return val
        return default

    def _build_sources(self, subsystem: str) -> Dict[str, Source]:
        """Merge built-in defaults for a subsystem with any YAML overrides.

        YAML nests as `subsystem: {source: {priority: N, timeout: T}}`, flattened
        by ROS to params `subsystem.source.priority` / `.timeout`.
        """
        merged: Dict[str, dict] = {
            name: dict(cfg) for name, cfg in DEFAULT_SOURCES.get(subsystem, {}).items()
        }
        for key, param in self.get_parameters_by_prefix(subsystem).items():
            parts = key.split('.')  # e.g. "vr.priority"
            if len(parts) != 2:
                continue
            src_name, field = parts
            if field not in ('priority', 'timeout'):
                continue
            merged.setdefault(src_name, {})[field] = param.value

        out: Dict[str, Source] = {}
        for name, cfg in merged.items():
            if 'priority' not in cfg or 'timeout' not in cfg:
                self.get_logger().warn(
                    f'{subsystem}/{name}: missing priority/timeout, skipping')
                continue
            out[name] = Source(name=name, priority=int(cfg['priority']),
                               timeout=float(cfg['timeout']))
        return out

    # ========================================================================
    # Time / callbacks
    # ========================================================================

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _make_cb(self, src: Source):
        def cb(msg):
            src.msg = msg
            src.stamp_ns = self._now_ns()
        return cb

    def _on_state(self, msg: RobotLowState) -> None:
        self.state = msg

    # ========================================================================
    # Arbitration
    # ========================================================================

    def _winner(self, subsystem: str) -> Optional[Source]:
        """Highest-priority source that is fresh and passes the resume fence."""
        now = self._now_ns()
        best: Optional[Source] = None
        for src in self.sources[subsystem].values():
            if src.msg is None:
                continue
            if (now - src.stamp_ns) / 1e9 > src.timeout:
                continue
            if self.resume_fence and src.stamp_ns < self.reset_ns:
                continue
            if best is None or src.priority > best.priority:
                best = src
        return best

    # ========================================================================
    # Servo clamping (global sanity bound)
    # ========================================================================

    def _clamp_servos(self, servos: List[ServoCmd]) -> List[ServoCmd]:
        ms, ma = self.max_servo_speed, self.max_servo_accel
        for s in servos:
            if ms > 0.0:
                if s.target_speed > ms:
                    s.target_speed = ms
                elif self.clamp_zero_speed and s.target_speed <= 0.0:
                    s.target_speed = ms  # 0 == "unlimited" in servo API; bound it
            if ma > 0.0 and s.target_acceleration > ma:
                s.target_acceleration = ma
        return servos

    def _disabled_servo(self) -> ServoCmd:
        s = ServoCmd()
        s.enabled = False
        s.target_location = 0.0
        s.target_speed = 0.0
        s.target_acceleration = 0.0
        s.target_torque = 0.0
        return s

    def _hold_servos(self, servo_states) -> List[ServoCmd]:
        """Build hold-at-current-position ServoCmds from ServoState feedback."""
        out = []
        for ss in servo_states:
            s = ServoCmd()
            s.enabled = True
            s.target_location = ss.current_location
            s.target_speed = self.estop_hold_speed
            s.target_acceleration = 0.0
            s.target_torque = 0.0
            out.append(s)
        return out

    # ========================================================================
    # Forward loop
    # ========================================================================

    def _forward(self) -> None:
        if self.estopped:
            self._publish_estop_commands()
            return

        # Arms: clamp servos, forward the winner (or starve -> firmware watchdog).
        for name in ('left_arm', 'right_arm'):
            w = self._winner(name)
            if w is not None:
                self._clamp_servos(w.msg.joint_cmd)
                self.pub[name].publish(w.msg)

        # Back + base: forward the winner as-is.
        for name in ('back', 'base'):
            w = self._winner(name)
            if w is not None:
                self.pub[name].publish(w.msg)

        # Head + eyes: merge into a single low/headcmd (head owns servos + eyes).
        self._publish_head(self._winner('head'), self._winner('eyes'))

    def _publish_head(self, head_win: Optional[Source], eyes_win: Optional[Source]) -> None:
        if head_win is None and eyes_win is None:
            return  # nothing fresh for the head module -> starve
        out = HeadCmd()
        if head_win is not None:
            out.servos = self._clamp_servos(list(head_win.msg.servos))
        else:
            # eyes-only: keep head servos torque-off while eyes animate
            out.servos = [self._disabled_servo() for _ in range(NUM_HEAD_SERVOS)]
        # int() per element: incoming eye_pwm is a numpy uint16 array; the HeadCmd
        # setter requires Python ints.
        out.eye_pwm = [int(v) for v in eyes_win.msg.eye_pwm] if eyes_win is not None else [0, 0]
        self.pub['head'].publish(out)

    # ========================================================================
    # E-stop
    # ========================================================================

    def _on_estop(self, _msg: Empty) -> None:
        if not self.estopped:
            self.get_logger().warn('E-STOP engaged — motion latched off')
        self.estopped = True
        self._publish_estop_commands()  # active stop now, don't wait for the tick
        self._publish_estop_state()

    def _on_reset(self, request, response):
        self.reset_ns = self._now_ns()
        was = self.estopped
        self.estopped = False
        self._publish_estop_state()
        response.success = True
        response.message = 'e-stop reset' if was else 'not stopped; fence advanced'
        self.get_logger().warn('E-STOP reset — resume fence advanced')
        return response

    def _publish_estop_state(self) -> None:
        m = Bool()
        m.data = self.estopped
        self.estop_state_pub.publish(m)

    def _estop_eye_pwm(self) -> List[int]:
        """Left/right alternating flash for the eyes while latched.

        A dim alternating blink reads as "stopped but powered" from across the
        room; the previous full-brightness stare was both harsh up close and
        easy to mistake for normal operation at a glance. Phase comes from the
        clock rather than a counter so the pattern is continuous regardless of
        which path (forward tick or the engage callback) publishes it.

        estop_eye_flash_hz <= 0 restores the old steady-both-eyes behavior.
        """
        if self.estop_eye_flash_hz <= 0.0:
            return [self.estop_eye_pwm, self.estop_eye_pwm]
        t = self.get_clock().now().nanoseconds / 1e9
        left_on = (t * self.estop_eye_flash_hz) % 1.0 < 0.5
        return [self.estop_eye_pwm, 0] if left_on else [0, self.estop_eye_pwm]

    def _publish_estop_commands(self) -> None:
        """Actively drive every subsystem to a stop while latched."""
        # Base: always stop immediately (better than the 500 ms watchdog).
        self.pub['base'].publish(Twist())

        hold = self.estop_hold and self.state is not None
        if hold:
            st = self.state
            la = ArmCmd(); la.joint_cmd = self._hold_servos(st.servo_state[LEFT_ARM_SLICE])
            ra = ArmCmd(); ra.joint_cmd = self._hold_servos(st.servo_state[RIGHT_ARM_SLICE])
            self.pub['left_arm'].publish(la)
            self.pub['right_arm'].publish(ra)

            head = HeadCmd()
            head.servos = self._hold_servos(st.servo_state[HEAD_SLICE])
            head.eye_pwm = self._estop_eye_pwm()
            self.pub['head'].publish(head)

            bc = BackCmd()
            bc.position = st.back_state.current_position
            bc.velocity = 0.0
            bc.acceleration = self.estop_back_accel
            self.pub['back'].publish(bc)
        else:
            # Limp: torque off arms + head. Back is starved so its firmware
            # watchdog holds the actuator at its current position.
            la = ArmCmd(); la.joint_cmd = [self._disabled_servo() for _ in range(NUM_ARM_SERVOS)]
            ra = ArmCmd(); ra.joint_cmd = [self._disabled_servo() for _ in range(NUM_ARM_SERVOS)]
            self.pub['left_arm'].publish(la)
            self.pub['right_arm'].publish(ra)

            head = HeadCmd()
            head.servos = [self._disabled_servo() for _ in range(NUM_HEAD_SERVOS)]
            head.eye_pwm = self._estop_eye_pwm()
            self.pub['head'].publish(head)


def main(args=None):
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
