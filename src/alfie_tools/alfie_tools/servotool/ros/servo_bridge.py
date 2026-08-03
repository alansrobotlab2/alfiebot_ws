"""ROS side of servotool3: state aggregation + arbitrated joint commanding.

Everything the web UI does goes through this class. It:

  * subscribes to the per-module gen2 state topics (arms / head / back) and to
    the command_mux's latched e-stop state,
  * publishes commands to the mux as the ``tool`` source (never straight to the
    firmware topics), so priority arbitration and the global servo clamp still
    apply,
  * only publishes for a subsystem the operator has explicitly taken control of,
    and drops that control if the browser stops sending heartbeats.

Release semantics matter: releasing a subsystem stops publishing. The mux then
forwards the next-highest fresh source, and if there is none the module firmware
watchdog limps the servos ~500 ms later. A held arm therefore falls when control
is released - the same thing that happens when any commander dies.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional

from alfie_msgs.msg import ArmCmd, ArmState, BackCmd, BackState, EyeCmd, HeadCmd, HeadState
from alfie_msgs.srv import BackRequestCalibration, ServoService
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Trigger

from alfie_tools.servotool3.ros import joint_config as jc

# A module is "online" if its last state message is newer than this.
STALE_AFTER_SEC = 0.5

# Window used to estimate per-topic publish rate.
RATE_WINDOW = 25

ARM_SIDE_NAMES = {0: 'left', 1: 'right', 255: 'unknown'}


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


class _RateMeter:
    """Publish-rate estimator for one topic."""

    def __init__(self) -> None:
        self._stamps: Deque[float] = deque(maxlen=RATE_WINDOW)

    def tick(self) -> None:
        self._stamps.append(time.monotonic())

    @property
    def last(self) -> Optional[float]:
        return self._stamps[-1] if self._stamps else None

    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0:
            return 0.0
        # Stale topics should read 0, not a frozen historical rate.
        if time.monotonic() - self._stamps[-1] > STALE_AFTER_SEC:
            return 0.0
        return (len(self._stamps) - 1) / span


@dataclass
class JointTarget:
    """What servotool3 is commanding for one joint while it holds control."""

    enabled: bool = False
    location: float = 0.0
    speed: float = jc.DEFAULT_SPEED
    acceleration: float = jc.DEFAULT_ACCEL
    torque: float = jc.DEFAULT_TORQUE

    def as_dict(self) -> Dict[str, Any]:
        return {
            'enabled': self.enabled,
            'target_location': self.location,
            'target_speed': self.speed,
            'target_acceleration': self.acceleration,
            'target_torque': self.torque,
        }


@dataclass
class BackTarget:
    """What servotool3 is commanding for the back actuator."""

    position: float = 0.0
    velocity: float = jc.BACK_DEFAULT_VELOCITY
    acceleration: float = jc.BACK_DEFAULT_ACCELERATION

    def as_dict(self) -> Dict[str, Any]:
        return {
            'position': self.position,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
        }


@dataclass
class Control:
    """Ownership + desired command for one subsystem."""

    owned: bool = False
    joints: List[JointTarget] = field(default_factory=list)
    eye_pwm: List[int] = field(default_factory=lambda: [0, 0])
    back: BackTarget = field(default_factory=BackTarget)


class ServoBridge:
    """Owns every ROS entity servotool3 needs, and the tool's command state."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self._lock = threading.RLock()

        # ---- Parameters ----------------------------------------------------
        self.source = node.declare_parameter('source', 'tool').value
        self.forward_rate = float(node.declare_parameter('forward_rate_hz', 20.0).value)
        self.control_timeout = float(node.declare_parameter('control_timeout', 1.5).value)
        self.max_speed = float(node.declare_parameter('max_speed', jc.MAX_SPEED).value)
        # Register reads are bus transactions squeezed into the module's 50 Hz
        # servo tick, so this stays low: it is a config view, not telemetry.
        self.memory_poll_rate = float(
            node.declare_parameter('memory_poll_hz', 2.0).value)

        # Absolute topic names, so the tool works from a bare `ros2 run` without
        # having to be launched inside the robot namespace (same approach as
        # joydrive). Set robot_namespace:='' for an un-namespaced robot.
        namespace = str(node.declare_parameter('robot_namespace', 'alfie').value).strip('/')
        self.prefix = f'/{namespace}/' if namespace else '/'

        jc.apply_urdf_limits(node.get_logger())

        # ---- Latest state --------------------------------------------------
        self.arm_state: Dict[str, Optional[ArmState]] = {jc.LEFT_ARM: None, jc.RIGHT_ARM: None}
        self.head_state: Optional[HeadState] = None
        self.back_state: Optional[BackState] = None
        self.estop_engaged: Optional[bool] = None      # None = mux not heard from

        self.rates: Dict[str, _RateMeter] = {
            jc.LEFT_ARM: _RateMeter(),
            jc.RIGHT_ARM: _RateMeter(),
            jc.HEAD: _RateMeter(),
            jc.BACK: _RateMeter(),
        }

        # ---- Control state -------------------------------------------------
        self.control: Dict[str, Control] = {s: Control() for s in jc.ALL_SUBSYSTEMS}
        for subsystem in jc.SERVO_SUBSYSTEMS:
            self.control[subsystem].joints = [
                JointTarget() for _ in jc.JOINTS_BY_SUBSYSTEM[subsystem]]
        self._last_heartbeat = 0.0
        self._deadman_warned = False

        # ---- Register-map view ----------------------------------------------
        # The UI shows one servo at a time, so only that one is polled. Selection
        # is server-side state rather than a query parameter so the SSE stream can
        # carry the map without the browser round-tripping for it.
        self.selection = {'subsystem': jc.LEFT_ARM, 'bus_id': 1}
        self._memory: Optional[Dict[str, Any]] = None
        self._memory_stamp = 0.0
        self._memory_error: Optional[str] = None
        self._memory_busy = False

        # ---- ROS entities ---------------------------------------------------
        group = ReentrantCallbackGroup()
        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)

        p = self.prefix
        node.create_subscription(
            ArmState, f'{p}low/left_arm/armstate',
            lambda m: self._on_arm(jc.LEFT_ARM, m), qos_be, callback_group=group)
        node.create_subscription(
            ArmState, f'{p}low/right_arm/armstate',
            lambda m: self._on_arm(jc.RIGHT_ARM, m), qos_be, callback_group=group)
        node.create_subscription(
            HeadState, f'{p}low/headstate', self._on_head, qos_be, callback_group=group)
        node.create_subscription(
            BackState, f'{p}low/backstate', self._on_back, qos_be, callback_group=group)
        node.create_subscription(
            Bool, f'{p}estop_state', self._on_estop_state, qos_latched, callback_group=group)

        self.pub = {
            jc.LEFT_ARM: node.create_publisher(
                ArmCmd, f'{p}cmd/left_arm/{self.source}', qos_be),
            jc.RIGHT_ARM: node.create_publisher(
                ArmCmd, f'{p}cmd/right_arm/{self.source}', qos_be),
            jc.HEAD: node.create_publisher(HeadCmd, f'{p}cmd/head/{self.source}', qos_be),
            jc.EYES: node.create_publisher(EyeCmd, f'{p}cmd/eyes/{self.source}', qos_be),
            jc.BACK: node.create_publisher(BackCmd, f'{p}cmd/back/{self.source}', qos_be),
        }
        self.estop_pub = node.create_publisher(Empty, f'{p}estop', qos_latched)
        self.estop_reset_cli = node.create_client(
            Trigger, f'{p}estop_reset', callback_group=group)
        self.calibrate_cli = node.create_client(
            BackRequestCalibration, f'{p}low/calibrate_back', callback_group=group)

        # Register-map service, one per servo module (read-only in this build).
        self.memory_cli = {
            jc.LEFT_ARM: node.create_client(
                ServoService, f'{p}low/left_arm/servoservice', callback_group=group),
            jc.RIGHT_ARM: node.create_client(
                ServoService, f'{p}low/right_arm/servoservice', callback_group=group),
            jc.HEAD: node.create_client(
                ServoService, f'{p}low/headservoservice', callback_group=group),
        }

        node.create_timer(1.0 / self.forward_rate, self._forward, callback_group=group)
        node.create_timer(1.0 / self.memory_poll_rate, self._poll_memory,
                          callback_group=group)

        node.get_logger().info(
            f'servo bridge up: publishing {p}cmd/<subsystem>/{self.source} at '
            f'{self.forward_rate:.0f} Hz, control deadman {self.control_timeout:.1f} s')

    # ======================================================================
    # Subscriptions
    # ======================================================================

    def _on_arm(self, subsystem: str, msg: ArmState) -> None:
        with self._lock:
            self.arm_state[subsystem] = msg
            self.rates[subsystem].tick()

    def _on_head(self, msg: HeadState) -> None:
        with self._lock:
            self.head_state = msg
            self.rates[jc.HEAD].tick()

    def _on_back(self, msg: BackState) -> None:
        with self._lock:
            self.back_state = msg
            self.rates[jc.BACK].tick()

    def _on_estop_state(self, msg: Bool) -> None:
        with self._lock:
            self.estop_engaged = bool(msg.data)

    # ======================================================================
    # Feedback helpers
    # ======================================================================

    def _servo_states(self, subsystem: str) -> Optional[List[Any]]:
        """Latest ServoState array for a subsystem, or None if it is offline."""
        if subsystem in self.arm_state:
            msg = self.arm_state[subsystem]
            return list(msg.joint_state) if msg is not None else None
        if subsystem == jc.HEAD:
            return list(self.head_state.servos) if self.head_state is not None else None
        return None

    def _module_online(self, subsystem: str) -> bool:
        meter = self.rates.get(subsystem)
        last = meter.last if meter is not None else None
        return last is not None and (time.monotonic() - last) < STALE_AFTER_SEC

    # ======================================================================
    # Control ownership
    # ======================================================================

    def touch(self) -> None:
        """Register operator presence (called on every heartbeat/command)."""
        with self._lock:
            self._last_heartbeat = time.monotonic()
            self._deadman_warned = False

    def take(self, subsystem: str) -> Dict[str, Any]:
        """Take control of a subsystem, seeded from its current feedback.

        Seeding matters: the tool starts commanding immediately at the
        forward rate, so the first command it sends must be "stay exactly where
        you are, with the torque state you already had".
        """
        with self._lock:
            if subsystem not in self.control:
                return {'ok': False, 'error': f'unknown subsystem "{subsystem}"'}

            control = self.control[subsystem]

            if subsystem in jc.SERVO_SUBSYSTEMS:
                states = self._servo_states(subsystem)
                if states is None or not self._module_online(subsystem):
                    return {'ok': False,
                            'error': f'no state from {jc.SUBSYSTEM_LABELS[subsystem]} '
                                     f'- is the module powered and the agent running?'}
                for target, state in zip(control.joints, states):
                    target.enabled = bool(state.enabled)
                    # Seeded from the measured angle *unclamped*: taking control
                    # must never itself be a motion command, and a joint parked
                    # outside its URDF limit would otherwise be told to snap
                    # into range the instant the tool takes over. Operator
                    # input is still clamped, in apply_command().
                    target.location = float(state.current_location)
                    target.speed = jc.DEFAULT_SPEED
                    target.acceleration = jc.DEFAULT_ACCEL
                    # A servo reporting a 0 torque limit cannot move; seed the
                    # tool with a usable limit rather than inheriting the zero.
                    torque = float(state.target_torque)
                    target.torque = torque if torque > 0.0 else jc.DEFAULT_TORQUE

            elif subsystem == jc.EYES:
                if self.head_state is not None:
                    control.eye_pwm = [int(v) for v in self.head_state.eye_state]

            elif subsystem == jc.BACK:
                if self.back_state is None or not self._module_online(jc.BACK):
                    return {'ok': False, 'error': 'no state from the back module'}
                # Unclamped for the same reason as the servo joints - and the
                # actuator genuinely reports a little past the nominal stroke.
                control.back.position = float(self.back_state.current_position)

            control.owned = True
            self._last_heartbeat = time.monotonic()
            self.node.get_logger().info(f'took control of {subsystem}')
            return {'ok': True}

    def release(self, subsystem: str) -> Dict[str, Any]:
        """Release a subsystem: stop publishing and let the mux fall through."""
        with self._lock:
            if subsystem not in self.control:
                return {'ok': False, 'error': f'unknown subsystem "{subsystem}"'}
            if self.control[subsystem].owned:
                self.control[subsystem].owned = False
                self.node.get_logger().info(f'released {subsystem}')
            return {'ok': True}

    def release_all(self, reason: str) -> None:
        with self._lock:
            owned = [s for s, c in self.control.items() if c.owned]
            for subsystem in owned:
                self.control[subsystem].owned = False
            if owned:
                self.node.get_logger().warn(f'released {", ".join(owned)} ({reason})')

    def torque_off(self, subsystem: Optional[str] = None) -> Dict[str, Any]:
        """Command torque off. Only meaningful while the subsystem is held."""
        targets = jc.SERVO_SUBSYSTEMS if subsystem is None else (subsystem,)
        with self._lock:
            for name in targets:
                if name not in self.control:
                    return {'ok': False, 'error': f'unknown subsystem "{name}"'}
                for joint in self.control[name].joints:
                    joint.enabled = False
            self._last_heartbeat = time.monotonic()
        return {'ok': True}

    # ======================================================================
    # Command application (from the web UI)
    # ======================================================================

    def apply_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """Update the desired command for one subsystem.

        Values are clamped here rather than trusted from the browser - the UI is
        one input path among several and this is the only place that sees all of
        them.
        """
        subsystem = payload.get('subsystem')
        with self._lock:
            if subsystem not in self.control:
                return {'ok': False, 'error': f'unknown subsystem "{subsystem}"'}
            control = self.control[subsystem]

            if subsystem in jc.SERVO_SUBSYSTEMS:
                infos = jc.JOINTS_BY_SUBSYSTEM[subsystem]
                for update in payload.get('joints', []):
                    index = update.get('index')
                    if not isinstance(index, int) or not 0 <= index < len(infos):
                        return {'ok': False, 'error': f'joint index {index} out of range'}
                    self._apply_joint(control.joints[index], infos[index], update)

            elif subsystem == jc.EYES:
                pwm = payload.get('eye_pwm')
                if isinstance(pwm, list) and len(pwm) == 2:
                    control.eye_pwm = [int(_clamp(float(v), 0, jc.EYE_PWM_MAX)) for v in pwm]

            elif subsystem == jc.BACK:
                back = payload.get('back', {})
                if 'position' in back:
                    control.back.position = _clamp(
                        float(back['position']), jc.BACK_MIN_POSITION, jc.BACK_MAX_POSITION)
                if 'velocity' in back:
                    control.back.velocity = _clamp(
                        float(back['velocity']), 0.0, jc.BACK_MAX_VELOCITY)
                if 'acceleration' in back:
                    control.back.acceleration = _clamp(
                        float(back['acceleration']), 0.0, jc.BACK_MAX_ACCELERATION)

            self._last_heartbeat = time.monotonic()
            self._deadman_warned = False
            return {'ok': True}

    def _apply_joint(self, target: JointTarget, info: jc.JointInfo,
                     update: Dict[str, Any]) -> None:
        if 'enabled' in update:
            target.enabled = bool(update['enabled'])
        if 'target_location' in update:
            target.location = _clamp(float(update['target_location']), info.lower, info.upper)
        if 'target_speed' in update:
            target.speed = _clamp(float(update['target_speed']), 0.0, self.max_speed)
        if 'target_acceleration' in update:
            target.acceleration = _clamp(float(update['target_acceleration']), 0.0, jc.MAX_ACCEL)
        if 'target_torque' in update:
            target.torque = _clamp(float(update['target_torque']), 0.0, jc.TORQUE_MAX)

    # ======================================================================
    # Forward loop
    # ======================================================================

    def _forward(self) -> None:
        """Publish held subsystems to the mux; enforce the operator deadman."""
        with self._lock:
            held = [s for s, c in self.control.items() if c.owned]
            if held:
                idle = time.monotonic() - self._last_heartbeat
                if idle > self.control_timeout:
                    if not self._deadman_warned:
                        self._deadman_warned = True
                        self.node.get_logger().warn(
                            f'no operator heartbeat for {idle:.1f} s - releasing control')
                    self.release_all('operator heartbeat lost')
                    return

            for subsystem in held:
                self._publish(subsystem)

    def _publish(self, subsystem: str) -> None:
        control = self.control[subsystem]

        if subsystem in (jc.LEFT_ARM, jc.RIGHT_ARM):
            msg = ArmCmd()
            for cmd, target in zip(msg.joint_cmd, control.joints):
                self._fill_servo_cmd(cmd, target)
            self.pub[subsystem].publish(msg)

        elif subsystem == jc.HEAD:
            msg = HeadCmd()
            for cmd, target in zip(msg.servos, control.joints):
                self._fill_servo_cmd(cmd, target)
            # Eyes ride their own mux channel; the mux overwrites this field
            # with the winning EyeCmd, so it is left at zero deliberately.
            msg.eye_pwm = [0, 0]
            self.pub[jc.HEAD].publish(msg)

        elif subsystem == jc.EYES:
            msg = EyeCmd()
            msg.eye_pwm = [int(v) for v in control.eye_pwm]
            self.pub[jc.EYES].publish(msg)

        elif subsystem == jc.BACK:
            msg = BackCmd()
            msg.position = float(control.back.position)
            msg.velocity = float(control.back.velocity)
            msg.acceleration = float(control.back.acceleration)
            self.pub[jc.BACK].publish(msg)

    @staticmethod
    def _fill_servo_cmd(cmd, target: JointTarget) -> None:
        cmd.enabled = bool(target.enabled)
        cmd.target_location = float(target.location)
        cmd.target_speed = float(target.speed)
        cmd.target_acceleration = float(target.acceleration)
        cmd.target_torque = float(target.torque)

    # ======================================================================
    # Register map
    # ======================================================================

    def select(self, subsystem: str, bus_id: int) -> Dict[str, Any]:
        """Point the register view at one physical servo."""
        servo = jc.bus_servo(subsystem, bus_id)
        if servo is None:
            return {'ok': False, 'error': f'no bus servo {subsystem}/{bus_id}'}
        with self._lock:
            changed = (self.selection['subsystem'], self.selection['bus_id']) != (
                subsystem, bus_id)
            self.selection = {'subsystem': subsystem, 'bus_id': bus_id}
            if changed:
                # Drop the old map rather than show one servo's registers under
                # another's name until the next poll lands.
                self._memory = None
                self._memory_stamp = 0.0
                self._memory_error = None
        return {'ok': True}

    def write_register(self, subsystem: str, bus_id: int, address: int,
                       value: int) -> Dict[str, Any]:
        """Write one register on one physical servo.

        The firmware is the authority on what may be written, how wide it is,
        and whether the lock and torque state permit it; this checks the same
        things only to give a better message before spending a bus transaction.
        Every write ends with a read-back on the module, so the returned map is
        what the servo actually holds.
        """
        register = next((r for r in jc.REGISTERS if r.address == address), None)
        if register is None or not register.writable:
            return {'ok': False, 'error': f'register 0x{address:02X} is not writable'}
        if not register.vmin <= value <= register.vmax:
            return {'ok': False,
                    'error': f'{register.label} must be {register.vmin}..{register.vmax}'}
        return self._memory_op(subsystem, bus_id, ord('w'), address, value)

    def set_lock(self, subsystem: str, bus_id: int, locked: bool) -> Dict[str, Any]:
        """Set or clear the servo's EEPROM write lock (register 0x37)."""
        return self._memory_op(subsystem, bus_id, ord('l') if locked else ord('u'), 0, 0)

    def _memory_op(self, subsystem: str, bus_id: int, operation: int,
                   address: int, value: int) -> Dict[str, Any]:
        """Run one register-service operation and fold the result into the view."""
        client = self.memory_cli.get(subsystem)
        if client is None or not client.service_is_ready():
            return {'ok': False,
                    'error': f'{jc.SUBSYSTEM_LABELS.get(subsystem, subsystem)} register '
                             f'service not available - is the module firmware flashed?'}

        request = ServoService.Request()
        request.servo = int(bus_id)
        request.operation = int(operation)
        request.address = int(address)
        request.value = int(value)

        # Writes and the poll share one bus; hold the slot so a poll cannot
        # interleave and report a pre-write map as the post-write state.
        with self._lock:
            self._memory_busy = True
        try:
            result = self._call(client, request, timeout=2.0)
        finally:
            with self._lock:
                self._memory_busy = False

        if result is None:
            return {'ok': False, 'error': 'register write timed out'}

        memory = result.memorymap
        outcome = int(memory.writebyteresult)

        if int(memory.readmemoryresult) == 1:
            with self._lock:
                if (self.selection['subsystem'], self.selection['bus_id']) == (
                        subsystem, bus_id):
                    self._memory = self._decode_memory(memory)
                    self._memory_stamp = time.monotonic()
                    self._memory_error = None

        if outcome == 0:
            # The firmware only reports "no write attempted" for a plain read, so
            # on a write verb it means the module did not recognise the verb at
            # all - i.e. it is running the earlier read-only build.
            return {'ok': False, 'code': 0,
                    'error': f'{jc.SUBSYSTEM_LABELS.get(subsystem, subsystem)} firmware '
                             f'predates the write path - reflash the module'}
        if outcome != jc.WRITE_OK:
            return {'ok': False,
                    'error': jc.WRITE_RESULTS.get(outcome, f'write failed ({outcome})'),
                    'code': outcome}
        return {'ok': True, 'message': jc.WRITE_RESULTS[jc.WRITE_OK],
                'width': int(memory.writewordresult)}

    def _poll_memory(self) -> None:
        """Refresh the selected servo's register map.

        Runs on the ROS executor. Only one read is ever in flight: the module
        serves these from its servo tick, so a backlog would just queue behind
        the bus and stale the view further.
        """
        with self._lock:
            if self._memory_busy:
                return
            subsystem = self.selection['subsystem']
            bus_id = self.selection['bus_id']
            self._memory_busy = True

        try:
            client = self.memory_cli.get(subsystem)
            if client is None or not client.service_is_ready():
                with self._lock:
                    self._memory_error = (
                        f'{jc.SUBSYSTEM_LABELS.get(subsystem, subsystem)} register '
                        f'service not available - firmware may predate it')
                    self._memory = None
                return

            request = ServoService.Request()
            request.servo = int(bus_id)
            request.operation = ord('r')
            request.address = 0
            request.value = 0

            result = self._call(client, request, timeout=1.0)
            if result is None:
                with self._lock:
                    self._memory_error = 'register read timed out'
                return

            memory = result.memorymap
            if int(memory.readmemoryresult) != 1:
                with self._lock:
                    self._memory_error = (
                        f'servo {bus_id} did not answer the register read')
                    self._memory = None
                return

            decoded = self._decode_memory(memory)
            with self._lock:
                self._memory = decoded
                self._memory_stamp = time.monotonic()
                self._memory_error = None
        finally:
            with self._lock:
                self._memory_busy = False

    @staticmethod
    def _decode_memory(memory) -> Dict[str, Any]:
        """ServoMemoryMap -> {field: int}, only the fields the UI knows about."""
        return {reg.field: int(getattr(memory, reg.field))
                for reg in jc.REGISTERS if hasattr(memory, reg.field)}

    def _memory_snapshot(self) -> Dict[str, Any]:
        servo = jc.bus_servo(self.selection['subsystem'], self.selection['bus_id'])
        age = (round(time.monotonic() - self._memory_stamp, 2)
               if self._memory_stamp else None)
        values = self._memory
        locked = None
        if values is not None and 'lockmark' in values:
            locked = values['lockmark'] != jc.LOCK_UNLOCKED
        return {
            'subsystem': self.selection['subsystem'],
            'bus_id': self.selection['bus_id'],
            'label': servo.label if servo else '',
            'joint_index': servo.joint_index if servo else None,
            'mirrored': bool(servo.mirrored) if servo else False,
            'reported': bool(servo.reported) if servo else False,
            'values': values,
            'age': age,
            'error': self._memory_error,
            'locked': locked,
            'faults': (jc.decode_status(values['servostatus'])
                       if values and 'servostatus' in values else []),
        }

    # ======================================================================
    # E-stop / services
    # ======================================================================

    def engage_estop(self) -> Dict[str, Any]:
        """Latch the mux e-stop. Everything the tool holds is released too."""
        self.estop_pub.publish(Empty())
        self.release_all('e-stop engaged from servotool3')
        self.node.get_logger().warn('E-STOP engaged from servotool3')
        return {'ok': True}

    def reset_estop(self, timeout: float = 3.0) -> Dict[str, Any]:
        if not self.estop_reset_cli.service_is_ready():
            return {'ok': False, 'error': 'estop_reset service unavailable (is command_mux up?)'}
        result = self._call(self.estop_reset_cli, Trigger.Request(), timeout)
        if result is None:
            return {'ok': False, 'error': 'estop_reset timed out'}
        return {'ok': bool(result.success), 'message': result.message}

    def calibrate_back(self, timeout: float = 5.0) -> Dict[str, Any]:
        if not self.calibrate_cli.service_is_ready():
            return {'ok': False, 'error': 'calibrate_back service unavailable'}
        result = self._call(self.calibrate_cli, BackRequestCalibration.Request(), timeout)
        if result is None:
            return {'ok': False, 'error': 'calibrate_back timed out'}
        return {'ok': bool(result.success)}

    def _call(self, client, request, timeout: float):
        """Call a service from an HTTP worker thread.

        The node is spun by a MultiThreadedExecutor elsewhere, so this polls the
        future instead of spinning (spinning here would re-enter the executor).
        """
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.result() if future.done() else None

    # ======================================================================
    # Snapshot for the web UI
    # ======================================================================

    def describe(self) -> Dict[str, Any]:
        """Static layout the UI needs once, at load."""
        return {
            'source': self.source,
            'prefix': self.prefix,
            'forward_rate_hz': self.forward_rate,
            'control_timeout': self.control_timeout,
            'subsystems': [
                {'name': s, 'label': jc.SUBSYSTEM_LABELS[s],
                 'commandable': s in jc.ALL_SUBSYSTEMS}
                for s in jc.ALL_SUBSYSTEMS
            ],
            'joints': [
                {'key': j.key, 'subsystem': j.subsystem, 'index': j.index,
                 'label': j.label, 'urdf_joint': j.urdf_joint,
                 'bus_ids': list(j.bus_ids), 'lower': j.lower, 'upper': j.upper}
                for j in jc.JOINTS
            ],
            # Physical servos, which is how the register map is addressed.
            'bus_servos': {
                subsystem: [
                    {'bus_id': s.bus_id, 'label': s.label, 'joint_index': s.joint_index,
                     'mirrored': s.mirrored, 'reported': s.reported}
                    for s in servos
                ]
                for subsystem, servos in jc.BUS_SERVOS.items()
            },
            'register_groups': [{'key': k, 'label': label}
                                for k, label in jc.REGISTER_GROUPS],
            'registers': [
                {'field': r.field, 'address': r.address, 'label': r.label,
                 'group': r.group, 'kind': r.kind, 'unit': r.unit,
                 'scale': r.scale, 'scaled_unit': r.scaled_unit, 'note': r.note,
                 'writable': r.writable, 'vmin': r.vmin, 'vmax': r.vmax}
                for r in jc.REGISTERS
            ],
            'lock': {'unlocked': jc.LOCK_UNLOCKED, 'locked': jc.LOCK_LOCKED},
            'memory_poll_hz': self.memory_poll_rate,
            'memory_writable': True,
            'limits': {
                'max_speed': self.max_speed,
                'max_accel': jc.MAX_ACCEL,
                'max_torque': jc.TORQUE_MAX,
                'eye_pwm_max': jc.EYE_PWM_MAX,
                'back': {'min_position': jc.BACK_MIN_POSITION,
                         'max_position': jc.BACK_MAX_POSITION,
                         'max_velocity': jc.BACK_MAX_VELOCITY,
                         'max_acceleration': jc.BACK_MAX_ACCELERATION},
            },
            'defaults': {
                'speed': jc.DEFAULT_SPEED,
                'acceleration': jc.DEFAULT_ACCEL,
                'torque': jc.DEFAULT_TORQUE,
            },
        }

    def snapshot(self) -> Dict[str, Any]:
        """Full live state, serialized for the SSE stream."""
        with self._lock:
            now = time.monotonic()
            modules = {}
            for subsystem in (jc.LEFT_ARM, jc.RIGHT_ARM, jc.HEAD, jc.BACK):
                meter = self.rates[subsystem]
                modules[subsystem] = {
                    'online': self._module_online(subsystem),
                    'hz': round(meter.hz(), 1),
                    'age': round(now - meter.last, 3) if meter.last is not None else None,
                }
            for subsystem in (jc.LEFT_ARM, jc.RIGHT_ARM):
                msg = self.arm_state[subsystem]
                if msg is not None:
                    modules[subsystem]['board_serial'] = ''.join(
                        f'{b:02x}' for b in msg.board_serial)
                    modules[subsystem]['side'] = ARM_SIDE_NAMES.get(
                        int(msg.arm_side), 'unknown')

            return {
                'time': time.time(),
                'estop': {'known': self.estop_engaged is not None,
                          'engaged': bool(self.estop_engaged)},
                'modules': modules,
                'joints': self._joint_snapshots(),
                'eyes': self._eye_snapshot(),
                'back': self._back_snapshot(),
                'control': {
                    name: {
                        'owned': control.owned,
                        'joints': [t.as_dict() for t in control.joints],
                        'eye_pwm': list(control.eye_pwm),
                        'back': control.back.as_dict(),
                    }
                    for name, control in self.control.items()
                },
                'memory': self._memory_snapshot(),
                'heartbeat_age': round(now - self._last_heartbeat, 2)
                if self._last_heartbeat else None,
            }

    def _joint_snapshots(self) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        cache = {s: self._servo_states(s) for s in jc.SERVO_SUBSYSTEMS}
        for info in jc.JOINTS:
            states = cache.get(info.subsystem)
            entry: Dict[str, Any] = {'key': info.key, 'online': False}
            if states is not None and info.index < len(states):
                state = states[info.index]
                status = int(state.servo_status)
                entry.update({
                    'online': self._module_online(info.subsystem),
                    'enabled': bool(state.enabled),
                    'target_location': float(state.target_location),
                    'target_speed': float(state.target_speed),
                    'target_acceleration': float(state.target_acceleration),
                    'target_torque': float(state.target_torque),
                    'current_location': float(state.current_location),
                    'current_speed': float(state.current_speed),
                    'current_load': float(state.current_load),
                    'current_temperature': int(state.current_temperature),
                    'current_voltage': float(state.current_voltage),
                    'current_current': float(state.current_current),
                    'servo_status': status,
                    'faults': jc.decode_status(status),
                    'critical': bool(status & jc.CRITICAL_STATUS_BITS),
                    'is_moving': bool(state.is_moving),
                })
            out.append(entry)
        return out

    def _eye_snapshot(self) -> Dict[str, Any]:
        if self.head_state is None:
            return {'online': False, 'state': [0, 0]}
        return {'online': self._module_online(jc.HEAD),
                'state': [int(v) for v in self.head_state.eye_state]}

    def _back_snapshot(self) -> Dict[str, Any]:
        msg = self.back_state
        if msg is None:
            return {'online': False}
        return {
            'online': self._module_online(jc.BACK),
            'board_temp': int(msg.board_temp),
            'limit_switch': bool(msg.limit_switch_triggered),
            'command_position': float(msg.command_position),
            'current_position': float(msg.current_position),
            'current_velocity': float(msg.current_velocity),
            'is_moving': bool(msg.is_moving),
            'is_calibrated': bool(msg.is_calibrated),
            'calibration_status': int(msg.calibration_status),
            'error_code': int(msg.error_code),
            'fault_latched': bool(msg.fault_latched),
            'stall_count': int(msg.stall_count),
            'stall_position': float(msg.stall_position),
            'pwm_output': int(msg.pwm_output),
        }
