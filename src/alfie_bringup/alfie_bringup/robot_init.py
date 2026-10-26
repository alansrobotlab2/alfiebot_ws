"""
robot_init — one-shot startup sequence: park the head, tuck the arms, home the back.

Homing the back drives it down onto its limit switch. With the arms anywhere but
tucked that can jam a hand into the base, so the back is only touched once every
arm joint has been confirmed in the tuck pose:

    1. head   all 3 servos -> 0
    2. arms   one arm at a time (arm_order), in stages (arm_stage_order):
                wrist           gripper, wrist_roll, wrist_pitch -> tuck
                shoulder_yaw    back into the arm's own plane
                elbow           fold the forearm (shortens the arm before it swings)
                shoulder_pitch  swing the folded arm to the tuck
    3. back   calibrate (down onto the limit switch, zeroed there)
    4. back   raise to back_height

Commands go through the command_mux as source "init" (cmd/<subsystem>/init),
which outranks every other source, so nothing fights the sequence while it runs.

Stuck detection: each joint follows a ramped setpoint at reduced torque (a snag
stalls the servo rather than forcing through it). A joint that trails its
setpoint by more than stuck_tolerance for stuck_sec, or misses its goal by
settle_sec after the ramp ends, fails the sequence. So does a back fault code, a
failed calibration, a board going silent, or the e-stop engaging. On failure the
mux e-stop is latched (estop_on_failure) so everything holds where it stopped,
error_sound plays, and the node exits non-zero. The back is never calibrated after an arm failure.

On success the node stops publishing, plays startup_sound and exits 0: the head
falls back to the idle behavior, and the arm boards' 500 ms watchdog lets the arms
go limp in the tuck.

Config: alfie_bringup/config/robot_init.yaml
"""

import os
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)

from alfie_msgs.msg import ArmCmd, ArmState, BackCmd, BackState, HeadCmd, HeadState, ServoCmd
from alfie_msgs.srv import BackRequestCalibration
from std_msgs.msg import Bool, Empty


SOURCE = 'init'

# Joint order of ArmCmd/ArmState and HeadCmd/HeadState.
ARM_JOINTS = ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
              'wrist_pitch', 'wrist_roll', 'gripper')
HEAD_JOINTS = ('yaw', 'pitch', 'roll')
GRIPPER = ARM_JOINTS.index('gripper')

# arm_stage_order names -> joints moved together in that stage
ARM_STAGES = {
    'wrist': ('gripper', 'wrist_roll', 'wrist_pitch'),
    'shoulder_yaw': ('shoulder_yaw',),
    'elbow': ('elbow_pitch',),
    'shoulder_pitch': ('shoulder_pitch',),
}

ARM_SIDE = {'left': 0, 'right': 1}  # ArmState.arm_side


class InitError(RuntimeError):
    pass


class Module:
    """One servo board (an arm or the head) as driven by the init sequence."""

    def __init__(self, name: str, joints, torque: List[float]):
        self.name = name
        self.joints = joints
        self.torque = torque
        self.servos = None              # latest ServoState list
        self.stamp = 0.0                # monotonic receipt time of self.servos
        self.arm_side: Optional[int] = None
        # Commanded positions. None = not commanding (board left to its watchdog).
        # Replaced wholesale, never mutated, so the publish timer needs no lock.
        self.setpoint: Optional[List[float]] = None

    def positions(self) -> List[float]:
        return [float(s.current_location) for s in self.servos]


class RobotInitNode(Node):

    def __init__(self):
        super().__init__('robot_init_node')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('startup_timeout', 90.0)   # s, for boards + mux to appear
        self.declare_parameter('state_timeout', 0.5)      # s, board state older = board lost
        # Motion
        self.declare_parameter('head_speed', 0.6)         # rad/s ramp
        self.declare_parameter('arm_speed', 0.4)          # rad/s ramp
        self.declare_parameter('head_torque', 0.0)        # 0..1000; 0 = firmware default (full)
        self.declare_parameter('arm_torque', 500.0)       # 0..1000
        self.declare_parameter('gripper_torque', 300.0)   # 0..1000
        self.declare_parameter('arm_order', ['left', 'right'])
        self.declare_parameter('arm_stage_order',
                               ['wrist', 'shoulder_yaw', 'elbow', 'shoulder_pitch'])
        # ARM_JOINTS order: shoulder_yaw, shoulder_pitch, elbow, wrist_pitch, wrist_roll, gripper
        self.declare_parameter('left_arm_tuck', [0.0, -0.08, -1.5, 0.08, 0.0, 0.0])
        self.declare_parameter('right_arm_tuck', [0.0, 0.03, -1.5, -0.14, 0.0, 0.0])
        # Stuck detection
        self.declare_parameter('stuck_tolerance', 0.25)   # rad behind the ramp
        self.declare_parameter('stuck_sec', 0.5)
        self.declare_parameter('reach_tolerance', 0.08)   # rad from goal = arrived
        self.declare_parameter('settle_sec', 2.0)
        # Back
        self.declare_parameter('calibrate_timeout', 30.0)  # s; firmware's own limit is 20
        self.declare_parameter('back_height', 0.34)        # m; firmware clamps to 0.350
        self.declare_parameter('back_speed', 0.048)        # m/s; firmware clamps to 0.048
        self.declare_parameter('back_accel', 0.35)         # m/s^2; firmware clamps to 0.35
        self.declare_parameter('back_tolerance', 0.005)    # m
        self.declare_parameter('back_timeout_margin', 10.0)  # s on top of height / speed
        self.declare_parameter('estop_on_failure', True)
        # Played on success / failure through the default sound server sink (the
        # reSpeaker, which TTS selects). Relative = under share/alfie_bringup/resource;
        # '' = off.
        self.declare_parameter('startup_sound', 'freesound_community-start-computeraif-14572.mp3')
        self.declare_parameter('error_sound', '51481822-error-382245.mp3')

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.rate = float(g('rate_hz'))
        self.startup_timeout = float(g('startup_timeout'))
        self.state_timeout = float(g('state_timeout'))
        self.head_speed = float(g('head_speed'))
        self.arm_speed = float(g('arm_speed'))
        self.arm_order = list(g('arm_order'))
        self.arm_stage_order = list(g('arm_stage_order'))
        self.tuck = {side: [float(v) for v in g(f'{side}_arm_tuck')] for side in ARM_SIDE}
        self.stuck_tol = float(g('stuck_tolerance'))
        self.stuck_sec = float(g('stuck_sec'))
        self.reach_tol = float(g('reach_tolerance'))
        self.settle_sec = float(g('settle_sec'))
        self.calibrate_timeout = float(g('calibrate_timeout'))
        self.back_height = float(g('back_height'))
        self.back_speed = float(g('back_speed'))
        self.back_accel = float(g('back_accel'))
        self.back_tol = float(g('back_tolerance'))
        self.back_timeout_margin = float(g('back_timeout_margin'))
        self.estop_on_failure = bool(g('estop_on_failure'))
        self.startup_sound = str(g('startup_sound'))
        self.error_sound = str(g('error_sound'))

        bad = [s for s in self.arm_order if s not in ARM_SIDE]
        bad += [s for s in self.arm_stage_order if s not in ARM_STAGES]
        bad += [f'{s}_arm_tuck' for s, t in self.tuck.items() if len(t) != len(ARM_JOINTS)]
        if bad:
            raise ValueError(f'bad robot_init config: {bad}')

        arm_torque = [float(g('arm_torque'))] * len(ARM_JOINTS)
        arm_torque[GRIPPER] = float(g('gripper_torque'))
        self.modules: Dict[str, Module] = {
            'head': Module('head', HEAD_JOINTS, [float(g('head_torque'))] * len(HEAD_JOINTS)),
            'left_arm': Module('left_arm', ARM_JOINTS, list(arm_torque)),
            'right_arm': Module('right_arm', ARM_JOINTS, list(arm_torque)),
        }
        self.back: Optional[BackState] = None
        self.back_stamp = 0.0
        self.back_target: Optional[BackCmd] = None
        self.estopped: Optional[bool] = None  # None until the mux reports

        qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(HeadState, 'low/headstate',
                                 lambda m: self._on_servos('head', m.servos), qos_be)
        self.create_subscription(ArmState, 'low/left_arm/armstate',
                                 lambda m: self._on_arm('left_arm', m), qos_be)
        self.create_subscription(ArmState, 'low/right_arm/armstate',
                                 lambda m: self._on_arm('right_arm', m), qos_be)
        self.create_subscription(BackState, 'low/backstate', self._on_back, qos_be)
        self.create_subscription(Bool, 'estop_state', self._on_estop_state, qos_latched)

        self.pubs = {
            'head': self.create_publisher(HeadCmd, f'cmd/head/{SOURCE}', qos_be),
            'left_arm': self.create_publisher(ArmCmd, f'cmd/left_arm/{SOURCE}', qos_be),
            'right_arm': self.create_publisher(ArmCmd, f'cmd/right_arm/{SOURCE}', qos_be),
            'back': self.create_publisher(BackCmd, f'cmd/back/{SOURCE}', qos_be),
        }
        self.estop_pub = self.create_publisher(Empty, 'estop', qos_latched)
        self.calibrate_cli = self.create_client(BackRequestCalibration, 'low/calibrate_back')

        self.create_timer(1.0 / self.rate, self._publish)

    # ------------------------------------------------------------------
    # Callbacks (executor thread)
    # ------------------------------------------------------------------

    def _on_servos(self, name: str, servos) -> None:
        m = self.modules[name]
        m.servos = list(servos)
        m.stamp = time.monotonic()

    def _on_arm(self, name: str, msg: ArmState) -> None:
        self.modules[name].arm_side = int(msg.arm_side)
        self._on_servos(name, msg.joint_state)

    def _on_back(self, msg: BackState) -> None:
        self.back = msg
        self.back_stamp = time.monotonic()

    def _on_estop_state(self, msg: Bool) -> None:
        self.estopped = bool(msg.data)

    def _publish(self) -> None:
        for name, m in self.modules.items():
            sp = m.setpoint
            if sp is None:
                continue
            # Servo speed is a cap well above the ramp: the ramp sets the pace,
            # the cap only stops a servo from snapping if a setpoint jumps.
            speed = 2.0 * (self.head_speed if name == 'head' else self.arm_speed)
            cmds = []
            for pos, torque in zip(sp, m.torque):
                c = ServoCmd()
                c.enabled = True
                c.target_location = float(pos)
                c.target_speed = speed
                c.target_acceleration = 0.0
                c.target_torque = torque
                cmds.append(c)
            if name == 'head':
                msg = HeadCmd()
                msg.servos = cmds
            else:
                msg = ArmCmd()
                msg.joint_cmd = cmds
            self.pubs[name].publish(msg)
        if self.back_target is not None:
            self.pubs['back'].publish(self.back_target)

    # ------------------------------------------------------------------
    # Sequence (main thread)
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._wait_ready()
        # Torque on, holding wherever each joint is now.
        for m in self.modules.values():
            m.setpoint = m.positions()

        self._move('head', dict(zip(HEAD_JOINTS, [0.0] * len(HEAD_JOINTS))), self.head_speed)
        for side in self.arm_order:
            tuck = dict(zip(ARM_JOINTS, self.tuck[side]))
            for stage in self.arm_stage_order:
                goals = {j: tuck[j] for j in ARM_STAGES[stage]}
                self._move(f'{side}_arm', goals, self.arm_speed, label=stage)
        self._calibrate_back()
        self._raise_back()

    def _wait_ready(self) -> None:
        self.get_logger().info('Waiting for boards, command_mux and back calibration service...')
        deadline = time.monotonic() + self.startup_timeout
        while True:
            missing = self._not_ready()
            if not missing:
                break
            if time.monotonic() > deadline:
                raise InitError(f'not ready after {self.startup_timeout:.0f}s: {", ".join(missing)}')
            time.sleep(0.2)
        if self.estopped:
            raise InitError('e-stop is latched; reset it and rerun robot_init')

    def _not_ready(self) -> List[str]:
        missing = []
        now = time.monotonic()
        for name, m in self.modules.items():
            if m.servos is None or now - m.stamp > self.state_timeout:
                missing.append(f'{name} state')
        for side in ARM_SIDE:
            m = self.modules[f'{side}_arm']
            # The arm firmware refuses torque until it knows which side it is.
            if m.arm_side is not None and m.arm_side != ARM_SIDE[side]:
                missing.append(f'{side}_arm side (board reports {m.arm_side})')
        if self.back is None or now - self.back_stamp > self.state_timeout:
            missing.append('back state')
        for name, pub in self.pubs.items():
            # cmd/* topics appear late after bringup (discovery lag).
            if pub.get_subscription_count() == 0:
                missing.append(f'command_mux on cmd/{name}/{SOURCE}')
        if self.estopped is None:
            missing.append('estop_state')
        if not self.calibrate_cli.service_is_ready():
            missing.append('low/calibrate_back')
        return missing

    def _check(self, back: bool = False) -> None:
        """Raise if the e-stop engaged or a board went quiet."""
        if self.estopped:
            raise InitError('e-stop engaged')
        now = time.monotonic()
        for name, m in self.modules.items():
            if now - m.stamp > self.state_timeout:
                raise InitError(f'{name} state lost ({now - m.stamp:.1f}s old)')
        if back and now - self.back_stamp > self.state_timeout:
            raise InitError(f'back state lost ({now - self.back_stamp:.1f}s old)')

    def _move(self, name: str, goals: Dict[str, float], speed: float, label: str = '') -> None:
        """Ramp the given joints of one module to their goals; raise if one sticks."""
        m = self.modules[name]
        idx = {j: m.joints.index(j) for j in goals}
        start = list(m.setpoint)
        cur = m.positions()
        dist = max(abs(goals[j] - start[i]) for j, i in idx.items())
        what = f'{name} {label}'.strip()
        if all(abs(goals[j] - cur[i]) < self.reach_tol for j, i in idx.items()):
            self.get_logger().info(f'{what}: already in place')
            return
        self.get_logger().info(
            f'{what}: ' + ', '.join(f'{j} {cur[i]:+.2f}->{goals[j]:+.2f}' for j, i in idx.items()))

        # All joints in the stage start and finish together.
        duration = max(dist / speed, 0.1)
        t0 = time.monotonic()
        lagging: Dict[str, float] = {}
        period = 1.0 / self.rate
        while True:
            self._check()
            now = time.monotonic()
            s = min(1.0, (now - t0) / duration)
            sp = list(start)
            for j, i in idx.items():
                sp[i] = start[i] + s * (goals[j] - start[i])
            m.setpoint = sp

            cur = m.positions()
            for j, i in idx.items():
                err = abs(sp[i] - cur[i])
                if err <= self.stuck_tol:
                    lagging.pop(j, None)
                elif now - lagging.setdefault(j, now) > self.stuck_sec:
                    raise InitError(
                        f'{name} {j} stuck at {cur[i]:+.3f} rad (commanded {sp[i]:+.3f}, '
                        f'goal {goals[j]:+.3f}, load {m.servos[i].current_load:.0f})')

            if s >= 1.0:
                off = {j: abs(goals[j] - cur[i]) for j, i in idx.items()}
                if all(e < self.reach_tol for e in off.values()):
                    return
                if now - (t0 + duration) > self.settle_sec:
                    j = max(off, key=off.get)
                    raise InitError(
                        f'{name} {j} did not reach {goals[j]:+.3f} rad '
                        f'(at {cur[idx[j]]:+.3f}, {off[j]:.3f} off)')
            time.sleep(period)

    def _calibrate_back(self) -> None:
        self.get_logger().info('Arms tucked; calibrating back (down to limit switch)...')
        future = self.calibrate_cli.call_async(BackRequestCalibration.Request())
        # The firmware blocks for the whole descent, so back state pauses; the
        # head and arms keep holding (and being checked) meanwhile.
        deadline = time.monotonic() + self.calibrate_timeout
        while not future.done():
            self._check()
            if time.monotonic() > deadline:
                raise InitError(f'back calibration timed out after {self.calibrate_timeout:.0f}s')
            time.sleep(0.05)
        result = future.result()
        if result is None or not result.success:
            pos = self.back.current_position if self.back else float('nan')
            raise InitError(
                f'back calibration failed (stalled, obstructed or never reached the '
                f'limit switch; last position {pos:.3f} m)')

        # Wait for post-calibration state before trusting the new zero.
        t_done = time.monotonic()
        while self.back_stamp <= t_done or not self.back.is_calibrated:
            if time.monotonic() - t_done > 2.0:
                raise InitError('back reported success but never showed is_calibrated')
            time.sleep(0.02)
        self.get_logger().info(
            f'Back calibrated (position {self.back.current_position:+.3f} m, '
            f'limit switch {"on" if self.back.limit_switch_triggered else "off"})')

    def _raise_back(self) -> None:
        h = self.back_height
        self.get_logger().info(f'Raising back to {h:.3f} m...')
        cmd = BackCmd()
        cmd.position = h
        cmd.velocity = self.back_speed
        cmd.acceleration = self.back_accel
        self.back_target = cmd

        deadline = time.monotonic() + abs(h) / self.back_speed + self.back_timeout_margin
        while True:
            self._check(back=True)
            b = self.back
            if b.error_code != BackState.ERROR_NONE:
                raise InitError(
                    f'back fault {b.error_code} at {b.current_position:.3f} m '
                    f'(stall at {b.stall_position:.3f} m, {b.stall_count} stalls)')
            if abs(b.current_position - h) <= self.back_tol:
                break
            if time.monotonic() > deadline:
                raise InitError(f'back did not reach {h:.3f} m (at {b.current_position:.3f} m)')
            time.sleep(0.02)
        self.get_logger().info(f'Back at {self.back.current_position:.3f} m')

    def fail(self, err: InitError) -> None:
        """Stop where everything is now and (optionally) latch the e-stop."""
        self.get_logger().error(f'Robot init FAILED: {err}')
        for m in self.modules.values():
            if m.setpoint is not None and m.servos is not None:
                m.setpoint = m.positions()
        if self.back_target is not None and self.back is not None:
            hold = BackCmd()
            hold.position = self.back.current_position
            hold.acceleration = self.back_accel
            self.back_target = hold
        if self.estop_on_failure:
            self.get_logger().error('Latching e-stop; inspect the robot, then call estop_reset')
            self.estop_pub.publish(Empty())

    def play_sound(self, path: str) -> None:
        if not path:
            return
        if not os.path.isabs(path):
            path = os.path.join(get_package_share_directory('alfie_bringup'), 'resource', path)
        try:
            subprocess.run(['paplay', path], check=True, timeout=15.0,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        except (OSError, subprocess.SubprocessError) as e:
            err = getattr(e, 'stderr', None)
            detail = err.decode(errors='replace').strip() if err else str(e)
            self.get_logger().warn(f'Sound {path} failed: {detail}')

    def release(self) -> None:
        for m in self.modules.values():
            m.setpoint = None
        self.back_target = None


def main(args=None):
    rclpy.init(args=args)
    node = RobotInitNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    code = 0
    try:
        node.run()
        node.get_logger().info('Robot init complete')
        node.release()
        node.play_sound(node.startup_sound)
    except InitError as e:
        node.fail(e)
        code = 1
        time.sleep(0.5)  # let the hold / e-stop go out before exiting
        node.play_sound(node.error_sound)
    except KeyboardInterrupt:
        code = 130
    finally:
        node.release()
        # Join the spin thread before teardown; exiting while it is still inside
        # rclpy's wait aborts the process ("terminate called without an active exception").
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
