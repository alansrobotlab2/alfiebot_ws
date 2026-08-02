#!/usr/bin/env python3
"""
Velocity-PID step-response harness for the back actuator.

Drives a sequence of velocity steps, logs BackState, and reports rise time,
overshoot, settling time, steady-state error and PWM saturation for each.

    ros2 run ... no, just:  python3 bringup/pid_tune.py --help

HOW A VELOCITY STEP IS PRODUCED
    BackCmd.velocity is a velocity LIMIT, not a velocity command. The firmware
    runs a cascade:

        position error -> x POSITION_KP (5.0) -> desired_velocity
                       -> clamped to min(cmd.velocity, MAX_ACTUATOR_VELOCITY)
                       -> acceleration ramp (MAX_ACTUATOR_ACCELERATION)
                       -> applyVelocityPID -> PWM

    So commanding a DISTANT position with velocity=V saturates the outer
    proportional term and the inner loop sees a step to V. That is what this
    harness does: it picks a target far enough away that the outer loop stays
    saturated for the whole measurement window, then stops short of the limits.

    Consequence: the observed rise is shaped by the acceleration ramp
    (0.35 m/s^2 -> ~130 ms to reach 46 mm/s), not purely by the PID. Rise times
    below roughly 150 ms are the ramp, not the loop. Judge KP/KI by overshoot,
    settling and steady-state error instead.

WHAT IT CANNOT MEASURE
    MIN_PWM_UPWARD (100) is a floor applied to any non-zero PID output, and duty
    100 already gives ~18.7 mm/s. Commanded velocities below that cannot be
    tracked - the loop chatters between zero and the floor. Steps below
    --min-trackable are skipped with a warning rather than silently producing
    nonsense.

SAFETY
    - Refuses to run unless BackState reports is_calibrated.
    - Clamps every target inside [--floor, --ceiling], default 0.020..0.330,
      inside the firmware's own 0..0.350. There is NO top limit switch.
    - Aborts the whole run on error_code != 0 or fault_latched.
    - Ctrl-C, and any exit path, commands a stop at the current position.
"""

import argparse
import csv
import math
import signal
import sys
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from alfie_msgs.msg import BackCmd, BackState

# The firmware creates its node in this namespace (NAMESPACE in config.h).
NS = "/alfie/low"

# micro-ROS side uses best-effort for both directions; a RELIABLE subscription
# here would simply never match and the harness would sit silent forever.
QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)


@dataclass
class Sample:
    t: float
    pos: float
    vel: float
    pwm: int
    cmd_vel: float
    error_code: int


@dataclass
class StepResult:
    label: str
    target_vel: float
    direction: str
    samples: list = field(default_factory=list)
    aborted: str = ""

    def metrics(self) -> dict:
        """Step-response metrics over the driving portion of the move."""
        v = [abs(s.vel) for s in self.samples]
        t = [s.t for s in self.samples]
        tgt = abs(self.target_vel)
        if len(v) < 10 or tgt <= 0:
            return {}

        # Steady state = last 30% of the window, which is past both the
        # acceleration ramp and any settling.
        tail_start = int(len(v) * 0.7)
        tail = v[tail_start:]
        steady = sum(tail) / len(tail)

        rise = None
        for ti, vi in zip(t, v):
            if vi >= 0.9 * tgt:
                rise = ti - t[0]
                break

        peak = max(v)
        overshoot = (peak - tgt) / tgt * 100.0 if tgt else 0.0

        # Settling: last time it left a +/-5% band around its own steady value.
        band = 0.05 * steady if steady else 0.0
        settle = None
        for i in range(len(v) - 1, -1, -1):
            if abs(v[i] - steady) > band:
                settle = t[i] - t[0]
                break

        sat = sum(1 for s in self.samples
                  if abs(s.pwm) >= 219) / len(self.samples) * 100.0
        floor = sum(1 for s in self.samples
                    if 0 < abs(s.pwm) <= 100) / len(self.samples) * 100.0

        mean = steady if steady else 1.0
        ripple = math.sqrt(sum((x - steady) ** 2 for x in tail) / len(tail)) / mean * 100.0

        return {
            "steady_mms": steady * 1000.0,
            "target_mms": tgt * 1000.0,
            "sse_pct": (steady - tgt) / tgt * 100.0,
            "rise_ms": rise * 1000.0 if rise is not None else float("nan"),
            "overshoot_pct": overshoot,
            "settle_ms": settle * 1000.0 if settle is not None else 0.0,
            "ripple_pct": ripple,
            "pwm_sat_pct": sat,
            "pwm_floor_pct": floor,
        }


class Tuner(Node):
    def __init__(self, args):
        super().__init__("back_pid_tuner")
        self.args = args
        self.state = None
        self.pub = self.create_publisher(BackCmd, f"{NS}/backcmd", QOS)
        self.create_subscription(BackState, f"{NS}/backstate", self._on_state, QOS)

    def _on_state(self, msg):
        self.state = msg

    # -- helpers ----------------------------------------------------------
    def send(self, position, velocity):
        m = BackCmd()
        m.position = float(position)
        m.velocity = float(velocity)
        m.acceleration = float(self.args.accel)
        self.pub.publish(m)

    def stop(self):
        if self.state is not None:
            self.send(self.state.current_position, 0.0)
            for _ in range(5):
                self.pub.publish(BackCmd(
                    position=float(self.state.current_position),
                    velocity=0.0,
                    acceleration=float(self.args.accel)))
                rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_state(self, timeout=10.0):
        t0 = time.time()
        while rclpy.ok() and self.state is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.state is not None

    def spin_until(self, predicate, timeout):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            if predicate():
                return True
        return False

    def fault(self):
        s = self.state
        if s is None:
            return ""
        if s.error_code != 0:
            return f"error_code={s.error_code}"
        if s.fault_latched:
            return "fault_latched"
        return ""

    def goto(self, position, velocity, settle=1.0, timeout=40.0):
        """Blocking move, used for repositioning between measured steps."""
        position = min(max(position, self.args.floor), self.args.ceiling)
        self.send(position, velocity)
        ok = self.spin_until(
            lambda: (self.state is not None
                     and abs(self.state.current_position - position) < 0.004)
                    or self.fault(),
            timeout)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < settle:
            self.send(position, velocity)
            rclpy.spin_once(self, timeout_sec=0.02)
        return ok and not self.fault()

    # -- the measurement --------------------------------------------------
    def run_step(self, target_vel, up: bool) -> StepResult:
        a = self.args
        label = f"{'UP  ' if up else 'DOWN'} {target_vel*1000:5.1f} mm/s"
        res = StepResult(label=label, target_vel=target_vel,
                         direction="up" if up else "down")

        start = a.floor + a.margin if up else a.ceiling - a.margin
        if not self.goto(start, a.reposition_vel):
            res.aborted = self.fault() or "could not reach start position"
            return res

        # Target far enough that the outer P term stays saturated throughout.
        # desired_velocity = 5.0 * error, so error > target_vel/5 keeps it pinned.
        span = a.ceiling - a.floor - 2 * a.margin
        target = a.ceiling - a.margin if up else a.floor + a.margin
        needed = target_vel / 5.0
        if span < needed * 1.5:
            res.aborted = (f"travel {span*1000:.0f} mm too short to hold "
                           f"{target_vel*1000:.0f} mm/s saturated")
            return res

        t0 = time.time()
        deadline = t0 + a.step_timeout
        stop_at = abs(target - start) - needed * 1.2   # leave the saturated zone

        self.send(target, target_vel)
        while rclpy.ok() and time.time() < deadline:
            self.send(target, target_vel)
            rclpy.spin_once(self, timeout_sec=0.005)
            s = self.state
            if s is None:
                continue
            if self.fault():
                res.aborted = self.fault()
                break
            res.samples.append(Sample(
                t=time.time() - t0,
                pos=s.current_position,
                vel=s.current_velocity,
                pwm=s.pwm_output if s.pwm_output <= 127 else s.pwm_output - 256,
                cmd_vel=target_vel,
                error_code=s.error_code,
            ))
            if abs(s.current_position - start) >= stop_at:
                break

        self.stop()
        time.sleep(0.4)
        return res


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--velocities", type=float, nargs="+",
                   default=[0.020, 0.030, 0.040, 0.046],
                   help="commanded velocity caps in m/s")
    p.add_argument("--min-trackable", type=float, default=0.0187,
                   help="below this the MIN_PWM_UPWARD floor makes tracking "
                        "impossible; such steps are skipped (m/s)")
    p.add_argument("--floor", type=float, default=0.020, help="lowest position used (m)")
    p.add_argument("--ceiling", type=float, default=0.330, help="highest position used (m)")
    p.add_argument("--margin", type=float, default=0.015,
                   help="keep-out from floor/ceiling for each measured run (m)")
    p.add_argument("--accel", type=float, default=0.35, help="commanded accel (m/s^2)")
    p.add_argument("--reposition-vel", type=float, default=0.040)
    p.add_argument("--step-timeout", type=float, default=30.0)
    p.add_argument("--down", action="store_true", help="also measure descending steps")
    p.add_argument("--csv", default="", help="write raw samples to this CSV")
    args = p.parse_args()

    rclpy.init()
    node = Tuner(args)

    def bail(*_):
        node.stop()
        rclpy.shutdown()
        sys.exit(130)
    signal.signal(signal.SIGINT, bail)

    print(f"waiting for {NS}/backstate ...")
    if not node.wait_for_state():
        print("  no BackState. Is the micro-ROS agent running and the board flashed?")
        print(f"  expected topics {NS}/backstate and {NS}/backcmd")
        node.destroy_node(); rclpy.shutdown(); return 1

    if not node.state.is_calibrated:
        print("  BackState.is_calibrated is False - home the actuator first.")
        print("  The datum must come from the limit switch, or every position")
        print("  in this run is measured against an arbitrary origin.")
        node.destroy_node(); rclpy.shutdown(); return 1

    print(f"  connected. position {node.state.current_position*1000:.1f} mm\n")

    results = []
    try:
        for vel in args.velocities:
            if vel < args.min_trackable:
                print(f"SKIP {vel*1000:.1f} mm/s - below the MIN_PWM_UPWARD floor "
                      f"({args.min_trackable*1000:.1f} mm/s); the loop would chatter")
                continue
            for up in ([True, False] if args.down else [True]):
                r = node.run_step(vel, up)
                results.append(r)
                if r.aborted:
                    print(f"{r.label}  ABORTED: {r.aborted}")
                    if "error_code" in r.aborted or "fault" in r.aborted:
                        raise RuntimeError(r.aborted)
                    continue
                m = r.metrics()
                if not m:
                    print(f"{r.label}  (too few samples)")
                    continue
                print(f"{r.label} | steady {m['steady_mms']:5.1f}  "
                      f"sse {m['sse_pct']:+6.1f}%  rise {m['rise_ms']:5.0f}ms  "
                      f"over {m['overshoot_pct']:5.1f}%  settle {m['settle_ms']:5.0f}ms  "
                      f"ripple {m['ripple_pct']:4.1f}%  "
                      f"sat {m['pwm_sat_pct']:4.1f}%  floor {m['pwm_floor_pct']:4.1f}%")
    except RuntimeError as e:
        print(f"\nRUN ABORTED: {e}")
    finally:
        node.stop()

    if args.csv and results:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["label", "t_s", "pos_m", "vel_mps", "pwm", "cmd_vel_mps"])
            for r in results:
                for s in r.samples:
                    w.writerow([r.label, f"{s.t:.4f}", f"{s.pos:.5f}",
                                f"{s.vel:.5f}", s.pwm, f"{s.cmd_vel:.4f}"])
        print(f"\nraw samples -> {args.csv}")

    print("""
READING THE RESULTS
  sse      steady-state error. Persistent negative = KI too low, or the duty
           ceiling is being hit (check sat%).
  over     overshoot. High with fast settle = KP too high. High with slow
           settle = KI too high.
  ripple   RMS variation at steady state. This is the number that used to be
           meaningless: before the encoder decoder was fixed the velocity
           signal was ~3x noise. If it is high now it is real.
  sat      % of samples at the duty ceiling. Anything much above zero means
           the step is asking for more than the motor can deliver, and the
           gains below it are not being exercised.
  floor    % of samples pinned at MIN_PWM_UPWARD. Non-zero means the deadband
           floor is driving, not the PID - those samples say nothing about
           the gains.
""")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
