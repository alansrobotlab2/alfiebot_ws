#!/usr/bin/env python3
"""Run one GR00T A/B trial: move arms + head to the training start pose, activate, auto-stop.

The idle/agent nodes steer the head between trials (e.g. toward sounds), so a
trial would otherwise start with the head wherever idle left it. Training
episodes start with the head centred, so this holds the head at the training
start pose through GR00T's own mux input (priority above idle/agent), then
activates the client, which takes over the same input, and deactivates it
after --duration seconds.

    python3 ab_trial.py --duration 45
"""

import argparse
import time

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from alfie_msgs.msg import ArmCmd, HeadCmd, RobotLowState, ServoCmd

# Median pose at the start of the pick episodes, CleanTheRoom (83 episodes), rad.
# Arm order: shoulder_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper
LEFT_START = (0.00, 0.10, -1.48, -0.14, 0.00, 0.00)
RIGHT_START = (0.00, 0.12, -1.48, -0.14, 0.00, 0.00)
HEAD_START = (0.009, 0.071, -0.012)
TOL = 0.05
# (speed rad/s, accel rad/s^2, torque raw) - gentler than the client's arm speed
ARM_CFG, GRIP_CFG, HEAD_CFG = (0.8, 3.0, 1000.0), (2.0, 8.0, 200.0), (1.0, 3.0, 500.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=45.0)
    ap.add_argument('--settle-timeout', type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('ab_trial')
    best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
    control = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    head_pub = node.create_publisher(HeadCmd, '/alfie/cmd/head/groot', best_effort)
    left_pub = node.create_publisher(ArmCmd, '/alfie/cmd/left_arm/groot', best_effort)
    right_pub = node.create_publisher(ArmCmd, '/alfie/cmd/right_arm/groot', best_effort)
    act_pub = node.create_publisher(Bool, '/alfie/groot_client/activate', control)
    pose = [None]
    node.create_subscription(
        RobotLowState, '/alfie/robotlowstate',
        lambda m: pose.__setitem__(0, [m.servo_state[i].current_location for i in range(15)]),
        best_effort)

    def servo(target, cfg):
        sc = ServoCmd()
        sc.enabled = True
        sc.target_location = float(target)
        sc.target_speed, sc.target_acceleration, sc.target_torque = (float(v) for v in cfg)
        return sc

    def arm(targets):
        a = ArmCmd()
        a.joint_cmd = [servo(t, GRIP_CFG if i == 5 else ARM_CFG) for i, t in enumerate(targets)]
        return a

    left_cmd, right_cmd = arm(LEFT_START), arm(RIGHT_START)
    cmd = HeadCmd()
    cmd.servos = [servo(t, HEAD_CFG) for t in HEAD_START]
    target = list(LEFT_START) + list(RIGHT_START) + list(HEAD_START)
    tol = [0.1 if i in (5, 11) else TOL for i in range(15)]

    def hold():
        left_pub.publish(left_cmd)
        right_pub.publish(right_cmd)
        head_pub.publish(cmd)

    # Wait for the activate subscriber so the message is not lost
    t0 = time.monotonic()
    while act_pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    t0 = time.monotonic()
    settled_since = None
    while time.monotonic() - t0 < args.settle_timeout:
        hold()
        rclpy.spin_once(node, timeout_sec=0.02)
        h = pose[0]
        if h is not None and all(abs(a - b) < t for a, b, t in zip(h, target, tol)):
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since > 0.3:
                break
        else:
            settled_since = None
    h = pose[0]
    if h is not None:
        off = [(i, round(a - b, 3)) for i, (a, b, t) in enumerate(zip(h, target, tol)) if abs(a - b) >= t]
        print(f'pose before activation: arms L {[round(x, 2) for x in h[0:6]]} '
              f'R {[round(x, 2) for x in h[6:12]]} head {[round(x, 3) for x in h[12:15]]}; '
              f'{"all within tolerance" if not off else "OFF (servo, error): " + str(off)}', flush=True)
    else:
        print('pose before activation: no robotlowstate received', flush=True)

    act_pub.publish(Bool(data=True))
    print(f'ACTIVATED {time.strftime("%T")}', flush=True)
    # Keep holding the pose until the client publishes its own (~1 chunk)
    t1 = time.monotonic()
    while time.monotonic() - t1 < 0.3:
        hold()
        rclpy.spin_once(node, timeout_sec=0.02)

    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    act_pub.publish(Bool(data=False))
    t2 = time.monotonic()
    while time.monotonic() - t2 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.05)
    print(f'DEACTIVATED {time.strftime("%T")}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
