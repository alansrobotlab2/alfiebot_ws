"""
idle_behavior — low-priority "life" behaviors for Alfie.

Publishes gentle, continuous idle motion to the command_mux at the lowest motion
priority (`cmd/eyes/idle`, `cmd/head/idle`). Any real commander (VR, GR00T, agent
gaze) outranks it, so this only shows through when nothing else owns the head/eyes.
It is a pure open-loop generator — no feedback, small amplitudes.

    cmd/eyes/idle  (EyeCmd)  — slow breathing brightness
    cmd/head/idle  (HeadCmd) — small sinusoidal look-around (optional)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from alfie_msgs.msg import HeadCmd, EyeCmd, ServoCmd


class IdleBehaviorNode(Node):

    def __init__(self):
        super().__init__('idle_behavior_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # ---- Parameters -----------------------------------------------------
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('eyes_enabled', True)
        self.declare_parameter('eye_min_pwm', 200)
        self.declare_parameter('eye_max_pwm', 2500)
        self.declare_parameter('breath_period_sec', 4.0)
        self.declare_parameter('head_enabled', True)
        self.declare_parameter('head_yaw_amp_rad', 0.15)
        self.declare_parameter('head_pitch_amp_rad', 0.05)
        self.declare_parameter('head_period_sec', 12.0)
        self.declare_parameter('head_speed', 0.3)

        g = self.get_parameter
        self.rate = float(g('rate_hz').value)
        self.eyes_enabled = bool(g('eyes_enabled').value)
        self.eye_min = int(g('eye_min_pwm').value)
        self.eye_max = int(g('eye_max_pwm').value)
        self.breath_period = float(g('breath_period_sec').value)
        self.head_enabled = bool(g('head_enabled').value)
        self.head_yaw_amp = float(g('head_yaw_amp_rad').value)
        self.head_pitch_amp = float(g('head_pitch_amp_rad').value)
        self.head_period = float(g('head_period_sec').value)
        self.head_speed = float(g('head_speed').value)

        # ---- Publishers -----------------------------------------------------
        self.eyes_pub = self.create_publisher(EyeCmd, 'cmd/eyes/idle', qos)
        self.head_pub = self.create_publisher(HeadCmd, 'cmd/head/idle', qos)

        self.start_ns = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'idle_behavior started (eyes={self.eyes_enabled}, head={self.head_enabled})')

    def _elapsed(self) -> float:
        return (self.get_clock().now().nanoseconds - self.start_ns) / 1e9

    def _tick(self) -> None:
        t = self._elapsed()

        if self.eyes_enabled:
            phase = 0.5 * (1.0 + math.sin(2.0 * math.pi * t / self.breath_period))
            pwm = int(self.eye_min + (self.eye_max - self.eye_min) * phase)
            pwm = max(0, min(4095, pwm))
            eyes = EyeCmd()
            eyes.eye_pwm = [pwm, pwm]
            self.eyes_pub.publish(eyes)

        if self.head_enabled:
            yaw = self.head_yaw_amp * math.sin(2.0 * math.pi * t / self.head_period)
            pitch = self.head_pitch_amp * math.sin(
                2.0 * math.pi * t / (self.head_period * 0.7) + 1.0)
            head = HeadCmd()
            head.servos = [
                self._servo(yaw),    # 0 pan/yaw
                self._servo(pitch),  # 1 tilt/pitch
                self._servo(0.0),    # 2 roll
            ]
            head.eye_pwm = [0, 0]  # eyes travel on the eyes channel
            self.head_pub.publish(head)

    def _servo(self, location: float) -> ServoCmd:
        s = ServoCmd()
        s.enabled = True
        s.target_location = float(location)
        s.target_speed = self.head_speed
        s.target_acceleration = 0.0
        s.target_torque = 0.0
        return s


def main(args=None):
    rclpy.init(args=args)
    node = IdleBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
