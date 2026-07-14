#!/usr/bin/env python3
"""
Odometry covariance relay.

The mecanum firmware publishes ``/odom`` with an all-zero covariance. robot_localization
does not override message covariances, and a zero covariance reads as "infinitely
certain", which breaks the filter. This node subscribes to the raw ``/odom``, stamps
sensible diagonal covariances onto pose and twist, and republishes as ``/odom_cov`` for
the EKF to consume.

Only used on the EKF (``use_ekf:=true``) path. On the simple path the plain
``odom_tf_broadcaster`` consumes ``/odom`` directly and this node is not needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry


class OdomCovarianceRelay(Node):
    def __init__(self):
        super().__init__('odom_covariance_relay')

        self.declare_parameter('in_topic', '/alfie/low/odom')
        self.declare_parameter('out_topic', '/odom_cov')
        # Diagonal covariances for [x, y, z, roll, pitch, yaw].
        self.declare_parameter('pose_covariance_diagonal',
                               [0.05, 0.05, 1e6, 1e6, 1e6, 0.1])
        # Diagonal covariances for [vx, vy, vz, vroll, vpitch, vyaw].
        self.declare_parameter('twist_covariance_diagonal',
                               [0.02, 0.02, 1e6, 1e6, 1e6, 0.05])

        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self._pose_cov = self._diag6(
            self.get_parameter('pose_covariance_diagonal').get_parameter_value().double_array_value)
        self._twist_cov = self._diag6(
            self.get_parameter('twist_covariance_diagonal').get_parameter_value().double_array_value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Odometry, out_topic, qos)
        self.create_subscription(Odometry, in_topic, self._on_odom, qos)
        self.get_logger().info(f'odom_covariance_relay up: "{in_topic}" -> "{out_topic}"')

    @staticmethod
    def _diag6(vals):
        v = list(vals) if vals is not None else []
        if len(v) != 6:
            v = [0.05, 0.05, 1e6, 1e6, 1e6, 0.1]
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 6 + i] = v[i]
        return cov

    def _on_odom(self, msg: Odometry):
        msg.pose.covariance = list(self._pose_cov)
        msg.twist.covariance = list(self._twist_cov)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
