#!/usr/bin/env python3
"""
odom -> base_link TF broadcaster.

The mecanum firmware publishes ``nav_msgs/Odometry`` on the global ``/odom`` topic
(frame ``odom``, child ``base_link``, full holonomic twist) but does NOT broadcast the
corresponding TF. Nav2 / Isaac ROS need that transform to exist. This node republishes
the odometry pose as an ``odom`` -> ``base_link`` transform.

Mutually exclusive with the robot_localization EKF: when the EKF runs it OWNS this
transform, so launch this node OR the EKF, never both (see foundations.launch.py
``use_ekf`` arg).

Note: this system remaps ``/tf`` -> ``/alfie/tf``; apply the same remap in the launch
so the broadcast lands on the namespaced TF topic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        self.declare_parameter('odom_topic', '/odom')
        # Empty -> use the frame ids carried in the Odometry message.
        self.declare_parameter('odom_frame', '')
        self.declare_parameter('base_frame', '')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # BEST_EFFORT sub is compatible with either a reliable or best-effort publisher.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.get_logger().info(f'odom_tf_broadcaster up: "{odom_topic}" -> TF')

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame or msg.header.frame_id or 'odom'
        t.child_frame_id = self.base_frame or msg.child_frame_id or 'base_link'

        p = msg.pose.pose
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
