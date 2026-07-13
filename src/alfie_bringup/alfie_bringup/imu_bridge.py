#!/usr/bin/env python3
"""
BNO085 IMU bridge.

The BNO085 (SparkFun BNO08x) is statically mounted at the top of the neck (it does
NOT move with the head) and reaches ROS today only as a ``GDBImu`` field inside the
``low/backstate`` (``alfie_msgs/BackState``) telemetry message. Isaac ROS cuVSLAM and
``robot_localization`` both want a clean, standalone ``sensor_msgs/Imu`` at a defined
frame with populated covariances.

This node subscribes to ``low/backstate``, converts ``back_state.imu`` into a proper
``sensor_msgs/Imu``, and republishes it on ``imu`` (``/alfie/imu`` under the namespace).

Frame: the IMU is rigid to the torso/base, so ``imu_link`` is defined to coincide with
the sensor's physical axes; the URDF places ``imu_link`` relative to ``base_link`` with
the real mounting pose. No axis re-mapping is applied here.

Covariances: firmware sends none, so we stamp fixed diagonal covariances from
parameters (tune per fusion needs). A negative orientation covariance[0] can be set to
signal "no absolute orientation" to consumers that honor that convention.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from alfie_msgs.msg import BackState
from sensor_msgs.msg import Imu


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.declare_parameter('backstate_topic', 'low/backstate')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('frame_id', 'imu_link')
        # Diagonal covariances (x, y, z). Defaults are rough BNO085-class values;
        # tune against the EKF / cuVSLAM. Use -1.0 for orientation[0] to mark the
        # orientation as unavailable to fusion.
        self.declare_parameter('orientation_covariance', [0.0025, 0.0025, 0.0025])
        self.declare_parameter('angular_velocity_covariance', [0.0004, 0.0004, 0.0004])
        self.declare_parameter('linear_acceleration_covariance', [0.04, 0.04, 0.04])

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        backstate_topic = self.get_parameter('backstate_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self._orient_cov = self._diag(
            self.get_parameter('orientation_covariance').get_parameter_value().double_array_value)
        self._ang_cov = self._diag(
            self.get_parameter('angular_velocity_covariance').get_parameter_value().double_array_value)
        self._lin_cov = self._diag(
            self.get_parameter('linear_acceleration_covariance').get_parameter_value().double_array_value)

        # Telemetry is published BEST_EFFORT depth-1 by the micro-ROS bridge; match it.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(Imu, imu_topic, sensor_qos)
        self.create_subscription(BackState, backstate_topic, self._on_backstate, sensor_qos)

        self._warned_empty = False
        self.get_logger().info(
            f'imu_bridge up: "{backstate_topic}".imu -> "{imu_topic}" (frame_id={self.frame_id})')

    @staticmethod
    def _diag(vals):
        """Expand a length-3 diagonal into a row-major 3x3 covariance matrix."""
        v = list(vals) if vals is not None else []
        if len(v) != 3:
            v = [0.0, 0.0, 0.0]
        return [
            v[0], 0.0, 0.0,
            0.0, v[1], 0.0,
            0.0, 0.0, v[2],
        ]

    def _on_backstate(self, msg: BackState):
        g = msg.imu
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        imu.orientation.x = g.orientation_x
        imu.orientation.y = g.orientation_y
        imu.orientation.z = g.orientation_z
        imu.orientation.w = g.orientation_w

        imu.angular_velocity.x = g.angular_velocity_x
        imu.angular_velocity.y = g.angular_velocity_y
        imu.angular_velocity.z = g.angular_velocity_z

        imu.linear_acceleration.x = g.linear_acceleration_x
        imu.linear_acceleration.y = g.linear_acceleration_y
        imu.linear_acceleration.z = g.linear_acceleration_z

        imu.orientation_covariance = list(self._orient_cov)
        imu.angular_velocity_covariance = list(self._ang_cov)
        imu.linear_acceleration_covariance = list(self._lin_cov)

        # Sanity: an all-zero quaternion + zero accel means the firmware hasn't
        # populated the BNO085 yet. Warn once so integration status is obvious.
        if (not self._warned_empty
                and g.orientation_w == 0.0 and g.orientation_x == 0.0
                and g.orientation_y == 0.0 and g.orientation_z == 0.0
                and g.linear_acceleration_z == 0.0):
            self.get_logger().warn(
                'BNO085 telemetry is all-zero — IMU may not be integrated/streaming yet.')
            self._warned_empty = True

        self.pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
