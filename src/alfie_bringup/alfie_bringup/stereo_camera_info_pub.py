#!/usr/bin/env python3
"""
Calibrated stereo CameraInfo publisher.

The GStreamer camera node publishes the ``left_wide`` / ``right_wide`` streams with a
FABRICATED CameraInfo template (fx = width, zero distortion) — fine for VR/NanoOWL but
useless for metric stereo. Rather than touch that heavily-optimized node, this node
loads the real stereo calibration (standard ROS ``camera_info`` YAMLs produced by
``camera_calibration``) and publishes proper ``CameraInfo`` on the nav pipeline's info
topics, restamped in lock-step with the incoming wide frames.

It keys its stamps off the compressed wide image topics (which always exist), so the
CameraInfo lines up with whatever ``image_transport republish`` emits downstream.

If a YAML is missing, it warns and stays quiet for that eye — the fabricated template
is never promoted onto the metric path.
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CompressedImage, CameraInfo


def load_camera_info(path, frame_id):
    """Parse a standard ROS camera_calibration YAML into a CameraInfo message."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    info = CameraInfo()
    info.width = int(data['image_width'])
    info.height = int(data['image_height'])
    info.header.frame_id = frame_id
    info.distortion_model = data.get('distortion_model', 'plumb_bob')
    info.k = [float(x) for x in data['camera_matrix']['data']]
    info.d = [float(x) for x in data['distortion_coefficients']['data']]
    info.r = [float(x) for x in data['rectification_matrix']['data']]
    info.p = [float(x) for x in data['projection_matrix']['data']]
    return info


class StereoCameraInfoPub(Node):
    def __init__(self):
        super().__init__('stereo_camera_info_pub')

        self.declare_parameter('left_yaml', '')
        self.declare_parameter('right_yaml', '')
        self.declare_parameter('left_image_topic',
                               'stereo_camera/left_wide/image_raw/compressed')
        self.declare_parameter('right_image_topic',
                               'stereo_camera/right_wide/image_raw/compressed')
        self.declare_parameter('left_info_topic', 'stereo_camera/left/camera_info')
        self.declare_parameter('right_info_topic', 'stereo_camera/right/camera_info')
        self.declare_parameter('left_frame_id', 'left_camera_optical_frame')
        self.declare_parameter('right_frame_id', 'right_camera_optical_frame')

        gp = lambda n: self.get_parameter(n).get_parameter_value().string_value
        left_yaml, right_yaml = gp('left_yaml'), gp('right_yaml')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._left_info = self._try_load(left_yaml, gp('left_frame_id'), 'left')
        self._right_info = self._try_load(right_yaml, gp('right_frame_id'), 'right')

        if self._left_info is not None:
            self._left_pub = self.create_publisher(CameraInfo, gp('left_info_topic'), sensor_qos)
            self.create_subscription(
                CompressedImage, gp('left_image_topic'),
                lambda m: self._publish(self._left_pub, self._left_info, m), sensor_qos)
        if self._right_info is not None:
            self._right_pub = self.create_publisher(CameraInfo, gp('right_info_topic'), sensor_qos)
            self.create_subscription(
                CompressedImage, gp('right_image_topic'),
                lambda m: self._publish(self._right_pub, self._right_info, m), sensor_qos)

        self.get_logger().info('stereo_camera_info_pub up')

    def _try_load(self, path, frame_id, side):
        if not path or not os.path.isfile(path):
            self.get_logger().warn(
                f'{side} calibration YAML missing ("{path}") — not publishing {side} CameraInfo. '
                f'Run stereo calibration first (F1).')
            return None
        try:
            info = load_camera_info(path, frame_id)
            self.get_logger().info(f'loaded {side} calibration: {path} ({info.width}x{info.height})')
            return info
        except Exception as e:  # noqa: BLE001 - surface any parse error clearly
            self.get_logger().error(f'failed to load {side} calibration "{path}": {e}')
            return None

    @staticmethod
    def _publish(pub, info, img_msg):
        info.header.stamp = img_msg.header.stamp
        pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraInfoPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
