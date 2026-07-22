#!/usr/bin/env python3
"""
Nav-foundations launch (Phase 0).

Brings up ONLY the navigation-foundation plumbing — it deliberately does NOT start the
AI stack (GR00T / MLC LLM / NanoOWL) so nav mode stays compute-isolated. It LAYERS on
top of the existing hardware bringup, which must already be running to provide:
  - camera wide streams  (/alfie/stereo_camera/{left,right}_wide/image_raw/compressed)
  - wheel odometry       (/odom, from the mecanum micro-ROS firmware)
  - BNO085 telemetry     (/alfie/low/backstate)
  - robot_state_publisher (URDF TF, /alfie/tf(_static))

What this launch adds:
  1. imu_bridge            -> clean /alfie/imu from the BNO085 telemetry
  2. odom -> base_link TF  -> either a plain broadcaster OR the robot_localization EKF
  3. stereo decode + calibrated CameraInfo for the metric stereo pipeline

TF note: robot_state_publisher here remaps /tf -> /alfie/tf, so every TF-broadcasting
node below carries the same remap to join the one tree.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg = get_package_share_directory('alfie_bringup')
    calib_dir = os.path.join(pkg, 'config', 'stereo_calibration')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')

    ns = LaunchConfiguration('namespace')
    use_ekf = LaunchConfiguration('use_ekf')
    left_yaml = LaunchConfiguration('left_yaml')
    right_yaml = LaunchConfiguration('right_yaml')
    enable_stereo_decode = LaunchConfiguration('enable_stereo_decode')

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    args = [
        DeclareLaunchArgument('namespace', default_value='alfie'),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description='true: robot_localization EKF owns odom->base_link (fuses wheel + '
                        'BNO085 yaw-rate). false: plain odom_tf_broadcaster.'),
        DeclareLaunchArgument('left_yaml', default_value=os.path.join(calib_dir, 'left.yaml')),
        DeclareLaunchArgument('right_yaml', default_value=os.path.join(calib_dir, 'right.yaml')),
        DeclareLaunchArgument('enable_stereo_decode', default_value='true',
                              description='Publish calibrated stereo CameraInfo '
                                          '(gates stereo_camera_info_pub).'),
    ]

    # --- IMU bridge -------------------------------------------------------
    imu_bridge = Node(
        package='alfie_bringup', executable='imu_bridge', name='imu_bridge',
        output='screen',
    )

    # --- Odometry -> base_link TF (mutually exclusive paths) --------------
    odom_tf = Node(
        package='alfie_bringup', executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster', output='screen',
        remappings=tf_remaps,
        condition=UnlessCondition(use_ekf),
    )
    odom_cov_relay = Node(
        package='alfie_bringup', executable='odom_covariance_relay',
        name='odom_covariance_relay', output='screen',
        condition=IfCondition(use_ekf),
    )
    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_yaml],
        remappings=tf_remaps,
        condition=IfCondition(use_ekf),
    )

    # --- Calibrated stereo CameraInfo ------------------------------------
    # NOTE: the wide->raw republish nodes were removed — nothing subscribed to
    # stereo_camera/{left,right}/image_raw. Re-add them (image_transport
    # republish) when a stereo depth consumer (stereo_image_proc / ESS / cuVSLAM)
    # is actually wired in. See config/stereo_calibration/README.md.
    camera_info = Node(
        package='alfie_bringup', executable='stereo_camera_info_pub',
        name='stereo_camera_info_pub', output='screen',
        parameters=[{'left_yaml': left_yaml, 'right_yaml': right_yaml}],
        condition=IfCondition(enable_stereo_decode),
    )

    # --- Navigation frames (PLACEHOLDERS — MEASURE ON HARDWARE) -----------
    # These frames are missing from the canonical URDF. To avoid baking guessed
    # numbers into the shared alfiebot.urdf, they are published here as static
    # transforms. Replace every translation marked TODO with a real measurement.
    #
    # Constraint #7: the URDF models head_yaw as base->neck2 (whole neck yaws),
    # which contradicts the "static neck mast, only the head moves" hardware. Until
    # that is reconciled, imu_link is parented to base_link (the IMU is torso/base-
    # fixed) and the camera to head_link. Revisit parents when the URDF is fixed.
    #
    def st(name, x, y, z, yaw, pitch, roll, parent, child):
        return Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=['--x', x, '--y', y, '--z', z,
                       '--yaw', yaw, '--pitch', pitch, '--roll', roll,
                       '--frame-id', parent, '--child-frame-id', child],
            remappings=tf_remaps)

    # base_footprint: ground-projected base frame (Nav2/REP-105). TODO z = base_link
    # height above the floor.
    base_footprint = st('tf_base_footprint', '0', '0', '0.0', '0', '0', '0',
                        'base_footprint', 'base_link')
    # IMU: static neck-top mount, treated as rigid to base. TODO x y z of the BNO085.
    imu_tf = st('tf_imu_link', '0.0', '0.0', '0.0', '0', '0', '0',
                'base_link', 'imu_link')
    # Camera body frame on the head. TODO x y z (and orientation if not level).
    cam_tf = st('tf_stereo_camera_link', '0.0', '0.0', '0.0', '0', '0', '0',
                'head_link', 'stereo_camera_link')
    # Optical frames: ROS optical convention (z fwd, x right, y down) = RPY(-pi/2,0,-pi/2)
    # => yaw=-pi/2, pitch=0, roll=-pi/2. Left optical is the stereo reference (identity
    # translation); right is +baseline along the optical x axis. TODO set right x =
    # stereo baseline in metres (placeholder 0.06).
    left_optical = st('tf_left_optical', '0', '0', '0', '-1.5707963', '0', '-1.5707963',
                      'stereo_camera_link', 'left_camera_optical_frame')
    right_optical = st('tf_right_optical', '0.06', '0', '0', '-1.5707963', '0', '-1.5707963',
                       'stereo_camera_link', 'right_camera_optical_frame')

    group = GroupAction([
        PushRosNamespace(ns),
        imu_bridge,
        odom_tf, odom_cov_relay, ekf,
        camera_info,
        base_footprint, imu_tf, cam_tf, left_optical, right_optical,
    ])

    return LaunchDescription(args + [group])
