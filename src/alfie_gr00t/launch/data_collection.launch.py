#!/usr/bin/env python3
"""
Data Collection Launch File

Launches the data recorder node for collecting demonstration data.
Use this during teleoperation to record training datasets.

Bags are written to <data_root>/<dataset>/demonstrations/. Pick the dataset
with dataset:=<name>; it can also be changed later without relaunching:

    ros2 param set /alfie/data_recorder dataset <name>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    data_root_arg = DeclareLaunchArgument(
        'data_root',
        default_value='~/alfiebot_ws/data',
        description='Root folder holding one sub-folder per dataset'
    )

    dataset_arg = DeclareLaunchArgument(
        'dataset',
        default_value='default',
        description='Dataset name; bags go to <data_root>/<dataset>/demonstrations'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Explicit bag directory; overrides data_root/dataset when set'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start recording on launch'
    )

    max_duration_arg = DeclareLaunchArgument(
        'max_duration',
        default_value='300.0',
        description='Maximum recording duration in seconds (default: 5 minutes)'
    )

    # Data recorder node (in /alfie namespace)
    data_recorder_node = Node(
        package='alfie_gr00t',
        executable='data_recorder',
        name='data_recorder',
        namespace='alfie',
        output='screen',
        parameters=[{
            'data_root': LaunchConfiguration('data_root'),
            'dataset': LaunchConfiguration('dataset'),
            'output_dir': LaunchConfiguration('output_dir'),
            'auto_start': LaunchConfiguration('auto_start'),
            'max_duration': LaunchConfiguration('max_duration'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        data_root_arg,
        dataset_arg,
        output_dir_arg,
        auto_start_arg,
        max_duration_arg,
        LogInfo(msg=['Data collection launch started']),
        LogInfo(msg=['Dataset: ', LaunchConfiguration('dataset'),
                     '  (data root: ', LaunchConfiguration('data_root'), ')']),
        data_recorder_node,
    ])
