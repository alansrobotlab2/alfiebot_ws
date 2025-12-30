#!/usr/bin/env python3
"""
Data Collection Launch File

Launches the data recorder node for collecting demonstration data.
Use this during teleoperation to record training datasets.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.expanduser('~/alfiebot_ws/data/demonstrations'),
        description='Directory to save demonstration recordings'
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
            'output_dir': LaunchConfiguration('output_dir'),
            'auto_start': LaunchConfiguration('auto_start'),
            'max_duration': LaunchConfiguration('max_duration'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        output_dir_arg,
        auto_start_arg,
        max_duration_arg,
        LogInfo(msg=['Data collection launch started']),
        LogInfo(msg=['Output directory: ', LaunchConfiguration('output_dir')]),
        data_recorder_node,
    ])
