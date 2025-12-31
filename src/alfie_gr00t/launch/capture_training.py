#!/usr/bin/env python3
"""
Capture Training Launch File

Launches VR teleoperation and data collection for capturing training demonstrations.
Also sets the camera framerate to 15fps for optimal training data collection.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    alfie_gr00t_launch_dir = os.path.join(
        get_package_share_directory('alfie_gr00t'),
        'launch'
    )

    # Alfie VR teleop node (directly instead of via launch file)
    alfie_teleop_node = Node(
        package='alfie_vr',
        namespace='alfie',
        executable='alfie_teleop_vr',
        name='alfie_teleop_vr_node',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='10',
        respawn=True,
        parameters=[{'back_height': 0.100}]
    )

    # Include alfie_gr00t data collection launch
    data_collection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(alfie_gr00t_launch_dir, 'data_collection.launch.py')
        )
    )

    # Set camera framerate to 15fps after a short delay (to ensure node is running)
    # The camera node should be already running from alfie_bringup
    set_framerate = TimerAction(
        period=2.0,  # Wait 2 seconds for nodes to start
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/alfie/stereo_camera', 'framerate', '15'],
                output='screen',
                shell=False
            )
        ]
    )

    return LaunchDescription([
        alfie_teleop_node,
        data_collection_launch,
        set_framerate,
    ])
