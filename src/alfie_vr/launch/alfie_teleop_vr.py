from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='alfie_vr',
            namespace='alfie',
            executable='alfie_teleop_vr',
            name='alfie_teleop_vr_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',
            sigkill_timeout='10',
            respawn=True
        ),

    ])
