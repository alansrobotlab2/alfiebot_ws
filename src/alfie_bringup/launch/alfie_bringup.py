from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='micro_ros_agent',
            namespace='',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '--baudrate', '921600', '-v4'],
            name='microros_agent_0',
            respawn=True
        ),

        Node(
            package='micro_ros_agent',
            namespace='',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB1', '--baudrate', '921600', '-v4'],
            name='microros_agent_1',
            respawn=True
        ),

        Node(
            package='alfie_tts',
            namespace='',
            executable='alfietts',
            name='tts_node',
            respawn=True
        ),

        Node(
            package='alfie_mic',
            namespace='',
            executable='audio_publisher',
            name='audio_publisher_node',
            respawn=True
        ),

        Node(
            package='alfie_asr',
            namespace='',
            executable='parakeet_asr_node',
            name='asr_node',
            respawn=True
        ),

        Node(
            package='alfie_llm',
            namespace='',
            executable='mlc_llm_serve_node',
            name='mlc_llm_node',
            respawn=True
        ),

        Node(
            package='foxglove_bridge',
            namespace='',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'send_buffer_limit': 200000000,  # 200MB (default is 10MB)
                'max_qos_depth': 2,  # Limit queue depth
                'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
            }],
            respawn=True
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('alfie_bringup'),
                    'launch',
                    'alfie_rgbd_pcl.launch.py'
                )
            ]),
        ),

    ])