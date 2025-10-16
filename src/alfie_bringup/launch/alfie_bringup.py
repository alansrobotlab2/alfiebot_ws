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
            namespace='alfie',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '--baudrate', '921600', '-v4'],
            name='microros_agent_0',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='micro_ros_agent',
            namespace='alfie',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB1', '--baudrate', '921600', '-v4'],
            name='microros_agent_1',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_bringup',
            namespace='alfie',
            executable='master_topics',
            name='master_topics_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_tts',
            namespace='alfie',
            executable='alfietts',
            name='tts_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_mic',
            namespace='alfie',
            executable='audio_publisher',
            name='audio_publisher_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_asr',
            namespace='alfie',
            executable='parakeet_asr_node',
            name='asr_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_llm',
            namespace='alfie',
            executable='mlc_llm_serve_node',
            name='mlc_llm_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='foxglove_bridge',
            namespace='alfie',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'send_buffer_limit': 200000000,  # 200MB (default is 10MB)
                'max_qos_depth': 2,  # Limit queue depth
                'capabilities': ['clientPublish', 'connectionGraph', 'assets'],
            }],
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
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