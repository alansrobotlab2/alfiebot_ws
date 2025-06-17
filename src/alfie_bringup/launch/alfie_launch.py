from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='micro_ros_agent',
            namespace='',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '--baudrate', '921600', '-v4'],
            name='microros_agent_0'
        ),

        Node(
            package='micro_ros_agent',
            namespace='',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB1', '--baudrate', '921600', '-v4'],
            name='microros_agent_1'
        ),

        Node(
            package='alfie_tts',
            namespace='',
            executable='alfietts',
            name='alfietts_node'
        ),

        Node(
            package='alfie_mic',
            namespace='',
            executable='audio_publisher',
            name='audio_publisher_node'
        ),

        Node(
            package='alfie_asr',
            namespace='',
            executable='asr_node',
            name='asr_node'
        ),

    ])