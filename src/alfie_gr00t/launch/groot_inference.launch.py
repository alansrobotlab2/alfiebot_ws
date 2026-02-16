#!/usr/bin/env python3
"""GR00T Inference Launch File.

Launches the GR00T N1.6 client node on the Jetson. The server runs separately
on the GPU workstation via `scripts/groot_inference_server.py`.

Usage:
    # Remote inference (TCP) — default, server runs on GPU workstation
    ros2 launch alfie_gr00t groot_inference.launch.py server_host:=192.168.50.108

    # Custom task
    ros2 launch alfie_gr00t groot_inference.launch.py task_description:="pick up the red can"

    # Sequential mode (disable overlapped inference)
    ros2 launch alfie_gr00t groot_inference.launch.py n_action_steps:=16 inference_trigger_step:=16

    # On-device inference (IPC) — if server runs on same machine
    ros2 launch alfie_gr00t groot_inference.launch.py transport:=ipc
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('alfie_gr00t')

    # Config file paths
    client_config_file = PathJoinSubstitution([pkg_share, 'config', 'groot_client.yaml'])

    # Launch arguments - Transport
    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='tcp',
        description='Transport type: "ipc" for on-device (faster), "tcp" for remote'
    )

    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='192.168.50.201',
        description='Server host for TCP transport'
    )

    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='5555',
        description='Server port for TCP transport'
    )

    ipc_path_arg = DeclareLaunchArgument(
        'ipc_path',
        default_value='/tmp/groot_inference.sock',
        description='Socket path for IPC transport'
    )

    # Launch arguments - Inference
    task_description_arg = DeclareLaunchArgument(
        'task_description',
        default_value='find the can and pick it up',
        description='Task description for policy conditioning'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety_limits',
        default_value='true',
        description='Enable joint and velocity safety limits'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'action_smoothing_alpha',
        default_value='0.95',
        description='Action smoothing EMA coefficient (0-1)'
    )

    action_chunk_size_arg = DeclareLaunchArgument(
        'action_chunk_size',
        default_value='16',
        description='Action chunk size — must match training horizon (always 16)'
    )

    n_action_steps_arg = DeclareLaunchArgument(
        'n_action_steps',
        default_value='8',
        description='Actions to execute per chunk (8=overlapped, 16=sequential)'
    )

    latency_skip_arg = DeclareLaunchArgument(
        'latency_skip',
        default_value='4',
        description='Universal latency skip (all body parts, ~280ms at 15 FPS)'
    )

    inference_trigger_step_arg = DeclareLaunchArgument(
        'inference_trigger_step',
        default_value='4',
        description='Fire inference at this step within execution window'
    )

    chunk_blend_steps_arg = DeclareLaunchArgument(
        'chunk_blend_steps',
        default_value='2',
        description='Blend joints (not base) over N steps at chunk transitions'
    )

    interpolate_arg = DeclareLaunchArgument(
        'interpolate_actions',
        default_value='true',
        description='Interpolate between actions for smooth 100Hz output'
    )


    # GR00T client node
    groot_client_node = Node(
        package='alfie_gr00t',
        executable='groot_client',
        name='groot_client',
        namespace='alfie',
        output='screen',
        emulate_tty=True,
        parameters=[
            client_config_file,
            {
                'transport': LaunchConfiguration('transport'),
                'server_host': LaunchConfiguration('server_host'),
                'server_port': LaunchConfiguration('server_port'),
                'ipc_path': LaunchConfiguration('ipc_path'),
                'task_description': LaunchConfiguration('task_description'),
                'enable_safety_limits': LaunchConfiguration('enable_safety_limits'),
                'action_smoothing_alpha': LaunchConfiguration('action_smoothing_alpha'),
                'action_chunk_size': LaunchConfiguration('action_chunk_size'),
                'n_action_steps': LaunchConfiguration('n_action_steps'),
                'latency_skip': LaunchConfiguration('latency_skip'),
                'inference_trigger_step': LaunchConfiguration('inference_trigger_step'),
                'chunk_blend_steps': LaunchConfiguration('chunk_blend_steps'),
                'interpolate_actions': LaunchConfiguration('interpolate_actions'),

            }
        ],
    )

    return LaunchDescription([
        # Transport arguments
        transport_arg,
        server_host_arg,
        server_port_arg,
        ipc_path_arg,

        # Inference arguments
        task_description_arg,
        enable_safety_arg,
        smoothing_alpha_arg,
        action_chunk_size_arg,
        n_action_steps_arg,
        latency_skip_arg,
        inference_trigger_step_arg,
        chunk_blend_steps_arg,
        interpolate_arg,


        # Log startup info
        LogInfo(msg=['=========================================']),
        LogInfo(msg=['GR00T N1.6 Inference Client']),
        LogInfo(msg=['=========================================']),
        LogInfo(msg=['Transport: ', LaunchConfiguration('transport')]),
        LogInfo(msg=['Server: ', LaunchConfiguration('server_host'), ':', LaunchConfiguration('server_port')]),
        LogInfo(msg=['Task: ', LaunchConfiguration('task_description')]),
        LogInfo(msg=['n_exec=', LaunchConfiguration('n_action_steps'),
                     ' skip=', LaunchConfiguration('latency_skip'),
                     ' trigger@', LaunchConfiguration('inference_trigger_step'),
]),
        LogInfo(msg=['=========================================']),

        # Nodes
        groot_client_node,
    ])
