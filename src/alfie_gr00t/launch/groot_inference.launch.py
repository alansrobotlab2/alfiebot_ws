#!/usr/bin/env python3
"""GR00T Inference Launch File.

Launches both the GR00T N1.6 server and client nodes for on-device inference.
The server runs the TensorRT model and the client collects observations and
publishes actions to the robot.

Usage:
    # On-device inference (IPC - default, fastest)
    ros2 launch alfie_gr00t groot_inference.launch.py

    # On-device with mock server (testing without GPU/model)
    ros2 launch alfie_gr00t groot_inference.launch.py mock_mode:=true

    # Remote inference (TCP) - only launches client, server runs elsewhere
    ros2 launch alfie_gr00t groot_inference.launch.py transport:=tcp server_host:=192.168.1.50 launch_server:=false

    # Custom task
    ros2 launch alfie_gr00t groot_inference.launch.py task_description:="pick up the red can"

    # Custom model checkpoint
    ros2 launch alfie_gr00t groot_inference.launch.py model_checkpoint:=/path/to/model.pth
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('alfie_gr00t')

    # Config file paths
    client_config_file = PathJoinSubstitution([pkg_share, 'config', 'groot_client.yaml'])
    server_config_file = PathJoinSubstitution([pkg_share, 'config', 'groot_server.yaml'])

    # Launch arguments - Server control
    launch_server_arg = DeclareLaunchArgument(
        'launch_server',
        default_value='true',
        description='Launch the server node (set to false for remote server)'
    )

    # Launch arguments - Transport
    transport_arg = DeclareLaunchArgument(
        'transport',
        default_value='ipc',
        description='Transport type: "ipc" for on-device (faster), "tcp" for remote'
    )

    server_host_arg = DeclareLaunchArgument(
        'server_host',
        default_value='192.168.1.100',
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

    # Launch arguments - Model configuration
    model_checkpoint_arg = DeclareLaunchArgument(
        'model_checkpoint',
        default_value='/home/alfie/cando',
        description='Path to GR00T model checkpoint directory'
    )

    use_tensorrt_arg = DeclareLaunchArgument(
        'use_tensorrt',
        default_value='true',
        description='Use TensorRT for accelerated inference'
    )

    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='false',
        description='Run server in mock mode (testing without GPU/model)'
    )

    # Launch arguments - Inference
    task_description_arg = DeclareLaunchArgument(
        'task_description',
        default_value='find the can and pick it up',
        description='Task description for policy conditioning'
    )

    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='15',
        description='Target inference FPS (command publishing is always 100 Hz)'
    )

    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety_limits',
        default_value='true',
        description='Enable joint and velocity safety limits'
    )

    smoothing_alpha_arg = DeclareLaunchArgument(
        'action_smoothing_alpha',
        default_value='0.5',
        description='Action smoothing EMA coefficient for joints (0-1)'
    )

    action_chunk_enabled_arg = DeclareLaunchArgument(
        'action_chunk_enabled',
        default_value='true',
        description='Enable action chunking (step through 16-action horizon at training rate)'
    )

    action_chunk_size_arg = DeclareLaunchArgument(
        'action_chunk_size',
        default_value='16',
        description='Number of actions to execute from the 16-step horizon (1-16)'
    )

    latency_skip_arg = DeclareLaunchArgument(
        'latency_skip_actions',
        default_value='2',
        description='Actions to skip at chunk start for latency compensation (0=disabled)'
    )

    chunk_blend_arg = DeclareLaunchArgument(
        'chunk_blend_actions',
        default_value='4',
        description='Crossfade window (in actions) at chunk boundaries (0=disabled)'
    )

    interpolate_arg = DeclareLaunchArgument(
        'interpolate_actions',
        default_value='true',
        description='Interpolate between actions for smooth 100Hz output'
    )

    # Launch arguments - Replay mode
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='',
        description='Path to LeRobot-format dataset for replay mode (empty = normal inference)'
    )

    episode_index_arg = DeclareLaunchArgument(
        'episode_index',
        default_value='0',
        description='Episode index to replay'
    )

    # GR00T server node
    groot_server_node = Node(
        package='alfie_gr00t',
        executable='groot_server',
        name='groot_server',
        namespace='alfie',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_server')),
        parameters=[
            server_config_file,
            {
                'transport': LaunchConfiguration('transport'),
                'bind_host': '*',
                'bind_port': LaunchConfiguration('server_port'),
                'ipc_path': LaunchConfiguration('ipc_path'),
                'model_checkpoint': LaunchConfiguration('model_checkpoint'),
                'use_tensorrt': LaunchConfiguration('use_tensorrt'),
                'mock_mode': LaunchConfiguration('mock_mode'),
                'dataset_path': LaunchConfiguration('dataset_path'),
                'episode_index': LaunchConfiguration('episode_index'),
            }
        ],
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
                'target_fps': LaunchConfiguration('target_fps'),
                'enable_safety_limits': LaunchConfiguration('enable_safety_limits'),
                'action_smoothing_alpha': LaunchConfiguration('action_smoothing_alpha'),
                'action_chunk_enabled': LaunchConfiguration('action_chunk_enabled'),
                'action_chunk_size': LaunchConfiguration('action_chunk_size'),
                'latency_skip_actions': LaunchConfiguration('latency_skip_actions'),
                'chunk_blend_actions': LaunchConfiguration('chunk_blend_actions'),
                'interpolate_actions': LaunchConfiguration('interpolate_actions'),
            }
        ],
    )

    return LaunchDescription([
        # Server control
        launch_server_arg,

        # Transport arguments
        transport_arg,
        server_host_arg,
        server_port_arg,
        ipc_path_arg,

        # Model configuration
        model_checkpoint_arg,
        use_tensorrt_arg,
        mock_mode_arg,

        # Inference arguments
        task_description_arg,
        target_fps_arg,
        enable_safety_arg,
        smoothing_alpha_arg,
        action_chunk_enabled_arg,
        action_chunk_size_arg,
        latency_skip_arg,
        chunk_blend_arg,
        interpolate_arg,

        # Replay arguments
        dataset_path_arg,
        episode_index_arg,

        # Log startup info
        LogInfo(msg=['=========================================']),
        LogInfo(msg=['GR00T N1.6 Inference System']),
        LogInfo(msg=['=========================================']),
        LogInfo(msg=['Transport: ', LaunchConfiguration('transport')]),
        LogInfo(msg=['Launch Server: ', LaunchConfiguration('launch_server')]),
        LogInfo(msg=['Mock Mode: ', LaunchConfiguration('mock_mode')]),
        LogInfo(msg=['Replay Dataset: ', LaunchConfiguration('dataset_path')]),
        LogInfo(msg=['Replay Episode: ', LaunchConfiguration('episode_index')]),
        LogInfo(msg=['Task: ', LaunchConfiguration('task_description')]),
        LogInfo(msg=['Inference FPS: ', LaunchConfiguration('target_fps'), ' | Command: 100 Hz']),
        LogInfo(msg=['=========================================']),

        # Nodes
        groot_server_node,
        groot_client_node,
    ])
