#!/usr/bin/env python3
"""GR00T Inference Launch File.

Launches the GR00T client node. The inference server runs locally on the
Orin (see ORIN_AGX_SERVER.md) and the client connects to it over localhost TCP.

Every client launch argument is optional: when left unset the value comes from
config/groot_client.yaml, which is the single source of truth. Passing an
argument overrides the yaml for that run only.

Usage:
    # Default: everything from groot_client.yaml, server on localhost
    ros2 launch alfie_gr00t groot_inference.launch.py

    # Custom task
    ros2 launch alfie_gr00t groot_inference.launch.py task_description:="pick up the red can"

    # Sequential mode (disable overlapped inference)
    ros2 launch alfie_gr00t groot_inference.launch.py n_action_steps:=16 inference_trigger_step:=16
"""

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Client parameters that can be overridden from the command line. Defaults are
# empty so groot_client.yaml stays authoritative; an override is applied only
# when the argument is actually passed.
OVERRIDABLE_ARGS = {
    # Transport
    'transport': 'Transport type: "tcp" (required for async) or "ipc" (sync only)',
    'server_host': 'Inference server host (localhost on the Orin)',
    'server_port': 'Inference server REQ/REP port',
    'ipc_path': 'Socket path for IPC transport',
    # Async ZMQ
    'use_async_zmq': 'Use async PUSH/PULL ZMQ (true) or sync REQ/REP (false)',
    'async_push_port': 'PUSH port for async client -> server observations',
    'async_pull_port': 'PULL port for async server -> client actions',
    # Inference
    'task_description': 'Task description for policy conditioning',
    'enable_safety_limits': 'Enable joint and velocity safety limits',
    'base_smoothing_alpha': 'Base velocity EMA coefficient (1.0 = no smoothing)',
    'joint_smoothing_alpha': 'Joint position EMA coefficient (0-1)',
    'action_chunk_size': 'Action chunk size, must match training horizon (16)',
    'n_action_steps': 'Actions to execute per chunk',
    'latency_skip': 'Universal latency skip (all body parts)',
    'inference_trigger_step': 'Fire inference at this step within execution window',
    'chunk_blend_steps': 'Blend joints (not base) over N steps at chunk transitions',
    'interpolate_actions': 'Interpolate between actions for smooth 100Hz output',
}


def _launch_client(context):
    client_config_file = PathJoinSubstitution(
        [FindPackageShare('alfie_gr00t'), 'config', 'groot_client.yaml'])

    overrides = {}
    for name in OVERRIDABLE_ARGS:
        value = LaunchConfiguration(name).perform(context)
        if value == '':
            continue
        # Parse like ros2 param would, so '12' -> int, 'true' -> bool. String
        # params stay raw so a task containing ':' isn't read as a mapping.
        string_params = ('transport', 'server_host', 'ipc_path', 'task_description')
        overrides[name] = value if name in string_params else yaml.safe_load(value)

    actions = [LogInfo(msg='GR00T client: parameters from groot_client.yaml')]
    if overrides:
        actions.append(LogInfo(msg=f'GR00T client: launch overrides {overrides}'))

    actions.append(Node(
        package='alfie_gr00t',
        executable='groot_client',
        name='groot_client',
        namespace='alfie',
        output='screen',
        emulate_tty=True,
        parameters=[client_config_file, overrides] if overrides else [client_config_file],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value='', description=desc)
         for name, desc in OVERRIDABLE_ARGS.items()]
        + [OpaqueFunction(function=_launch_client)]
    )
