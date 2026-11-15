#!/usr/bin/env python3
"""GR00T N1.7 inference server launch file.

Starts scripts/groot_inference_server.py as a plain process with the
Isaac-GR00T venv interpreter (torch + TensorRT live there, not in system
python), from the Isaac-GR00T directory. Every setting comes from
config/groot_server.yaml; a launch argument overrides one key for that run.

Loading the checkpoint and the TRT engines takes 1-4 minutes. The client
(groot_inference.launch.py) can start before the server is up: it idles until
activated and pings again then.

If something is already listening on the server port (a server started by
hand), this launch logs that and starts nothing, instead of crash-looping on
the bind.

Usage:
    ros2 launch alfie_gr00t groot_server.launch.py
    # fused 4-step head: only the 26k engine build has one (the 40k build is 8-step only)
    ros2 launch alfie_gr00t groot_server.launch.py denoising_steps:=4 \\
        checkpoint:=/home/alfie/Isaac-GR00T/checkpoints/alfie/checkpoint-v2-26000 \\
        trt_engine_dir:=/home/alfie/Isaac-GR00T/gr00t_trt_deployment/alfie_v2_26k_int8_rtc/engines
"""

import glob
import importlib.util
import os
import socket

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


CONFIG_KEY = 'groot_server'

# yaml key -> (CLI flag, description). Order is the order on the command line.
SERVER_ARGS = {
    'checkpoint': ('--checkpoint', 'GR00T checkpoint directory'),
    'embodiment': ('--embodiment', 'Embodiment tag'),
    'device': ('--device', 'CUDA device'),
    'trt_engine_dir': ('--trt-engine-dir', 'Full-pipeline TRT engines dir ("" = PyTorch)'),
    'denoising_steps': ('--denoising-steps', 'Denoising steps (0 = checkpoint default)'),
    'gpu_image_preprocessing': ('--gpu-image-preprocessing', 'VLM image preprocessing on GPU'),
    'host': ('--host', 'Bind host'),
    'port': ('--port', 'REQ/REP port'),
    'async_pull_port': ('--async-pull-port', 'Async PULL port (client observations)'),
    'async_push_port': ('--async-push-port', 'Async PUSH port (actions to client)'),
    'rtc': ('--rtc', 'Enable Real-Time Chunking'),
    'rtc_frozen_steps': ('--rtc-frozen-steps', 'Override frozen steps (-1 = as sent)'),
    'rtc_overlap_steps': ('--rtc-overlap-steps', 'Cap the RTC overlap (0 = as sent)'),
    'rtc_ramp_rate': ('--rtc-ramp-rate', 'RTC frozen->free ramp rate'),
    'verbose': ('--verbose', 'Verbose logging'),
}
# Plain store_true flags: present when true, absent when false.
STORE_TRUE = ('rtc', 'verbose')
# argparse BooleanOptionalAction: --flag / --no-flag.
BOOL_OPTIONAL = ('gpu_image_preprocessing',)
# Process-level settings, not server flags.
PROCESS_ARGS = {
    'isaac_groot_dir': 'Isaac-GR00T checkout (server working directory)',
    'python': 'Isaac-GR00T venv python interpreter',
}


def _load_config():
    path = os.path.join(get_package_share_directory('alfie_gr00t'),
                        'config', 'groot_server.yaml')
    with open(path) as f:
        return path, yaml.safe_load(f)[CONFIG_KEY]


def _port_in_use(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex((host if host not in ('0.0.0.0', '*') else '127.0.0.1',
                             int(port))) == 0


def _venv_env(python):
    """Environment of an activated Isaac-GR00T venv.

    The venv's pip wheels ship TensorRT, CUDA 13 and cuDNN libraries that the
    engines and torch need on LD_LIBRARY_PATH; the hand-started server had
    these from an activated shell.
    """
    venv = os.path.dirname(os.path.dirname(python))
    site = glob.glob(os.path.join(venv, 'lib', 'python3.*', 'site-packages'))
    libs = []
    if site:
        sp = site[0]
        libs = [os.path.join(sp, 'tensorrt_libs')]
        libs += sorted(glob.glob(os.path.join(sp, 'nvidia', '*', 'lib')))
        libs.append(os.path.join(sp, 'torch', 'lib'))
    cuda = sorted(glob.glob('/usr/local/cuda-1*'))
    cuda_home = cuda[-1] if cuda else '/usr/local/cuda'
    libs.append(os.path.join(cuda_home, 'lib64'))
    libs = [d for d in libs if os.path.isdir(d)]
    return {
        'VIRTUAL_ENV': venv,
        'PATH': os.path.join(venv, 'bin') + ':' + os.environ.get('PATH', ''),
        'LD_LIBRARY_PATH': ':'.join(libs + [os.environ.get('LD_LIBRARY_PATH', '')]),
        'CUDA_HOME': cuda_home,
        'CUDA_PATH': cuda_home,
    }


def _launch_server(context):
    config_path, cfg = _load_config()

    overrides = {}
    for name in list(SERVER_ARGS) + list(PROCESS_ARGS):
        value = LaunchConfiguration(name).perform(context)
        if value != '':
            # Paths stay raw strings; numbers/bools parse like yaml.
            overrides[name] = value if name in (
                'checkpoint', 'trt_engine_dir', 'isaac_groot_dir', 'python',
                'embodiment', 'device', 'host') else yaml.safe_load(value)
    cfg = {**cfg, **overrides}

    actions = [LogInfo(msg=f'GR00T server: parameters from {config_path}')]
    if overrides:
        actions.append(LogInfo(msg=f'GR00T server: launch overrides {overrides}'))

    if _port_in_use(cfg['host'], cfg['port']):
        actions.append(LogInfo(msg=(
            f'GR00T server: port {cfg["port"]} is already in use (server started '
            'by hand?); not starting another one')))
        return actions

    # The installed copy of the script (colcon build after editing it).
    script = importlib.util.find_spec('alfie_gr00t.scripts.groot_inference_server').origin

    cmd = [cfg['python'], '-u', script]
    for key, (flag, _) in SERVER_ARGS.items():
        if key not in cfg:
            continue
        value = cfg[key]
        if key in STORE_TRUE:
            if value:
                cmd.append(flag)
        elif key in BOOL_OPTIONAL:
            cmd.append(flag if value else '--no-' + flag[2:])
        elif value is not None and value != '':
            cmd += [flag, str(value)]

    actions.append(LogInfo(msg='GR00T server: ' + ' '.join(cmd)))
    actions.append(ExecuteProcess(
        cmd=cmd,
        cwd=cfg['isaac_groot_dir'],
        additional_env=_venv_env(cfg['python']),
        name='groot_server',
        output='screen',
        emulate_tty=True,
        # The server traps SIGINT/SIGTERM and tears down ZMQ + CUDA; give it time.
        sigterm_timeout='15',
        sigkill_timeout='30',
        # A crash after a multi-minute load shouldn't hot-loop the GPU.
        respawn=True,
        respawn_delay=15.0,
    ))
    return actions


def generate_launch_description():
    args = {**{k: d for k, (_, d) in SERVER_ARGS.items()}, **PROCESS_ARGS}
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value='',
                               description=f'{desc} (default: groot_server.yaml)')
         for name, desc in args.items()]
        + [OpaqueFunction(function=_launch_server)]
    )
