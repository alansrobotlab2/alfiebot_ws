from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory
import importlib.resources


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('alfie_urdf'),
        'urdf',
        'alfiebot.urdf'
    )
    
    # Get SSL certificate paths for Foxglove Bridge
    # Use the same certs as alfie_vr so browser trusts both connections
    import alfie_vr
    vr_pkg_dir = os.path.dirname(alfie_vr.__file__)
    cert_file = os.path.join(vr_pkg_dir, 'cert.pem')
    key_file = os.path.join(vr_pkg_dir, 'key.pem')
    
    # Read URDF file content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # Robot State Publisher - publishes URDF to /robot_description and TF transforms
        Node(
            package='robot_state_publisher',
            namespace='alfie',
            remappings=[
                ('joint_states', '/alfie/joint_states'),
                ('/tf', '/alfie/tf'),
                ('/tf_static', '/alfie/tf_static'),
            ],
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
            emulate_tty=True,
            respawn=True
        ),

        Node(
            package='micro_ros_agent',
            namespace='alfie',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '--baudrate', '1500000', '-v1'],
            name='microros_agent_gdb0',
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
            arguments=['serial', '--dev', '/dev/ttyUSB1', '--baudrate', '1500000', '-v1'],
            name='microros_agent_gdb1',
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
            arguments=['serial', '--dev', '/dev/ttyACM0', '--baudrate', '1500000', '-v1'],
            name='microros_agent_backdriver',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_bringup',
            namespace='alfie',
            executable='master_status',
            name='master_status_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_bringup',
            namespace='alfie',
            executable='master_cmd',
            name='master_cmd_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_bringup',
            namespace='alfie',
            executable='jetson_stats',
            name='jetson_stats_node',
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



        # Stereo USB camera - GStreamer pipeline for dual-resolution output
        # Produces 4 streams: left_wide, right_wide, left_center, right_center (all 640x480)
        Node(
            package='alfie_bringup',
            namespace='alfie',
            executable='gstreamer_camera_node',
            name='stereo_camera',
            parameters=[{
                'device': '/dev/video0',
                # Dual-resolution mode: capture at max resolution, output 4 streams
                # 'source_width': 3200,  # Max resolution for ELP H120
                # 'source_height': 1200,  # Max resolution for ELP H120
                # 'output_width': 640,   # Output resolution width
                # 'output_height': 480,  # Output resolution height
                # 'framerate': 15,
                'source_width': 1600,  # Max resolution for ELP H120
                'source_height': 600,  # Max resolution for ELP H120
                'output_width': 320,   # Output resolution width
                'output_height': 240,  # Output resolution height
                'framerate': 15,
                'jpeg_quality': 70,
                'use_hardware_accel': True,

                'flip_vertical': True,
                'camera_frame_id': 'stereo_camera_link',
            }],
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',
            sigkill_timeout='10',
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
            remappings=[
                ('/initialpose', '/alfie/initialpose'),
                ('/move_base_simple/goal', '/alfie/move_base_simple/goal'),
                ('/clicked_point', '/alfie/clicked_point'),
            ],
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

    ])