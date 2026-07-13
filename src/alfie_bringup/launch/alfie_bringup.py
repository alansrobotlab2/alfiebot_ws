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
            arguments=['serial', '--dev', '/dev/ttyAlfieD', '--baudrate', '1500000', '-v1'],
            name='microros_agent_drive',
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
            arguments=['serial', '--dev', '/dev/ttyAlfieB', '--baudrate', '1500000', '-v1'],
            name='microros_agent_back',
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
            arguments=['serial', '--dev', '/dev/ttyAlfieH', '--baudrate', '1500000', '-v1'],
            name='microros_agent_head',
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
            arguments=['serial', '--dev', '/dev/ttyAlfieL', '--baudrate', '1500000', '-v1'],
            name='microros_agent_left_arm',
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
            arguments=['serial', '--dev', '/dev/ttyAlfieR', '--baudrate', '1500000', '-v1'],
            name='microros_agent_right_arm',
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
            package='alfie_mic',
            namespace='alfie',
            executable='respeaker_control',
            name='respeaker_control_node',
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',  # Wait 5 seconds for graceful shutdown
            sigkill_timeout='10',  # Force kill after 10 seconds
            respawn=True
        ),

        Node(
            package='alfie_mic',
            namespace='alfie',
            executable='led_behavior',
            name='led_behavior_node',
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
            package='alfie_wakeword',
            namespace='alfie',
            executable='wakeword_node',
            name='wakeword_node',
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
            package='alfie_agent',
            namespace='alfie',
            executable='agent_node',
            name='agent_node',
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
                'framerate': 30,
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

        # On-demand visual room recognition service (teachable place recognition).
        # Subscribes to the wide stereo eyes, serves classify/teach over local HTTP
        # (127.0.0.1:8182) which the agent's identify_room/learn_room tools call.
        Node(
            package='alfie_room',
            namespace='alfie',
            executable='room_node',
            name='room_node',
            parameters=[{
                'http_port': 8182,
                'model_path': '/home/alfie/alfiebot_ws/models/dinov2_vits14.onnx',
                'store_path': '/home/alfie/alfiebot_ws/data/rooms/rooms.json',
                'sim_threshold': 0.55,
                'margin': 0.05,
                'vault_root': '~/obsidian',
            }],
            output='screen',
            emulate_tty=True,
            sigterm_timeout='5',
            sigkill_timeout='10',
            respawn=True
        ),

        # On-demand open-vocabulary object detection (NanoOWL / OWL-ViT). Keeps
        # the model warm and holds the latest center-eye frame, but runs inference
        # only when its nanoowl/detect service is called (by the agent's `look`
        # tool) — so it never competes with the LLM/GR00T for the GPU while idle.
        # Uses the pre-built TensorRT engine (~130 ms/call); rebuild it with
        # `python3 -m nanoowl.build_image_encoder_engine <path>` (see alfie_nanoowl
        # README for the Jetson TensorRT setup it needs).
        Node(
            package='alfie_nanoowl',
            namespace='alfie',
            executable='nanoowl_node',
            name='nanoowl_node',
            parameters=[{
                'image_topic': 'stereo_camera/left_center/image_raw/compressed',
                'image_encoder_engine':
                    '/home/alfie/nanoowl_data/owl_image_encoder_patch32.engine',
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