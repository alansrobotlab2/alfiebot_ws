"""Launch the NanoOWL open-vocabulary detection node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'image_topic',
            default_value='stereo_camera/left_center/image_raw/compressed'),
        DeclareLaunchArgument('prompt', default_value='a person, a face, a hand'),
        DeclareLaunchArgument('threshold', default_value='0.1'),
        DeclareLaunchArgument('rate_hz', default_value='5.0'),
        # Point this at a pre-built TensorRT engine for a big speedup (see README).
        DeclareLaunchArgument('image_encoder_engine', default_value=''),
        DeclareLaunchArgument('publish_annotated', default_value='false'),
    ]

    node = Node(
        package='alfie_nanoowl',
        executable='nanoowl_node',
        name='nanoowl_node',
        namespace='alfie',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'prompt': LaunchConfiguration('prompt'),
            'threshold': LaunchConfiguration('threshold'),
            'rate_hz': LaunchConfiguration('rate_hz'),
            'image_encoder_engine': LaunchConfiguration('image_encoder_engine'),
            'publish_annotated': LaunchConfiguration('publish_annotated'),
        }],
    )

    return LaunchDescription(args + [node])
