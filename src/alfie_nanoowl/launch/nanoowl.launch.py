"""Launch the NanoOWL open-vocabulary detection node."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Use the pre-built TensorRT engine by default if it's present (~130 ms/frame vs
# ~500 ms pure-PyTorch); fall back to PyTorch if it hasn't been built yet.
DEFAULT_ENGINE = os.path.expanduser('~/nanoowl_data/owl_image_encoder_patch32.engine')

# A broad-ish default prompt so a bare "what do you see?" (no look_for) returns
# something useful; any detect call can retarget the detector at anything.
DEFAULT_PROMPT = ('a person, a face, a hand, a cup, a bottle, a phone, '
                  'a laptop, a book, a chair, a dog, a cat')


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'image_topic',
            default_value='stereo_camera/left_center/image_raw/compressed'),
        DeclareLaunchArgument('prompt', default_value=DEFAULT_PROMPT),
        DeclareLaunchArgument('threshold', default_value='0.1'),
        # Point this at a pre-built TensorRT engine for a big speedup (see README).
        DeclareLaunchArgument(
            'image_encoder_engine',
            default_value=DEFAULT_ENGINE if os.path.exists(DEFAULT_ENGINE) else ''),
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
            'image_encoder_engine': LaunchConfiguration('image_encoder_engine'),
            'publish_annotated': LaunchConfiguration('publish_annotated'),
        }],
    )

    return LaunchDescription(args + [node])
