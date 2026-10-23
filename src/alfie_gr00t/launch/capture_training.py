#!/usr/bin/env python3
"""
Capture Training Launch File

Launches data collection for capturing training demonstrations. Requires
alfie_bringup to be running, which already provides the camera and the VR
teleop node. The camera stays at its bringup rate (30fps); rosbag_to_groot
resamples to --fps by taking the most recent frame at each sample time.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    alfie_gr00t_launch_dir = os.path.join(
        get_package_share_directory('alfie_gr00t'),
        'launch'
    )

    # Include alfie_gr00t data collection launch
    data_collection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(alfie_gr00t_launch_dir, 'data_collection.launch.py')
        )
    )

    return LaunchDescription([
        data_collection_launch,
    ])
