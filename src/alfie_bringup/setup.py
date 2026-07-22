import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'alfie_bringup'


def files(pattern):
    """glob() limited to regular files (data_files can't copy directories)."""
    return [p for p in glob(pattern) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install all launch/*.py into share/alfie_bringup/launch
        ('share/' + package_name + '/launch', files('launch/*')),
        # install config (ekf.yaml, etc.)
        ('share/' + package_name + '/config', files('config/*')),
        # install stereo calibration YAMLs / README (may be sparse until F1 is run)
        ('share/' + package_name + '/config/stereo_calibration',
            files('config/stereo_calibration/*')),
        # install SSL certificates for Foxglove Bridge TLS
        ('share/' + package_name + '/certs', files('certs/*.pem')),
        # install SSL certificates for WebRTC streaming into package directory
        ('lib/python3.10/site-packages/' + package_name, 
            [package_name + '/key.pem', package_name + '/cert.pem']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_low_status = alfie_bringup.master_low_status:main',
            'command_mux = alfie_bringup.command_mux:main',
            'idle_behavior = alfie_bringup.idle_behavior:main',
            'master_watchdog = alfie_bringup.master_watchdog:main',
            'jetson_stats = alfie_bringup.jetson_stats:main',
            'gstreamer_camera_node = alfie_bringup.gstreamer_camera_node:main',
            'gstreamer_camera_node_hw = alfie_bringup.gstreamer_camera_node_hw:main',
            'imu_bridge = alfie_bringup.imu_bridge:main',
            'odom_tf_broadcaster = alfie_bringup.odom_tf_broadcaster:main',
            'odom_covariance_relay = alfie_bringup.odom_covariance_relay:main',
            'stereo_camera_info_pub = alfie_bringup.stereo_camera_info_pub:main',
        ],
    },
)