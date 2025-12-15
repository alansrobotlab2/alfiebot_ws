from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'alfie_vr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/launch', glob('launch/*.yaml')),
    ],
    package_data={
        package_name: [
            'alfievr/**/*.py',
            'web-ui/*.html',
            'web-ui/*.js',
            'web-ui/*.css',
            'web-ui/media/*',
            'web-ui/libs/*',
            'cert.pem',
            'key.pem',
            'config.yaml',
        ],
    },
    install_requires=[
        'setuptools',
        'websockets',
        'numpy',
        'scipy',
        'python-socketio',
        'fastapi',
        'uvicorn',
        'aiortc',
        'aiohttp',
        'opencv-python',
        'av',
    ],
    zip_safe=False,
    maintainer='alansrobotlab',
    maintainer_email='alansrobotlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'alfie_teleop_vr = alfie_vr.alfie_teleop_vr:main',
            'ros_video_webrtc = alfie_vr.ros_video_webrtc:main',
            'ros_video_webrtc_hw = alfie_vr.ros_video_webrtc_hw:main',
        ],
    },
)
