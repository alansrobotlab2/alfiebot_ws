from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'alfie_urdf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        # Install mesh files
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
        ('share/' + package_name + '/meshes', glob('meshes/*.dae')),
        ('share/' + package_name + '/meshes', glob('meshes/*.obj')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='URDF and mesh files for Alfiebot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
