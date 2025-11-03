from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'alfie_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install UI file to lib directory for runtime access
        ('lib/python3.10/site-packages/' + package_name + '/servotool', ['alfie_tools/servotool/servotool.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='Tools for Alfie robot servo configuration and monitoring',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_memory_reader = alfie_tools.servo_memory_reader:main',
            'servotool = alfie_tools.servotool.servotool_node:main',
            'servotool2 = alfie_tools.servotool2.servotool2_node:main',
            'joydrive = alfie_tools.joydrive.joydrive_node:main',
            'rightarmecho = alfie_tools.rightarmecho:main',
        ],
    },
)
