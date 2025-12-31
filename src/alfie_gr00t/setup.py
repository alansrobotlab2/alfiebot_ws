from setuptools import find_packages, setup

package_name = 'alfie_gr00t'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/data_collection.launch.py',
            'launch/capture_training.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='NVIDIA GR00T n1.6 integration for Alfie robotics platform',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'data_recorder = alfie_gr00t.nodes.data_recorder:main',
        ],
    },
)
