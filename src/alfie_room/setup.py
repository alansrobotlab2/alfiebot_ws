from setuptools import find_packages, setup

package_name = 'alfie_room'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='On-demand teachable visual room recognition (DINOv2 embeddings) '
                'exposed to the agent over a small local HTTP endpoint.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'room_node = alfie_room.room_node:main',
        ],
    },
)
