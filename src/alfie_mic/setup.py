from setuptools import find_packages, setup

package_name = 'alfie_mic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'alfie_msgs'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'audio_publisher = alfie_mic.audio_publisher:main',
        ],
    },
)
