from setuptools import find_packages, setup
from glob import glob

package_name = 'alfie_nanoowl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='NanoOWL open-vocabulary object detection: publishes a detection list from a camera stream.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nanoowl_node = alfie_nanoowl.nanoowl_node:main',
        ],
    },
)
