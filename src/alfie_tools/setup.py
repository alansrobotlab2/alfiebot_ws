from setuptools import find_packages, setup

package_name = 'alfie_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/python3.10/site-packages/' + package_name, ['alfie_tools/servotool.ui']),
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
            'servo_memory_reader = alfie_tools.servo_memory_reader:main',
            'servotool = alfie_tools.servotool:main',
        ],
    },
)
