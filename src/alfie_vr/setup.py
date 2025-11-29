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
    ],
    package_data={
        package_name: [
            'xlevr/**/*.py',
            'web-ui/*.html',
            'web-ui/*.js',
            'web-ui/*.css',
            'web-ui/media/*',
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
            'head_tracker = alfie_vr.head_tracker:main',
        ],
    },
)
