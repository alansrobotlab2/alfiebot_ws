from setuptools import find_packages, setup

package_name = 'alfie_llm'

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
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ollama_serve_node = alfie_llm.ollama_serve_node:main',
        ],
    },
)
