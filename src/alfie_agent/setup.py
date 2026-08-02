from setuptools import find_namespace_packages, setup

package_name = 'alfie_agent'

setup(
    name=package_name,
    version='0.0.0',
    # Namespace discovery so the data-only prompts/ dir (no __init__.py) is
    # declared explicitly instead of tripping setuptools' "would be ignored" warning.
    packages=find_namespace_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name + '.prompts': ['*.md']},
    include_package_data=True,
    install_requires=['setuptools', 'alfie_msgs', 'requests'],
    zip_safe=False,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='Conversation agent: ASR-to-TTS bridge with a tool-calling brain '
                '(obsidian vault tools) over the local MLC-LLM server.',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = alfie_agent.agent_node:main',
        ],
    },
)
