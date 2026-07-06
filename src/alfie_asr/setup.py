from setuptools import find_packages, setup

package_name = 'alfie_asr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'onnx_asr',
        'onnxruntime',
        'silero-vad',
        # torchaudio must match the installed torch (2.8.0, CUDA 12.6) — the
        # unpinned wheel (2.11.0) is built for CUDA 13 and fails to load
        # libcudart.so.13 on this Jetson.
        'torchaudio==2.8.0',
        'numpy',
    ],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parakeet_asr_node = alfie_asr.parakeet_asr_node:main',
        ],
    },
)
