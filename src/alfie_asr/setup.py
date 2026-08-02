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
        # The CUDA/TensorRT build — parakeet_asr_node asks for
        # CUDAExecutionProvider. Plain `onnxruntime` is the CPU-only wheel and
        # silently downgrades us to CPUExecutionProvider.
        'onnxruntime-gpu',
        'silero-vad',
        # Must match the installed torch, and must come from the +cu130 index
        # (download.pytorch.org/whl/cu130) — NOT the jetson-ai-lab sbsa/cu130
        # wheels. Those are built for Thor (sm_110/sm_121 only); Orin is sm_87
        # and needs the sm_80 cubins that the upstream build carries, which run
        # on any sm_8x device. Superseded the old 2.8.0/CUDA 12.6 pin when this
        # board moved to JetPack 7.2 / CUDA 13.2.
        'torchaudio==2.11.0',
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
