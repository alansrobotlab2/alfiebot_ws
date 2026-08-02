from setuptools import find_packages, setup
from glob import glob

package_name = 'alfie_wakeword'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Bundle the wake-word + oww base models so the node runs fully offline.
        # glob('models/*') so ONNX external-data weight files (*.onnx.data) come
        # along with their .onnx graph — else the model fails to load.
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    # openwakeword >= 0.5 for the wakeword_models/inference_framework API.
    install_requires=['setuptools', 'openwakeword>=0.5.0', 'numpy'],
    zip_safe=True,
    maintainer='alfie',
    maintainer_email='alansrobotlab@gmail.com',
    description='openWakeWord wake-word detection: opens the conversation window and barges in on TTS.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wakeword_node = alfie_wakeword.wakeword_node:main',
        ],
    },
)
