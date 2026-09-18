from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_detection'

# ROS2 ament_python: install package metadata, launch/config resources,
# model weights, and helper scripts under lib/<package>.
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'models'),
            glob('vision_detection_models/*.pt'),
        ),
        # Keep script wrappers available in lib/<package>
        (
            'lib/' + package_name,
            [
                'scripts/cylinder_detection',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rongshen Yin',
    maintainer_email='yinrsh@engineering.upenn.edu',
    description='YOLO-based cylinder detection for RealSense camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cylinder_detection = vision_detection.detection_node:main',
        ],
    },
)