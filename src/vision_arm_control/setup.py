from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_arm_control'

# ROS2 ament_python: install package metadata, launch/config resources,
# and helper scripts under lib/<package> for ros2 run compatibility.
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
        # Keep script wrappers available in lib/<package>
        (
            'lib/' + package_name,
            [
                'scripts/vision_to_arm_transform',
                'scripts/arm_pose_controller',
                'scripts/target_selector',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arno',
    maintainer_email='arno@example.com',
    description='Vision-Arm Integration Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_to_arm_transform = vision_arm_control.vision_to_arm_transform:main',
            'arm_pose_controller = vision_arm_control.arm_pose_controller:main',
            'target_selector = vision_arm_control.target_selector:main',
        ],
    },
)