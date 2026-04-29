"""
重构版视觉-机械臂集成启动文件。

职责：
1. 启动检测节点（连续发布语义信息）
2. 启动相机系 -> 基座系变换节点
3. 启动单次定位测试版控制节点

说明：
- 继续沿用 ExecuteProcess + `ros2 run`，避免 ament_python libexec 路径问题。
- 参数尽量下沉到 yaml；launch 只做路径组织和少量显式覆盖。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vision_detection_share = get_package_share_directory('vision_detection')
    vision_arm_control_share = get_package_share_directory('vision_arm_control')
    handeye_share = get_package_share_directory('hand_eye_calibration')

    detection_params = os.path.join(vision_detection_share, 'config', 'cylinder_detection_refactor.params.yaml')
    controller_params = os.path.join(vision_arm_control_share, 'config', 'arm_pose_controller_bottom_scan.params.yaml')

    default_calib = os.path.join(handeye_share, 'calib.yaml')

    calib_file = LaunchConfiguration('calib_file')
    base_frame = LaunchConfiguration('base_frame')
    effector_frame = LaunchConfiguration('effector_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')
    enable_arm_pose_controller = LaunchConfiguration('enable_arm_pose_controller')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('calib_file', default_value=default_calib))
    ld.add_action(DeclareLaunchArgument('base_frame', default_value='base_link'))
    ld.add_action(DeclareLaunchArgument('effector_frame', default_value='tcp_link'))
    ld.add_action(DeclareLaunchArgument('camera_frame', default_value='camera_link'))
    ld.add_action(DeclareLaunchArgument('camera_optical_frame', default_value='camera_color_optical_frame'))
    ld.add_action(DeclareLaunchArgument('enable_arm_pose_controller', default_value='true'))

    vision_detection_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'vision_detection', 'cylinder_detection',
            '--ros-args', '--params-file', detection_params,
        ],
        output='screen',
        name='vision_detection',
    )
    ld.add_action(vision_detection_node)

    vision_transform_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'vision_arm_control', 'vision_to_arm_transform',
            '--ros-args',
            '-p', ['calib_file:=', calib_file],
            '-p', ['base_frame:=', base_frame],
            '-p', ['effector_frame:=', effector_frame],
            '-p', ['camera_frame:=', camera_frame],
            '-p', ['camera_optical_frame:=', camera_optical_frame],
            '-p', 'publish_camera_link_to_optical_tf:=true',
            '-p', 'enable_debug:=true',
        ],
        output='screen',
        name='vision_to_arm_transform',
    )
    ld.add_action(vision_transform_node)

    arm_controller_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'vision_arm_control', 'arm_pose_controller',
            '--ros-args', '--params-file', controller_params,
            '-p', ['calib_file:=', calib_file],
            '-p', ['target_frame:=', base_frame],
            '-p', ['desired_camera_frame:=', camera_optical_frame],
            '-p', ['calibration_camera_frame:=', camera_frame],
        ],
        output='screen',
        name='arm_pose_controller',
        condition=IfCondition(enable_arm_pose_controller),
    )
    ld.add_action(arm_controller_node)

    return ld
