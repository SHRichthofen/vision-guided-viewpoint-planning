"""
统一 bringup 入口：同时拉起 AGX 官方 MoveIt2 控制链与视觉-控制链。

推荐测试方式：
  ros2 launch control vision_bottom_scan_bringup.launch.py arm_type:=piper
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    control_share = get_package_share_directory('control')
    vision_arm_control_share = get_package_share_directory('vision_arm_control')
    handeye_share = get_package_share_directory('hand_eye_calibration')

    default_calib = os.path.join(handeye_share, 'calib.yaml')

    calib_file = LaunchConfiguration('calib_file')
    base_frame = LaunchConfiguration('base_frame')
    effector_frame = LaunchConfiguration('effector_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')

    vision_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision_arm_control_share, 'launch', 'vision_arm_integration_refactor.launch.py')
        ),
        launch_arguments={
            'calib_file': calib_file,
            'base_frame': base_frame,
            'effector_frame': effector_frame,
            'camera_frame': camera_frame,
            'camera_optical_frame': camera_optical_frame,
            'enable_arm_pose_controller': LaunchConfiguration('enable_legacy_arm_pose_controller'),
        }.items(),
    )

    moveit_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'run_agx_moveit_refactor.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'can_port': LaunchConfiguration('can_port'),
            'arm_type': LaunchConfiguration('arm_type'),
            'effector_type': LaunchConfiguration('effector_type'),
            'revo2_type': LaunchConfiguration('revo2_type'),
            'tcp_offset': LaunchConfiguration('tcp_offset'),
            'follow': LaunchConfiguration('follow'),
            'enable_executor': LaunchConfiguration('enable_executor'),
            'enable_view_goal_solver': LaunchConfiguration('enable_view_goal_solver'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('arm_type', default_value='piper'),
        DeclareLaunchArgument('effector_type', default_value='none'),
        DeclareLaunchArgument('revo2_type', default_value='left'),
        DeclareLaunchArgument('tcp_offset', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument('enable_executor', default_value='true'),
        DeclareLaunchArgument('enable_view_goal_solver', default_value='false'),
        DeclareLaunchArgument('enable_legacy_arm_pose_controller', default_value='true'),
        DeclareLaunchArgument('calib_file', default_value=default_calib),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('effector_frame', default_value='tcp_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_color_optical_frame'),
        moveit_chain,
        vision_chain,
    ])
