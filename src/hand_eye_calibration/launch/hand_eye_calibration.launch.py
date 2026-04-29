"""
手眼标定入口（Phase B 兼容版）

说明：
  - 自研 hand_eye_calibration_node 已在 CMake 中停用
  - 本 launch 仅保留棋盘 TF 发布（供 easy_handeye2 采样）
  - 旧版启动逻辑以注释形式保留，便于后续回滚

用法：
  ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包目录和配置文件
    pkg_dir = get_package_share_directory('hand_eye_calibration')
    config_dir = os.path.join(pkg_dir, 'config')
    config_file = os.path.join(config_dir, 'chessboard_tf_publisher.yaml')

    # 旧版本（保留注释，暂不启用）：
    # config_file = os.path.join(config_dir, 'hand_eye_calibration.yaml')
    # calibration_node = Node(
    #     package='hand_eye_calibration',
    #     executable='hand_eye_calibration_node',
    #     name='hand_eye_calibration',
    #     output='screen',
    #     parameters=[config_file],
    #     remappings=[
    #         ('/tool_pose', '/tool_pose'),
    #         ('/camera/camera/color/image_raw', '/camera/camera/color/image_raw'),
    #     ]
    # )
    
    chessboard_tf_node = Node(
        package='hand_eye_calibration',
        executable='chessboard_tf_publisher_node',
        name='chessboard_tf_publisher_node',
        output='screen',
        parameters=[config_file],
    )
    
    return LaunchDescription([
        chessboard_tf_node,
    ])
