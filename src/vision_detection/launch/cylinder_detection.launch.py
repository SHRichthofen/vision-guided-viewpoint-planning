"""
圆柱检测启动文件

启动YOLO圆柱检测节点：
  ros2 launch vision_detection cylinder_detection.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_detection')
    model_dir = os.path.join(pkg_dir, 'models')
    
    ld = LaunchDescription()
    
    # ==================== 圆柱检测节点 ====================
    # 功能：
    #   - 订阅RealSense RGB相机
    #   - Stage 1: 圆柱检测 (YOLO)
    #   - Stage 2: 孔口检测 (YOLO + 分割)
    #   - 发布: /vision/cylinder_pose(中心点兼容接口) + /vision/selected_cylinder_semantics(真实语义接口)
    detection_node = Node(
        package='vision_detection',
        executable='cylinder_detection',
        name='cylinder_detection',
        output='screen',
        parameters=[
            {'model_dir': model_dir},
            {'enable_debug': True},
            {'publish_interval_ms': 100},
        ]
    )
    ld.add_action(detection_node)
    
    return ld
