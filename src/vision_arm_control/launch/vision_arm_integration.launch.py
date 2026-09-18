"""
视觉-机械臂集成启动文件

启动顺序：
1. 圆柱检测节点（vision_detection）
2. 转换层节点（vision_to_arm_transform）
3. 控制层节点（arm_pose_controller）

数据流：
  RealSense → [vision_detection] → /vision/cylinders (PoseArray)
                                 ↓
                        [vision_detection UI选择确认]
                               ↓
                           /target_id (Int32, 确认后发布)
                                 ↓
         [vision_to_arm_transform] ← /vision/selected_cylinder_semantics
                    ↓
         /cylinder_semantics_base (base_link坐标系)
                    ↓
         [arm_pose_controller] ← /target_id
                    ↓
           /target_pose → MoveIt → Arm Execute

解决方案：使用ExecuteProcess with 'ros2 run'来启动ament_python节点
这样避免了libexec目录问题，直接通过PATH找到可执行文件。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()
    scan_distance = '0.18'  # 统一距离参数：修改这里可全局生效
    axis_observe_offset_m = '0.00'
    camera_standoff_m = '0.10'
    target_mode = 'axis_guided_view'
    roll_reference_axis = 'world_z'
    calib_file = os.path.join(
        get_package_share_directory('hand_eye_calibration'), 'calib.yaml')
    base_frame = 'base_link'
    # 按当前URDF命名约定
    effector_frame = 'Empty_Link6'
    camera_frame = 'camera_link'
    camera_optical_frame = 'camera_color_optical_frame'
    
    # ==================== 圆柱检测节点 ====================
    vision_detection_node = ExecuteProcess(
        cmd=['ros2', 'run', 'vision_detection', 'cylinder_detection',
             '--ros-args', '-p', 'model_dir:=""', '-p', 'enable_debug:=true', '-p', 'publish_interval_ms:=100',
             # 关闭每帧检测日志刷屏
             '-p', 'enable_frame_logs:=false',
             '-p', 'enable_visualization:=true', '-p', 'require_confirm_before_publish:=true',
             '-p', f'scan_distance:={scan_distance}', '-p', 'look_inward:=true'],
        output='screen',
        name='vision_detection',
    )
    ld.add_action(vision_detection_node)
    
    # ==================== 转换层节点 ====================
    vision_transform_node = ExecuteProcess(
        cmd=['ros2', 'run', 'vision_arm_control', 'vision_to_arm_transform',
             '--ros-args', '-p', f'calib_file:={calib_file}', '-p', 'enable_debug:=true',
             '-p', f'base_frame:={base_frame}', '-p', f'effector_frame:={effector_frame}', '-p', f'camera_frame:={camera_frame}',
             '-p', f'camera_optical_frame:={camera_optical_frame}', '-p', 'publish_camera_link_to_optical_tf:=true'],
        output='screen',
        name='vision_to_arm_transform',
    )
    ld.add_action(vision_transform_node)
    
    # ==================== 控制层节点 ====================
    arm_controller_node = ExecuteProcess(
        cmd=['ros2', 'run', 'vision_arm_control', 'arm_pose_controller',
                             '--ros-args', '-p', f'scan_distance:={scan_distance}', '-p', 'enable_debug:=true', '-p', f'target_frame:={base_frame}',
             '-p', f'calib_file:={calib_file}',
             '-p', 'look_inward:=true',
             # 旧模式（保留注释）：仅 normal_scan
             # '-p', 'target_mode:=legacy_normal_scan',
             '-p', f'target_mode:={target_mode}',
             '-p', f'axis_observe_offset_m:={axis_observe_offset_m}',
             '-p', f'camera_standoff_m:={camera_standoff_m}',
             '-p', f'roll_reference_axis:={roll_reference_axis}',
             '-p', f'desired_camera_frame:={camera_optical_frame}',
             '-p', f'calibration_camera_frame:={camera_frame}',
             '-p', 'use_tf_camera_frame_alignment:=true'],
        output='screen',
        name='arm_pose_controller',
    )
    ld.add_action(arm_controller_node)
    
    return ld
