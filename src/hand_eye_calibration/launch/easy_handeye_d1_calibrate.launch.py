from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    easy_handeye_pkg = get_package_share_directory('easy_handeye2')

    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/calibrate.launch.py"),
        launch_arguments={
            'name': 'd1_d435i_eih_camlink',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'base_link',
            # 末端帧切换：采用 URDF 末端 link Empty_Link6
            # 历史配置（仅保留回溯）：'robot_effector_frame': 'Joint6',
            'robot_effector_frame': 'Empty_Link6',
            'tracking_base_frame': 'camera_link',
            # 方案A：直接将 marker 帧切换为 AprilTag 帧
            # 默认采用 apriltag_ros 的标准帧命名：tag<family>:<id>
            'tracking_marker_frame': 'tag36h11:0',
        }.items(),
    )

    return LaunchDescription([
        calibrate,
    ])
