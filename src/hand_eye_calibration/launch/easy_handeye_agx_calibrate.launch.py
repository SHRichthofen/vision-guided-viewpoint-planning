from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    easy_handeye_pkg = get_package_share_directory('easy_handeye2')

    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/calibrate.launch.py"),
        launch_arguments={
            'name': 'agx_d435i_eih_camlink',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tcp_link',
            'tracking_base_frame': 'camera_link',
            'tracking_marker_frame': 'tag36h11:0',
        }.items(),
    )

    return LaunchDescription([
        calibrate,
    ])
