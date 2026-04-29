from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    handeye_pkg = get_package_share_directory('hand_eye_calibration')

    apriltag_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{handeye_pkg}/launch/apriltag_tf_publisher.launch.py"),
        launch_arguments={
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }.items(),
    )

    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{handeye_pkg}/launch/easy_handeye_d1_calibrate.launch.py"),
    )

    return LaunchDescription([
        apriltag_tf,
        calibrate,
    ])
