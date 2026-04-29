from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    easy_handeye_pkg = get_package_share_directory('easy_handeye2')

    publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/publish.launch.py"),
        launch_arguments={
            'name': 'd1_d435i_eih_camlink',
        }.items(),
    )

    return LaunchDescription([
        publish,
    ])
