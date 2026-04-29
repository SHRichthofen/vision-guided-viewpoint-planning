from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    easy_handeye_pkg = get_package_share_directory('easy_handeye2')

    arg_mode = DeclareLaunchArgument(
        'mode',
        default_value='calibrate',
        description='easy_handeye mode: calibrate | publish | evaluate'
    )
    arg_name = DeclareLaunchArgument('name', default_value='agx_d435i_eih_camlink')
    arg_calib_type = DeclareLaunchArgument('calibration_type', default_value='eye_in_hand')
    arg_robot_base = DeclareLaunchArgument('robot_base_frame', default_value='base_link')
    arg_robot_eff = DeclareLaunchArgument('robot_effector_frame', default_value='tcp_link')
    arg_tracking_base = DeclareLaunchArgument('tracking_base_frame', default_value='camera_link')
    arg_tracking_marker = DeclareLaunchArgument('tracking_marker_frame', default_value='tag36h11:0')

    calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/calibrate.launch.py"),
        condition=LaunchConfigurationEquals('mode', 'calibrate'),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        }.items(),
    )

    publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/publish.launch.py"),
        condition=LaunchConfigurationEquals('mode', 'publish'),
        launch_arguments={
            'name': LaunchConfiguration('name'),
        }.items(),
    )

    evaluate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{easy_handeye_pkg}/launch/evaluate.launch.py"),
        condition=LaunchConfigurationEquals('mode', 'evaluate'),
        launch_arguments={
            'name': LaunchConfiguration('name'),
        }.items(),
    )

    return LaunchDescription([
        arg_mode,
        arg_name,
        arg_calib_type,
        arg_robot_base,
        arg_robot_eff,
        arg_tracking_base,
        arg_tracking_marker,
        calibrate,
        publish,
        evaluate,
    ])
