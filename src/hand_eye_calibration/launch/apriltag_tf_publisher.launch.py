import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hand_eye_calibration')
    default_params = os.path.join(pkg_dir, 'config', 'apriltag_handeye.yaml')

    arg_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='AprilTag parameters YAML file'
    )
    arg_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw',
        description='Input image topic for AprilTag detector'
    )
    arg_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Input camera_info topic for AprilTag detector'
    )
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('image_rect', LaunchConfiguration('image_topic')),
            ('camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    return LaunchDescription([
        arg_params_file,
        arg_image_topic,
        arg_camera_info_topic,
        apriltag_node,
    ])
