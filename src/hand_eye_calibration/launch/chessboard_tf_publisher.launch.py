import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hand_eye_calibration')
    config_file = os.path.join(pkg_dir, 'config', 'chessboard_tf_publisher.yaml')

    return LaunchDescription([
        Node(
            package='hand_eye_calibration',
            executable='chessboard_tf_publisher_node',
            name='chessboard_tf_publisher_node',
            output='screen',
            parameters=[config_file],
        )
    ])
