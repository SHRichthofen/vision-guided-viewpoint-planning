import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    can_port = LaunchConfiguration('can_port')
    arm_type = LaunchConfiguration('arm_type')
    effector_type = LaunchConfiguration('effector_type')
    revo2_type = LaunchConfiguration('revo2_type')
    tcp_offset = LaunchConfiguration('tcp_offset')
    follow = LaunchConfiguration('follow')
    enable_executor = LaunchConfiguration('enable_executor')

    control_share = get_package_share_directory('control')
    agx_ctrl_share = get_package_share_directory('agx_arm_ctrl')
    executor_params_path = os.path.join(control_share, 'config', 'vision_moveit_executor.params.yaml')

    agx_moveit_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(agx_ctrl_share, 'launch', 'start_single_agx_arm_moveit.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'can_port': can_port,
            'arm_type': arm_type,
            'effector_type': effector_type,
            'revo2_type': revo2_type,
            'tcp_offset': tcp_offset,
            'follow': follow,
        }.items(),
    )

    vision_moveit_executor_node = Node(
        package='control',
        executable='vision_moveit_executor',
        namespace=namespace,
        output='screen',
        parameters=[executor_params_path],
        condition=IfCondition(enable_executor),
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('arm_type', default_value='piper'),
        DeclareLaunchArgument('effector_type', default_value='none'),
        DeclareLaunchArgument('revo2_type', default_value='left'),
        DeclareLaunchArgument('tcp_offset', default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('follow', default_value='true'),
        DeclareLaunchArgument('enable_executor', default_value='true'),
        agx_moveit_chain,
        vision_moveit_executor_node,
    ])
