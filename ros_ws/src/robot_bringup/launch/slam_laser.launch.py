# pyright: reportMissingImports=false
"""激光建图：/scan + slam_toolbox online_async；默认参数为 robot_bringup 的 full_scan.yaml（与 tb3_stack YOLO 建图时序一致）。"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    _slam_pkg = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml',
    )
    _full_scan = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'mapper_params_online_async_full_scan.yaml',
    )
    default_slam_params = _full_scan if os.path.isfile(_full_scan) else _slam_pkg

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='slam_toolbox 参数；默认同 tb3_stack 的 full_scan（/scan，与 YOLO 建图时序一致）；可改回 slam_toolbox 包内 yaml',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
