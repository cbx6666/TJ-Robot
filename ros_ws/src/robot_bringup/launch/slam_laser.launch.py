# pyright: reportMissingImports=false
"""Laser SLAM launcher (restored map pipeline)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_slam_params = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config",
        "mapper_params_online_async.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
