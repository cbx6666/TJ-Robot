# pyright: reportMissingImports=false
"""Mock manipulation bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="robot_manipulation",
            executable="mock_pick_place_node",
            name="mock_pick_place",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
