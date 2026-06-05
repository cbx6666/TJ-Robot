# pyright: reportMissingImports=false
"""Mock manipulation bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pick_min_distance_m = LaunchConfiguration("pick_min_distance_m")
    pick_max_distance_m = LaunchConfiguration("pick_max_distance_m")
    pick_max_yaw_error_deg = LaunchConfiguration("pick_max_yaw_error_deg")
    pick_delay_min_sec = LaunchConfiguration("pick_delay_min_sec")
    pick_delay_max_sec = LaunchConfiguration("pick_delay_max_sec")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("pick_min_distance_m", default_value="0.0"),
        DeclareLaunchArgument("pick_max_distance_m", default_value="1.5"),
        DeclareLaunchArgument("pick_max_yaw_error_deg", default_value="90.0"),
        DeclareLaunchArgument("pick_delay_min_sec", default_value="2.0"),
        DeclareLaunchArgument("pick_delay_max_sec", default_value="5.0"),
        Node(
            package="robot_manipulation",
            executable="mock_pick_place_node",
            name="mock_pick_place",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "pick_min_distance_m": pick_min_distance_m,
                    "pick_max_distance_m": pick_max_distance_m,
                    "pick_max_yaw_error_deg": pick_max_yaw_error_deg,
                    "pick_delay_min_sec": pick_delay_min_sec,
                    "pick_delay_max_sec": pick_delay_max_sec,
                }
            ],
        ),
    ])
