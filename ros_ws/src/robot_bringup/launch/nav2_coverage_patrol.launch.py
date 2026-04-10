# pyright: reportMissingImports=false
"""Launch Nav2 (planner/controller/bt/behaviors + costmaps) and run coverage patrol goals.

Assumptions:
- slam_toolbox is already running and publishing /map and map->odom TF
- robot publishes /odom and TF tree for base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    default_params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "nav2_params_slam.yaml",
    )
    default_bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_w_replanning_and_recovery.xml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            LogInfo(msg=["Starting Nav2 with params=", params_file]),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    params_file,
                    {"use_sim_time": use_sim_time, "default_bt_xml_filename": default_bt_xml},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "bt_navigator",
                            "behavior_server",
                        ],
                    }
                ],
            ),
            Node(
                package="robot_navigation",
                executable="coverage_patrol_nav2",
                name="coverage_patrol_nav2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )

