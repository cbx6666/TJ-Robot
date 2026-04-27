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
    nav2_base_params = LaunchConfiguration("nav2_base_params")
    nav2_profile_params = LaunchConfiguration("nav2_profile_params")

    default_base_params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "nav2",
        "base.yaml",
    )
    default_profile_params = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "nav2",
        "profiles",
        "coverage_patrol.yaml",
    )
    default_bt_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees",
        "navigate_w_replanning_and_recovery.xml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("nav2_base_params", default_value=default_base_params),
            DeclareLaunchArgument("nav2_profile_params", default_value=default_profile_params),
            LogInfo(msg=["Starting Nav2 with base=", nav2_base_params]),
            LogInfo(msg=["Starting Nav2 with profile=", nav2_profile_params]),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_base_params, nav2_profile_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_base_params, nav2_profile_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[
                    nav2_base_params,
                    nav2_profile_params,
                    {"use_sim_time": use_sim_time, "default_bt_xml_filename": default_bt_xml},
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_base_params, nav2_profile_params, {"use_sim_time": use_sim_time}],
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

