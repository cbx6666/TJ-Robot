# pyright: reportMissingImports=false
"""Select A* or Dijkstra Nav2 parameters and start the patrol-ready stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _include_selected_nav2(context, *args, **kwargs):
    planner_type = LaunchConfiguration("planner_type").perform(context).strip().lower()
    override_params_file = LaunchConfiguration("params_file").perform(context).strip()

    if planner_type not in ("astar", "dijkstra"):
        raise ValueError("planner_type must be 'astar' or 'dijkstra'")

    robot_navigation_share = get_package_share_directory("robot_navigation")
    selected_params_file = override_params_file or os.path.join(
        robot_navigation_share,
        "config",
        f"nav2_params_{planner_type}.yaml",
    )

    nav2_launch = os.path.join(robot_navigation_share, "launch", "nav2_bringup.launch.py")

    return [
        LogInfo(
            msg=[
                "Starting Nav2 patrol stack: planner_type=",
                planner_type,
                ", params_file=",
                selected_params_file,
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map": LaunchConfiguration("map"),
                "params_file": selected_params_file,
                "autostart": LaunchConfiguration("autostart"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items(),
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="robot_navigation",
                    executable="patrol_waypoints.py",
                    name="patrol_waypoints",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("start_patrol")),
                ),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "planner_type",
                default_value="astar",
                description="Global planner algorithm: astar or dijkstra.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo /clock when running in simulation.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "maps", "map.yaml"]
                ),
                description="Static map yaml file.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional explicit Nav2 params file. Empty means select by planner_type.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz for 2D Pose Estimate and Nav2 goal validation.",
            ),
            DeclareLaunchArgument(
                "start_patrol",
                default_value="false",
                description="Start the waypoint patrol client after launching Nav2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
                ),
                description="RViz config file.",
            ),
            OpaqueFunction(function=_include_selected_nav2),
        ]
    )
