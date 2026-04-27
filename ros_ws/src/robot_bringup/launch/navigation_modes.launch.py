# pyright: reportMissingImports=false
"""Navigation strategy launcher for mapping and post-map stages."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_selected(context):
    strategy = LaunchConfiguration("navigation_strategy").perform(context).strip().lower()
    use_sim_time = LaunchConfiguration("use_sim_time")
    if strategy == "point_to_point":
        return [
            Node(
                package="robot_navigation",
                executable="point_to_point",
                name="point_to_point",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    if strategy == "coverage_patrol_nav2":
        return [
            Node(
                package="robot_navigation",
                executable="coverage_patrol_nav2",
                name="coverage_patrol_nav2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    if strategy == "wall_follow_coverage":
        return [
            Node(
                package="robot_navigation",
                executable="wall_follow_coverage",
                name="wall_follow_coverage",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    # default coverage_patrol (laser-only, no Nav2)
    return [
        Node(
            package="robot_navigation",
            executable="coverage_patrol",
            name="coverage_patrol",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "navigation_strategy",
            default_value="coverage_patrol_nav2",
            description=(
                "coverage_patrol | coverage_patrol_nav2 | "
                "point_to_point | wall_follow_coverage"
            ),
        ),
        Node(
            package="robot_navigation",
            executable="navigation_command_router",
            name="navigation_command_router",
            output="screen",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "default_strategy": LaunchConfiguration("navigation_strategy"),
                }
            ],
        ),
        OpaqueFunction(function=_launch_selected),
    ])
