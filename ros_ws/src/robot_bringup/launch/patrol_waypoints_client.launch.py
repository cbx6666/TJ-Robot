# pyright: reportMissingImports=false
"""启动 NavigationManager 巡航客户端（不启动 Nav2）。

- patrol_trigger_mode=event：等待 /task/events 的 patrol_start（run_full_system 语音巡检）
- patrol_trigger_mode=auto：Nav2 就绪后立即巡（由 nav2_patrol.launch 使用）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> None:
    bringup_share = get_package_share_directory("robot_bringup")
    default_map = os.path.join(bringup_share, "maps", "map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map_yaml")
    patrol_trigger_mode = LaunchConfiguration("patrol_trigger_mode")
    task_events_topic = LaunchConfiguration("task_events_topic")

    patrol_params = {
        "use_sim_time": use_sim_time,
        "map_yaml": map_yaml,
        "patrol_trigger_mode": patrol_trigger_mode,
        "task_events_topic": task_events_topic,
        "coverage_planner_enabled": True,
        "coverage_sample_spacing_m": 1.10,
        "coverage_max_waypoints": 40,
        "waypoint_min_obstacle_distance_m": 0.35,
        "waypoint_timeout_sec": 6.0,
        "waypoint_timeout_progress_m": 0.08,
        "patrol_stuck_enabled": True,
        "patrol_stuck_min_progress_m": 0.08,
        "patrol_stuck_window_sec": 4.0,
        "patrol_stuck_cmd_required": True,
        "patrol_stuck_near_goal_tolerance_m": 0.35,
        "patrol_failed_corridor_enabled": True,
        "patrol_escape_enabled": True,
        "patrol_approach_offsets_enabled": True,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_yaml", default_value=default_map),
            DeclareLaunchArgument("patrol_trigger_mode", default_value="event"),
            DeclareLaunchArgument("task_events_topic", default_value="/task/events"),
            Node(
                package="robot_navigation",
                executable="patrol_waypoints.py",
                name="patrol_waypoints",
                output="screen",
                parameters=[patrol_params],
            ),
        ]
    )
