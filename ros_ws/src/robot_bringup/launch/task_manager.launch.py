# pyright: reportMissingImports=false
"""Task manager bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_bringup")
    default_map = os.path.join(bringup_share, "maps", "map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    parsed_intent_topic = LaunchConfiguration("parsed_intent_topic")
    task_events_topic = LaunchConfiguration("task_events_topic")
    navigate_to_pose_action = LaunchConfiguration("navigate_to_pose_action")
    map_yaml = LaunchConfiguration("map_yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml",
                default_value=default_map,
                description="静态地图 yaml（与 Nav2 map:= 一致，供覆盖式巡检）",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "parsed_intent_topic",
                default_value="/interaction/parsed_intent",
            ),
            DeclareLaunchArgument("task_events_topic", default_value="/task/events"),
            DeclareLaunchArgument(
                "navigate_to_pose_action",
                default_value="navigate_to_pose",
                description="Nav2 bt_navigator 的 NavigateToPose 动作名",
            ),
            Node(
                package="robot_tasks",
                executable="task_manager_node",
                name="task_manager",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_tasks",
                executable="command_executor_node",
                name="command_executor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "parsed_intent_topic": parsed_intent_topic,
                        "task_events_topic": task_events_topic,
                        "navigate_to_pose_action": navigate_to_pose_action,
                    }
                ],
            ),
            Node(
                package="robot_tasks",
                executable="object_fetch_orchestrator_node",
                name="object_fetch_orchestrator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "task_events_topic": task_events_topic,
                        "navigate_to_pose_action": navigate_to_pose_action,
                        "target_stable_samples": 5,
                        "target_stable_max_jump_m": 0.15,
                        "target_tentative_samples": 2,
                        "target_tentative_max_jump_m": 0.45,
                        "target_point_stale_sec": 1.0,
                        "target_lost_resume_patrol_sec": 5.0,
                    }
                ],
            ),
            Node(
                package="robot_navigation",
                executable="patrol_waypoints.py",
                name="patrol_waypoints",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "map_yaml": map_yaml,
                        "patrol_trigger_mode": "event",
                        "task_events_topic": task_events_topic,
                        "nav2_ready_timeout_sec": 120.0,
                        "patrol_publish_initial_pose_on_start": False,
                        "patrol_seed_initial_pose_from_tf": True,
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
                ],
            ),
        ]
    )
