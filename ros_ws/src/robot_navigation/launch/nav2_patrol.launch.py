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
from launch_ros.parameter_descriptions import ParameterValue
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
        # nav2_bringup.launch.py 内部会在 localization 后再延迟启动 navigation 节点。
        # 巡航节点如果太早启动，会卡在等待 lifecycle active 的入口阶段，看起来像“机器人不动”。
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="robot_navigation",
                    executable="coverage_monitor.py",
                    name="coverage_monitor",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": ParameterValue(
                                LaunchConfiguration("use_sim_time"), value_type=bool
                            ),
                            "map_topic": LaunchConfiguration("coverage_map_topic"),
                            "scan_topic": LaunchConfiguration("coverage_scan_topic"),
                            "required_coverage_percent": ParameterValue(
                                LaunchConfiguration("coverage_required_percent"),
                                value_type=float,
                            ),
                        }
                    ],
                    condition=IfCondition(LaunchConfiguration("start_coverage_monitor")),
                ),
                Node(
                    package="robot_navigation",
                    executable="patrol_waypoints.py",
                    name="patrol_waypoints",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": ParameterValue(
                                LaunchConfiguration("use_sim_time"), value_type=bool
                            ),
                            "map_yaml": LaunchConfiguration("map"),
                            "waypoint_timeout_sec": ParameterValue(
                                LaunchConfiguration("waypoint_timeout_sec"),
                                value_type=float,
                            ),
                            "waypoint_timeout_progress_m": ParameterValue(
                                LaunchConfiguration("waypoint_timeout_progress_m"),
                                value_type=float,
                            ),
                            "patrol_stuck_enabled": ParameterValue(
                                LaunchConfiguration("patrol_stuck_enabled"),
                                value_type=bool,
                            ),
                            "patrol_stuck_min_progress_m": ParameterValue(
                                LaunchConfiguration("patrol_stuck_min_progress_m"),
                                value_type=float,
                            ),
                            "patrol_stuck_window_sec": ParameterValue(
                                LaunchConfiguration("patrol_stuck_window_sec"),
                                value_type=float,
                            ),
                            "patrol_stuck_cmd_required": ParameterValue(
                                LaunchConfiguration("patrol_stuck_cmd_required"),
                                value_type=bool,
                            ),
                            "patrol_stuck_near_goal_tolerance_m": ParameterValue(
                                LaunchConfiguration("patrol_stuck_near_goal_tolerance_m"),
                                value_type=float,
                            ),
                            "patrol_failed_corridor_enabled": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_enabled"),
                                value_type=bool,
                            ),
                            "patrol_failed_corridor_radius_m": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_radius_m"),
                                value_type=float,
                            ),
                            "patrol_failed_corridor_behind_m": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_behind_m"),
                                value_type=float,
                            ),
                            "patrol_failed_corridor_ahead_m": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_ahead_m"),
                                value_type=float,
                            ),
                            "patrol_failed_corridor_ttl_sec": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_ttl_sec"),
                                value_type=float,
                            ),
                            "patrol_failed_corridor_max": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_max"),
                                value_type=int,
                            ),
                            "patrol_failed_corridor_start_grace_m": ParameterValue(
                                LaunchConfiguration("patrol_failed_corridor_start_grace_m"),
                                value_type=float,
                            ),
                            "patrol_approach_offsets_enabled": ParameterValue(
                                LaunchConfiguration("patrol_approach_offsets_enabled"),
                                value_type=bool,
                            ),
                            "patrol_approach_offset_m": ParameterValue(
                                LaunchConfiguration("patrol_approach_offset_m"),
                                value_type=float,
                            ),
                            "patrol_approach_yaw_variants_enabled": ParameterValue(
                                LaunchConfiguration("patrol_approach_yaw_variants_enabled"),
                                value_type=bool,
                            ),
                            "patrol_escape_enabled": ParameterValue(
                                LaunchConfiguration("patrol_escape_enabled"),
                                value_type=bool,
                            ),
                            "patrol_escape_cmd_topic": LaunchConfiguration(
                                "patrol_escape_cmd_topic"
                            ),
                            "cmd_vel_topic": LaunchConfiguration("patrol_escape_cmd_topic"),
                            "patrol_escape_min_move_m": ParameterValue(
                                LaunchConfiguration("patrol_escape_min_move_m"),
                                value_type=float,
                            ),
                            "patrol_escape_max_attempts": ParameterValue(
                                LaunchConfiguration("patrol_escape_max_attempts"),
                                value_type=int,
                            ),
                            "patrol_escape_sequence_attempts": ParameterValue(
                                LaunchConfiguration("patrol_escape_sequence_attempts"),
                                value_type=int,
                            ),
                            "patrol_escape_backward_speed": ParameterValue(
                                LaunchConfiguration("patrol_escape_backward_speed"),
                                value_type=float,
                            ),
                            "patrol_escape_forward_speed": ParameterValue(
                                LaunchConfiguration("patrol_escape_forward_speed"),
                                value_type=float,
                            ),
                            "patrol_escape_turn_speed": ParameterValue(
                                LaunchConfiguration("patrol_escape_turn_speed"),
                                value_type=float,
                            ),
                            "patrol_escape_backward_duration_sec": ParameterValue(
                                LaunchConfiguration("patrol_escape_backward_duration_sec"),
                                value_type=float,
                            ),
                            "patrol_escape_rotate_duration_sec": ParameterValue(
                                LaunchConfiguration("patrol_escape_rotate_duration_sec"),
                                value_type=float,
                            ),
                            "patrol_escape_arc_duration_sec": ParameterValue(
                                LaunchConfiguration("patrol_escape_arc_duration_sec"),
                                value_type=float,
                            ),
                            "patrol_no_motion_after_goal_sec": ParameterValue(
                                LaunchConfiguration("patrol_no_motion_after_goal_sec"),
                                value_type=float,
                            ),
                            "patrol_no_motion_max_count": ParameterValue(
                                LaunchConfiguration("patrol_no_motion_max_count"),
                                value_type=int,
                            ),
                            "coverage_planner_enabled": ParameterValue(
                                LaunchConfiguration("coverage_planner_enabled"),
                                value_type=bool,
                            ),
                            "coverage_sample_spacing_m": ParameterValue(
                                LaunchConfiguration("coverage_sample_spacing_m"),
                                value_type=float,
                            ),
                            "coverage_max_waypoints": ParameterValue(
                                LaunchConfiguration("coverage_max_waypoints"),
                                value_type=int,
                            ),
                            "waypoint_min_obstacle_distance_m": ParameterValue(
                                LaunchConfiguration("waypoint_min_obstacle_distance_m"),
                                value_type=float,
                            ),
                        }
                    ],
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
                "waypoint_timeout_sec",
                default_value="6.0",
                description="Cancel a waypoint goal after this many seconds without progress. 0 disables timeout.",
            ),
            DeclareLaunchArgument(
                "waypoint_timeout_progress_m",
                default_value="0.08",
                description="Goal-distance decrease that resets the waypoint no-progress timeout.",
            ),
            DeclareLaunchArgument(
                "patrol_stuck_enabled",
                default_value="true",
                description="Enable patrol odom-window stuck detection while Nav2 is navigating.",
            ),
            DeclareLaunchArgument(
                "patrol_stuck_min_progress_m",
                default_value="0.08",
                description="Minimum odom translation required during the stuck detection window.",
            ),
            DeclareLaunchArgument(
                "patrol_stuck_window_sec",
                default_value="4.0",
                description="Seconds of odom history used to detect no-progress stuck states.",
            ),
            DeclareLaunchArgument(
                "patrol_stuck_cmd_required",
                default_value="true",
                description="Require cmd_vel activity before odom no-progress is treated as stuck.",
            ),
            DeclareLaunchArgument(
                "patrol_stuck_near_goal_tolerance_m",
                default_value="0.35",
                description="Do not trigger stuck recovery this close to the selected approach goal.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_enabled",
                default_value="true",
                description="Reject candidate routes that pass through recent failed path corridors.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_radius_m",
                default_value="0.60",
                description="Radius around each failed corridor polyline point used for route rejection.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_behind_m",
                default_value="0.80",
                description="Failed path distance behind the stuck pose to record.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_ahead_m",
                default_value="2.00",
                description="Failed path distance ahead of the stuck pose to record.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_ttl_sec",
                default_value="240.0",
                description="Seconds before a failed corridor expires. 0 keeps corridors forever.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_max",
                default_value="12",
                description="Maximum failed corridors to remember.",
            ),
            DeclareLaunchArgument(
                "patrol_failed_corridor_start_grace_m",
                default_value="0.25",
                description="Initial path distance allowed while leaving a corridor that contains the current pose.",
            ),
            DeclareLaunchArgument(
                "patrol_approach_offsets_enabled",
                default_value="true",
                description="Generate left/right/front/back approach goals around each patrol point.",
            ),
            DeclareLaunchArgument(
                "patrol_approach_offset_m",
                default_value="0.60",
                description="Offset distance for alternate approach goals.",
            ),
            DeclareLaunchArgument(
                "patrol_approach_yaw_variants_enabled",
                default_value="true",
                description="Try alternate yaw values for each approach goal.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_enabled",
                default_value="true",
                description="Publish direct cmd_vel recovery motions after canceling stuck Nav2 goals.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_cmd_topic",
                default_value="/cmd_vel",
                description="cmd_vel topic used by the patrol escape controller.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_min_move_m",
                default_value="0.10",
                description="Minimum odom translation required for escape success.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_max_attempts",
                default_value="2",
                description="Maximum escape events before skipping the waypoint.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_sequence_attempts",
                default_value="1",
                description="Full escape motion sequence repeats per escape event.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_backward_speed",
                default_value="0.12",
                description="Backward linear speed for direct escape.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_forward_speed",
                default_value="0.08",
                description="Forward linear speed for arc escape.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_turn_speed",
                default_value="0.50",
                description="Angular speed for rotate escape.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_backward_duration_sec",
                default_value="1.0",
                description="Backward escape command duration.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_rotate_duration_sec",
                default_value="0.7",
                description="In-place rotate escape command duration.",
            ),
            DeclareLaunchArgument(
                "patrol_escape_arc_duration_sec",
                default_value="1.0",
                description="Forward arc escape command duration.",
            ),
            DeclareLaunchArgument(
                "patrol_no_motion_after_goal_sec",
                default_value="3.0",
                description="Seconds after goal acceptance before no odom movement is reported.",
            ),
            DeclareLaunchArgument(
                "patrol_no_motion_max_count",
                default_value="2",
                description="Consecutive accepted-but-no-motion count before the waypoint is skipped.",
            ),
            DeclareLaunchArgument(
                "coverage_planner_enabled",
                default_value="true",
                description="Generate patrol waypoints from the static map.",
            ),
            DeclareLaunchArgument(
                "coverage_sample_spacing_m",
                default_value="1.10",
                description="Grid spacing for map-based patrol waypoint sampling.",
            ),
            DeclareLaunchArgument(
                "coverage_max_waypoints",
                default_value="40",
                description="Maximum generated patrol waypoints.",
            ),
            DeclareLaunchArgument(
                "waypoint_min_obstacle_distance_m",
                default_value="0.35",
                description="Minimum map clearance for generated patrol waypoints.",
            ),
            DeclareLaunchArgument(
                "start_coverage_monitor",
                default_value="true",
                description="Start /map + /scan coverage monitor.",
            ),
            DeclareLaunchArgument(
                "coverage_required_percent",
                default_value="95.0",
                description="Coverage percentage at which /navigation/coverage_done becomes true.",
            ),
            DeclareLaunchArgument(
                "coverage_map_topic",
                default_value="/map",
                description="OccupancyGrid topic used by the coverage monitor.",
            ),
            DeclareLaunchArgument(
                "coverage_scan_topic",
                default_value="/scan",
                description="LaserScan topic used by the coverage monitor.",
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
