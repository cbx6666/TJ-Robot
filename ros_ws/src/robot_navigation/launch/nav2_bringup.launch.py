# pyright: reportMissingImports=false
"""Explicit Nav2 bringup for the indoor patrol stack.

启动顺序必须保守：
1. map_server 先进入 active，确保 /map 已经发布；
2. AMCL 再 configure/active，避免 configure 阶段等不到地图；
3. planner/controller 最后启动，避免 global costmap 在 map->odom 还不存在时卡住。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _nav2_node(package: str, executable: str, name: str, params_file, common_params):
    # Remap /tf to relative names so future namespaces keep Nav2's expected behavior.
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[params_file, common_params],
        remappings=tf_remaps,
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    autostart_param = ParameterValue(autostart, value_type=bool)
    common_params = {"use_sim_time": use_sim_time_param}
    lifecycle_common_params = {
        "use_sim_time": use_sim_time_param,
        "autostart": autostart_param,
    }

    navigation_nodes = [
        _nav2_node(
            "nav2_controller",
            "controller_server",
            "controller_server",
            params_file,
            common_params,
        ),
        _nav2_node(
            "nav2_planner",
            "planner_server",
            "planner_server",
            params_file,
            common_params,
        ),
        _nav2_node(
            "nav2_behaviors",
            "behavior_server",
            "behavior_server",
            params_file,
            common_params,
        ),
        _nav2_node(
            "nav2_bt_navigator",
            "bt_navigator",
            "bt_navigator",
            params_file,
            common_params,
        ),
        _nav2_node(
            "nav2_waypoint_follower",
            "waypoint_follower",
            "waypoint_follower",
            params_file,
            common_params,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                params_file,
                {
                    **lifecycle_common_params,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                    ],
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time_param}],
            condition=IfCondition(use_rviz),
        ),
    ]

    return LaunchDescription(
        [
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
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_navigation"), "config", "nav2_params_astar.yaml"]
                ),
                description="Full Nav2 parameter file.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 lifecycle nodes to active.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the Nav2 view.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"]
                ),
                description="RViz config file.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time_param,
                        "yaml_filename": map_file,
                    },
                ],
            ),
            _nav2_node("nav2_amcl", "amcl", "amcl", params_file, common_params),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    params_file,
                    {
                        **lifecycle_common_params,
                        "node_names": ["map_server"],
                    },
                ],
            ),
            # map_server active 后 AMCL 才能稳定拿到 /map；否则 AMCL configure 偶发卡住，
            # 后续 planner 会一直报 Invalid frame ID "map"。
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_amcl",
                        output="screen",
                        parameters=[
                            params_file,
                            {
                                **lifecycle_common_params,
                                "node_names": ["amcl"],
                            },
                        ],
                    )
                ],
            ),
            # 等 map_server 和 AMCL 都有时间发布 /map 与 map->odom，再启动 navigation。
            TimerAction(period=12.0, actions=navigation_nodes),
        ]
    )
