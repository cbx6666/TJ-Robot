import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def _default_nav2_params_file() -> str:
    """与工作区源码一致：若设置了 ROS_WS 且 src 下存在 yaml，则用 src（避免只改源码未 rebuild 却仍读 install 副本）。"""
    ws = os.environ.get("ROS_WS", "").strip()
    if ws:
        src_yaml = Path(ws) / "src" / "robot_navigation" / "config" / "nav2_params.yaml"
        if src_yaml.is_file():
            return str(src_yaml.resolve())
    return str(Path(__file__).resolve().parent.parent / "config" / "nav2_params.yaml")


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    defer_navigation_autostart = LaunchConfiguration("defer_navigation_autostart")

    nav2_bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "maps", "map.yaml"]
                ),
                description="Static map yaml file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=_default_nav2_params_file(),
                description="Nav2 parameter file（默认：若 export ROS_WS=...则用该工作区源码中的 yaml）",
            ),
            # bringup_launch 默认 use_composition=True，在同一 component_container_isolated 里
            # map_server 尚未就绪时 localization 的 lifecycle_manager 会先 configure，
            # 易出现 “Failed to change state for node: map_server”，/map 无发布者、map 坐标系缺失。
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="必须为 Python 识别的 True/False。False=各节点独立进程，避免 map_server 生命周期竞态",
            ),
            DeclareLaunchArgument(
                "defer_navigation_autostart",
                default_value="false",
                description=(
                    "true：拆成 localization + navigation 两次 include，且 navigation 的 "
                    "lifecycle_manager 不 autostart，须由脚本在 map_server active 后调用 "
                    "manage_nodes STARTUP，避免与 navigation 并行配置时 map_server 卡死、无 map TF"
                ),
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [nav2_bringup_launch_dir, "bringup_launch.py"]
                            )
                        ),
                        launch_arguments={
                            "slam": "False",
                            "use_sim_time": use_sim_time,
                            "map": map_file,
                            "params_file": params_file,
                            "autostart": "True",
                            "use_composition": use_composition,
                        }.items(),
                    ),
                ],
                condition=IfCondition(
                    EqualsSubstitution(defer_navigation_autostart, "false")
                ),
            ),
            GroupAction(
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [nav2_bringup_launch_dir, "localization_launch.py"]
                            )
                        ),
                        launch_arguments={
                            "namespace": "",
                            "map": map_file,
                            "use_sim_time": use_sim_time,
                            "autostart": "True",
                            "params_file": params_file,
                            "use_composition": use_composition,
                            "use_respawn": "False",
                            "container_name": "nav2_container",
                            "log_level": "info",
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [nav2_bringup_launch_dir, "navigation_launch.py"]
                            )
                        ),
                        launch_arguments={
                            "namespace": "",
                            "use_sim_time": use_sim_time,
                            "autostart": "False",
                            "params_file": params_file,
                            "use_composition": use_composition,
                            "use_respawn": "False",
                            "container_name": "nav2_container",
                            "log_level": "info",
                        }.items(),
                    ),
                ],
                condition=IfCondition(
                    EqualsSubstitution(defer_navigation_autostart, "true")
                ),
            ),
        ]
    )
