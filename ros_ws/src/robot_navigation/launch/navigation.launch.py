import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
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
        ]
    )
