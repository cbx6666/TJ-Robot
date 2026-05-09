# pyright: reportMissingImports=false
"""静态地图 + Nav2 的统一 ROS 入口（对齐 nav2_bringup：用 IncludeLaunchDescription 包一层 navigation.launch.py）。

地图、params 路径仍由外层脚本（tj_sim_nav2_stack.sh）准备好（含 /tmp ASCII 拷贝）后通过 launch 参数传入，
避免在多个 bash 里重复写 ros2 launch … navigation.launch.py。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    nav_launch = PathJoinSubstitution(
        [FindPackageShare("robot_navigation"), "launch", "navigation.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "map",
                description="map yaml 绝对路径（建议为 /tmp 下 ASCII 拷贝）",
            ),
            DeclareLaunchArgument(
                "params_file",
                description="nav2 params yaml 绝对路径（建议为 /tmp 下 ASCII 拷贝）",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="与 navigation.launch 一致；False 降低 map_server 生命周期竞态",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("params_file"),
                    "use_composition": LaunchConfiguration("use_composition"),
                }.items(),
            ),
        ]
    )
