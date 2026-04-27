# pyright: reportMissingImports=false
"""Standard Nav2 bringup wrapper."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    share = get_package_share_directory("robot_bringup")
    default_base = os.path.join(share, "config", "nav2", "base.yaml")
    default_profile = os.path.join(share, "config", "nav2", "profiles", "coverage_patrol.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("nav2_base_params", default_value=default_base),
            DeclareLaunchArgument("nav2_profile_params", default_value=default_profile),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "nav2_coverage_patrol.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "nav2_base_params": LaunchConfiguration("nav2_base_params"),
                    "nav2_profile_params": LaunchConfiguration("nav2_profile_params"),
                }.items(),
            ),
        ]
    )
