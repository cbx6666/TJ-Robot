# pyright: reportMissingImports=false
"""Indoor search task composition placeholder."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(share: str, name: str, use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    share = get_package_share_directory("robot_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            _include(share, "nav2.launch.py", use_sim_time),
            _include(share, "perception.launch.py", use_sim_time),
            _include(share, "task_pipeline.launch.py", use_sim_time),
        ]
    )
