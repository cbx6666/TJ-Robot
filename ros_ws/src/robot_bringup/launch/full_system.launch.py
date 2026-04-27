# pyright: reportMissingImports=false
"""Full system composition after simulation/base is available."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def _include(share: str, launch_file: str, use_sim_time):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", launch_file)),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_share = get_package_share_directory("robot_bringup")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            _include(bringup_share, "perception.launch.py", use_sim_time),
            _include(bringup_share, "task_pipeline.launch.py", use_sim_time),
        ]
    )
