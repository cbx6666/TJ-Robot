# pyright: reportMissingImports=false
"""Perception bringup wrapper for YOLO/person pipeline."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    share = get_package_share_directory("human_yolo_seg")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_yolo", default_value="true"),
            DeclareLaunchArgument("person_slam_mode", default_value="mark_then_strip"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "yolo_person_seg.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_yolo": LaunchConfiguration("enable_yolo"),
                    "person_slam_mode": LaunchConfiguration("person_slam_mode"),
                }.items(),
            ),
        ]
    )
