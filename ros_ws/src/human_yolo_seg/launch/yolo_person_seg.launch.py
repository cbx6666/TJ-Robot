# pyright: reportMissingImports=false
"""兼容旧文件名：等价于 launch yolo_object_seg.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("human_yolo_seg")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "yolo_object_seg.launch.py")),
            ),
        ]
    )
