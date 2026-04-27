# pyright: reportMissingImports=false
"""Gazebo bringup entrypoint.

The existing project starts Gazebo, robot spawning, obstacle spawning, SLAM
smoke checks, and RViz through `scripts/tb3_stack.sh`. This launch file exists
as the ROS-layer placeholder so higher-level apps have a stable name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="small_house.world"),
            LogInfo(
                msg=[
                    "Gazebo is managed by scripts/tb3_stack.sh today; requested world=",
                    world_file,
                ]
            ),
        ]
    )
