# pyright: reportMissingImports=false
"""Application entry: simulation layer.

For the complete Gazebo/TurtleBot3 stack use `scripts/run_simulation.sh`.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [LogInfo(msg="Use scripts/run_simulation.sh to start Gazebo/TurtleBot3.")]
    )
