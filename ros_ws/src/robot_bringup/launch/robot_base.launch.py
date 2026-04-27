# pyright: reportMissingImports=false
"""Robot base bringup placeholder.

`tb3_stack.sh` currently prepares the TurtleBot3 model and starts
robot_state_publisher. Keep base bringup as a named layer for future migration.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [LogInfo(msg="Robot base is currently started by scripts/tb3_stack.sh.")]
    )
