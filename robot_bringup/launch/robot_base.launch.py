# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [LogInfo(msg="Robot base runtime is managed by scripts/tb3_stack.sh.")]
    )
