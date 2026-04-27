# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [LogInfo(msg="Gazebo runtime is managed by scripts/run_simulation.sh.")]
    )
