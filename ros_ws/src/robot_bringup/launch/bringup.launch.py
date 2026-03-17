# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="robot_bringup skeleton is ready. Replace this with the real system launch."),
    ])

