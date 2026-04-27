# pyright: reportMissingImports=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_bringup"), "world", "small_house.world"]
        ),
        description="Gazebo world file to load",
    )

    world_file = LaunchConfiguration("world_file")

    return LaunchDescription(
        [
            world_file_arg,
            LogInfo(msg=["Starting robot_bringup with world=", world_file]),
        ]
    )
