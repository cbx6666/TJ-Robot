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

    map_pgm_arg = DeclareLaunchArgument(
        "map_pgm",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_bringup"), "maps", "map.pgm"]
        ),
        description="Map image file for nav stack",
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("robot_bringup"), "maps", "map.yaml"]
        ),
        description="Map yaml file for nav stack",
    )

    world_file = LaunchConfiguration("world_file")
    map_pgm = LaunchConfiguration("map_pgm")
    map_yaml = LaunchConfiguration("map_yaml")

    return LaunchDescription(
        [
            world_file_arg,
            map_pgm_arg,
            map_yaml_arg,
            LogInfo(msg=["Starting robot_bringup with world=", world_file]),
            LogInfo(msg=["map_pgm=", map_pgm, ", map_yaml=", map_yaml]),
        ]
    )
