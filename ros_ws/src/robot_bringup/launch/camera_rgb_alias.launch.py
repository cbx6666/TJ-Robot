# pyright: reportMissingImports=false
"""Gazebo 一体 depth 相机发布 /camera/image_raw；为兼容 YOLO 配置转发到 /camera/rgb/*。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="topic_tools",
                executable="relay",
                name="relay_camera_image_to_rgb",
                parameters=[{"use_sim_time": use_sim}],
                arguments=[
                    "/camera/image_raw",
                    "/camera/rgb/image_raw",
                ],
                output="screen",
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="relay_camera_info_to_rgb",
                parameters=[{"use_sim_time": use_sim}],
                arguments=[
                    "/camera/camera_info",
                    "/camera/rgb/camera_info",
                ],
                output="screen",
            ),
        ]
    )
