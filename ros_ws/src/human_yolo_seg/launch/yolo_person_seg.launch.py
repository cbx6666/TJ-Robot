# pyright: reportMissingImports=false
"""Minimal YOLO recognition launch (keep detection only)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "output_topic", default_value="/human_yolo/annotated_image"
            ),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("model_path", default_value="yolo26n-seg.pt"),
            DeclareLaunchArgument("conf_threshold", default_value="0.25"),
            DeclareLaunchArgument("imgsz", default_value="640"),
            DeclareLaunchArgument("device", default_value="auto"),
            Node(
                package="human_yolo_seg",
                executable="yolo_person_seg_node",
                name="yolo_person_seg",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "model_path": LaunchConfiguration("model_path"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "imgsz": LaunchConfiguration("imgsz"),
                        "device": LaunchConfiguration("device"),
                        "publish_person_azimuths": False,
                    }
                ],
            ),
        ]
    )
