# pyright: reportMissingImports=false
"""将深度图注册到 RGB 相机坐标系（depth_image_proc::RegisterNode 组件）。

Humble 的 depth_image_proc 通常无独立 register 可执行文件，须加载到 component_container。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  use_sim = LaunchConfiguration("use_sim_time")
  return LaunchDescription(
    [
      DeclareLaunchArgument("use_sim_time", default_value="true"),
      DeclareLaunchArgument(
        "rgb_camera_info_topic", default_value="/camera/camera_info"
      ),
      DeclareLaunchArgument(
        "depth_image_topic", default_value="/tb3_depth_only/depth/image_raw"
      ),
      DeclareLaunchArgument(
        "depth_camera_info_topic",
        default_value="/tb3_depth_only/depth/camera_info",
      ),
      DeclareLaunchArgument(
        "registered_depth_topic",
        default_value="/yolo_objects/depth_registered/image_rect",
      ),
      DeclareLaunchArgument(
        "registered_depth_info_topic",
        default_value="/yolo_objects/depth_registered/camera_info",
      ),
      ComposableNodeContainer(
        name="yolo_depth_register_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[{"use_sim_time": use_sim}],
        composable_node_descriptions=[
          ComposableNode(
            package="depth_image_proc",
            plugin="depth_image_proc::RegisterNode",
            name="register_node",
            parameters=[{"use_sim_time": use_sim}],
            remappings=[
              ("rgb/camera_info", LaunchConfiguration("rgb_camera_info_topic")),
              ("depth/image_rect", LaunchConfiguration("depth_image_topic")),
              ("depth/camera_info", LaunchConfiguration("depth_camera_info_topic")),
              (
                "depth_registered/image_rect",
                LaunchConfiguration("registered_depth_topic"),
              ),
              (
                "depth_registered/camera_info",
                LaunchConfiguration("registered_depth_info_topic"),
              ),
            ],
          )
        ],
      ),
    ]
  )
