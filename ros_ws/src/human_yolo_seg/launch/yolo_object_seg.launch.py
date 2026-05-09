# pyright: reportMissingImports=false
"""YOLO 多类物体 Seg + 地图定位（topic 前缀默认 /yolo_objects）。"""

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
                "output_topic", default_value="/yolo_objects/annotated_image"
            ),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            # 与 robot_bringup rgbd_to_scan、Gazebo 插件 camera_name=tb3_depth_only 对齐；勿用不存在的 /camera/depth
            DeclareLaunchArgument(
                "depth_topic", default_value="/tb3_depth_only/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_camera_info_topic",
                default_value="/tb3_depth_only/depth/camera_info",
            ),
            DeclareLaunchArgument("model_path", default_value="yolo26n-seg.pt"),
            DeclareLaunchArgument("conf_threshold", default_value="0.25"),
            DeclareLaunchArgument("imgsz", default_value="480"),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "annotated_overlay_mode",
                default_value="boxes",
                description="annotated_image 叠图：boxes(快)/plot_no_masks/full(含分割蒙层最慢)",
            ),
            DeclareLaunchArgument("publish_target_point_3d", default_value="true"),
            DeclareLaunchArgument("target_frame_id", default_value="map"),
            DeclareLaunchArgument("depth_unit_divisor", default_value="1000.0"),
            DeclareLaunchArgument(
                "target_class_ids_csv",
                default_value="0,56",
                description="COCO 类别逗号分隔，如 56 仅椅子、0,56、all 全类",
            ),
            Node(
                package="human_yolo_seg",
                # 使用 yolo_detector_node：与 yolo_object_seg_node 同源，且在旧 workspace 重建前一般已存在于 install，
                # 避免因缺少 yolo_object_seg_node 脚本导致静默起不来（RViz Image 订阅不到 /yolo_objects/annotated_image）。
                executable="yolo_detector_node",
                name="yolo_object_seg",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "depth_camera_info_topic": LaunchConfiguration(
                            "depth_camera_info_topic"
                        ),
                        "model_path": LaunchConfiguration("model_path"),
                        "conf_threshold": LaunchConfiguration("conf_threshold"),
                        "imgsz": LaunchConfiguration("imgsz"),
                        "device": LaunchConfiguration("device"),
                        "annotated_overlay_mode": LaunchConfiguration("annotated_overlay_mode"),
                        "target_class_ids_csv": LaunchConfiguration("target_class_ids_csv"),
                        "publish_target_point_3d": LaunchConfiguration("publish_target_point_3d"),
                        "target_frame_id": LaunchConfiguration("target_frame_id"),
                        "depth_unit_divisor": LaunchConfiguration("depth_unit_divisor"),
                        "enable_target_tracking": True,
                        "max_targets_3d_per_frame": 5,
                        "min_track_hits_for_publish": 2,
                        "publish_target_markers": True,
                        "publish_target_azimuths": False,
                    }
                ],
            ),
        ]
    )
