# pyright: reportMissingImports=false
"""YOLO 多类物体 Seg + 地图定位（topic 前缀默认 /yolo_objects）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    enable_reg = LaunchConfiguration("enable_depth_register").perform(context).strip().lower()
    use_reg = enable_reg in ("1", "true", "yes", "on")
    if use_reg:
        depth_topic = "/yolo_objects/depth_registered/image_rect"
        depth_ci_topic = LaunchConfiguration("camera_info_topic").perform(context)
        depth_aligned = True
    else:
        depth_topic = LaunchConfiguration("depth_topic").perform(context)
        depth_ci_topic = LaunchConfiguration("depth_camera_info_topic").perform(context)
        depth_aligned = (
            LaunchConfiguration("depth_pixels_aligned").perform(context).strip().lower()
            in ("1", "true", "yes", "on")
        )

    node = Node(
        package="human_yolo_seg",
        executable="yolo_detector_node",
        name="yolo_object_seg",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context).lower()
                in ("1", "true", "yes", "on"),
                "image_topic": LaunchConfiguration("image_topic").perform(context),
                "output_topic": LaunchConfiguration("output_topic").perform(context),
                "camera_info_topic": LaunchConfiguration("camera_info_topic").perform(context),
                "depth_topic": depth_topic,
                "depth_camera_info_topic": depth_ci_topic,
                "model_path": LaunchConfiguration("model_path").perform(context),
                "conf_threshold": float(
                    LaunchConfiguration("conf_threshold").perform(context)
                ),
                "imgsz": int(LaunchConfiguration("imgsz").perform(context)),
                "device": LaunchConfiguration("device").perform(context),
                "annotated_overlay_mode": LaunchConfiguration("annotated_overlay_mode").perform(
                    context
                ),
                "target_class_ids_csv": LaunchConfiguration("target_class_ids_csv").perform(
                    context
                ),
                "publish_target_point_3d": LaunchConfiguration("publish_target_point_3d")
                .perform(context)
                .lower()
                in ("1", "true", "yes", "on"),
                "target_frame_id": LaunchConfiguration("target_frame_id").perform(context),
                "depth_unit_divisor": float(
                    LaunchConfiguration("depth_unit_divisor").perform(context)
                ),
                "depth_max_m": float(LaunchConfiguration("depth_max_m").perform(context)),
                "depth_sensor_far_m": float(
                    LaunchConfiguration("depth_sensor_far_m").perform(context)
                ),
                "depth_reject_near_clip_ratio": float(
                    LaunchConfiguration("depth_reject_near_clip_ratio").perform(context)
                ),
                "depth_range_to_optical_z": LaunchConfiguration("depth_range_to_optical_z")
                .perform(context)
                .lower()
                in ("1", "true", "yes", "on"),
                "use_depth_sync": LaunchConfiguration("use_depth_sync")
                .perform(context)
                .lower()
                in ("1", "true", "yes", "on"),
                "depth_sync_slop_sec": float(
                    LaunchConfiguration("depth_sync_slop_sec").perform(context)
                ),
                "depth_sample_stat": LaunchConfiguration("depth_sample_stat").perform(context),
                "depth_pixels_aligned": depth_aligned,
                "max_depth_age_sec": float(
                    LaunchConfiguration("max_depth_age_sec").perform(context)
                ),
                "target_point_use_seg_mask_depth": True,
                "enable_target_tracking": True,
                "max_targets_3d_per_frame": 5,
                "min_track_hits_for_publish": 3,
                "marker_publish_only_eligible": True,
                "clear_target_point_when_lost": True,
                "publish_target_markers": True,
                "publish_target_azimuths": False,
            }
        ],
    )
    return [node]


def generate_launch_description():
    enable_reg = LaunchConfiguration("enable_depth_register")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "output_topic", default_value="/yolo_objects/annotated_image"
            ),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument(
                "enable_depth_register",
                default_value="false",
                description="true: 启动 depth_image_proc/register 并自动用注册深度+RGB 内参",
            ),
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
            DeclareLaunchArgument("annotated_overlay_mode", default_value="boxes"),
            DeclareLaunchArgument("publish_target_point_3d", default_value="true"),
            DeclareLaunchArgument("target_frame_id", default_value="map"),
            DeclareLaunchArgument("depth_unit_divisor", default_value="1000.0"),
            DeclareLaunchArgument("depth_max_m", default_value="12.0"),
            DeclareLaunchArgument("depth_sensor_far_m", default_value="20.0"),
            DeclareLaunchArgument("depth_reject_near_clip_ratio", default_value="0.98"),
            DeclareLaunchArgument("depth_range_to_optical_z", default_value="false"),
            DeclareLaunchArgument("use_depth_sync", default_value="true"),
            DeclareLaunchArgument("depth_sync_slop_sec", default_value="0.12"),
            DeclareLaunchArgument("depth_sample_stat", default_value="median"),
            DeclareLaunchArgument("depth_pixels_aligned", default_value="false"),
            DeclareLaunchArgument("max_depth_age_sec", default_value="1.0"),
            DeclareLaunchArgument("target_class_ids_csv", default_value="39,41,75"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("human_yolo_seg"),
                            "launch",
                            "yolo_depth_register.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(enable_reg),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rgb_camera_info_topic": LaunchConfiguration("camera_info_topic"),
                }.items(),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
