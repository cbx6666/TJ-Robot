# pyright: reportMissingImports=false
"""订阅相机 RGB，Ultralytics YOLO（Seg 推荐）多类物体定位（默认 COCO：瓶/杯/花瓶）。

发布：标注图像、地图 MarkerArray（多目标）、主目标 PointStamped、检测统计话题（默认前缀 /yolo_objects）。

3D 深度：RGB/Depth 近似时间同步；在检测框映射的深度 ROI 内做稳健统计（默认 median，对齐 yolo_ros）。
可选 depth_image_proc 注册深度（与 RGB 同像素网格）。

默认权重 yolo26n-seg.pt：放包内 models/ 后 colcon build，或 model_path 传绝对路径。
"""

from __future__ import annotations

import math
import os
import sys
import time
from contextlib import nullcontext
from typing import Any

import numpy as np

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from visualization_msgs.msg import Marker, MarkerArray

import message_filters
import tf2_ros

from human_yolo_seg.utils.depth_sample import (
    RgbDepthProjector,
    sample_depth_roi,
    sample_depth_with_optical_z,
)
from human_yolo_seg.utils.object_semantics import resolve_desired_coco_class_ids
from human_yolo_seg.utils.person_scan_sync_utils import norm_angle

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs import do_transform_point  # type: ignore


def _check_numpy_compatible_with_cv_bridge() -> None:
    """Humble 的 cv_bridge 针对 NumPy 1.x 编译，NumPy 2.x 会在 imgmsg_to_cv2 等处崩溃。"""
    import numpy as np

    major = int(np.__version__.split(".", 1)[0])
    if major >= 2:
        print(
            f"ERROR: 当前 NumPy {np.__version__} 与 ROS 2 Humble 的 cv_bridge 不兼容（常见报错: _ARRAY_API）。\n"
            "  请使用与 ros2 相同的 python3 执行: pip install 'numpy>=1.23,<2'\n"
            "  然后重启 yolo 节点。requirements.txt 已约束 numpy<2。",
            file=sys.stderr,
        )
        raise RuntimeError("NumPy>=2 is incompatible with cv_bridge on ROS 2 Humble")


def _try_import_ultralytics():
    try:
        from ultralytics import YOLO  # type: ignore

        return YOLO
    except ImportError as e:
        print(
            "ERROR: 未安装 ultralytics/torch。请使用与 ros2 相同的 python3：\n"
            "  pip install ultralytics torch torchvision opencv-python-headless\n"
            "或: pip install -r <工作空间>/src/human_yolo_seg/requirements.txt",
            file=sys.stderr,
        )
        raise e


def resolve_model_path(logger, model_path: str) -> str:
    """优先：存在的绝对/相对路径；否则 share/human_yolo_seg/models/<文件名>；否则原样交给 Ultralytics。"""
    expanded = os.path.expanduser(model_path.strip())
    if os.path.isfile(expanded):
        resolved = os.path.abspath(expanded)
        logger.info(f"模型路径（用户指定文件）: {resolved}")
        return resolved
    basename = os.path.basename(expanded)
    try:
        share = get_package_share_directory("human_yolo_seg")
        cand = os.path.join(share, "models", basename)
        if os.path.isfile(cand):
            logger.info(f"模型路径（包内 models/）: {cand}")
            return cand
    except PackageNotFoundError:
        logger.warning("未找到已安装的 human_yolo_seg share（请先 colcon build）")
    logger.info(f"按 Ultralytics 解析模型: {expanded}")
    return expanded


class YoloObjectSegNode(Node):
    def __init__(self) -> None:
        # 与 launch 中 name='yolo_object_seg' 一致，便于 ros2 param get /yolo_object_seg …
        super().__init__("yolo_object_seg")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/yolo_objects/annotated_image")
        self.declare_parameter("model_path", "yolo26n-seg.pt")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        # COCO：39 bottle, 41 cup, 75 vase（可乐罐无专用类，多落 bottle/cup）
        self.declare_parameter("target_class_ids", [39, 41, 75])
        self.declare_parameter(
            "target_class_ids_csv",
            "",
        )
        # auto：有 NVIDIA+CUDA 版 PyTorch 时用 cuda:0，否则 cpu（无需手改代码）
        self.declare_parameter("device", "auto")
        # CUDA 下 predict(half=True) 可明显加速；CPU 会被自动忽略。
        self.declare_parameter("use_fp16", True)
        # 叠图方式：boxes=仅画框（最快）；plot_no_masks=Ultralytics plot 关 mask；full=含分割蒙层（最慢）。
        self.declare_parameter("annotated_overlay_mode", "boxes")
        self.declare_parameter("max_inferences_per_sec", 0.0)
        self.declare_parameter("publish_detection_stats", True)
        self.declare_parameter("stats_topic_prefix", "/yolo_objects")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("publish_target_azimuths", False)
        self.declare_parameter("target_azimuth_topic", "/yolo_objects/target_azimuth_ranges")
        self.declare_parameter("laser_frame_id", "base_scan")
        self.declare_parameter("ground_frame_id", "base_footprint")
        self.declare_parameter("publish_azimuth_debug", True)
        self.declare_parameter("azimuth_debug_topic", "/yolo_objects/target_azimuth_debug")
        self.declare_parameter("log_target_azimuth_to_console", True)
        self.declare_parameter("log_target_azimuth_interval_sec", 1.0)
        # 目标方位角：tf_geometry=内参+TF+地面求交；linear_fov=框横坐标占图像宽×水平视场（粗略、无 TF）
        self.declare_parameter("target_azimuth_mode", "linear_fov")
        self.declare_parameter("target_azimuth_linear_hfov_deg", 0.0)
        self.declare_parameter("target_azimuth_linear_camera_yaw_deg", 0.0)
        self.declare_parameter("target_azimuth_linear_use_mask", False)
        self.declare_parameter("depth_topic", "/tb3_depth_only/depth/image_raw")
        self.declare_parameter(
            "depth_camera_info_topic",
            "/tb3_depth_only/depth/camera_info",
        )
        self.declare_parameter("publish_target_point_3d", True)
        self.declare_parameter("target_point_camera_topic", "/yolo_objects/target_point_camera")
        self.declare_parameter("target_point_map_topic", "/yolo_objects/target_point_map")
        self.declare_parameter("target_label_topic", "/yolo_objects/target_label")
        self.declare_parameter("desired_object_topic", "/yolo_objects/desired_object_label")
        self.declare_parameter("desired_object_ttl_sec", 120.0)
        self.declare_parameter("prefer_nearest_when_filtered", True)
        self.declare_parameter("target_frame_id", "map")
        self.declare_parameter("depth_unit_divisor", 1000.0)
        # 仅当深度值为「沿像素射线距离」时开启；Gazebo 仿真深度多为光学 Z（默认 false）。
        self.declare_parameter("depth_range_to_optical_z", False)
        self.declare_parameter("depth_min_m", 0.15)
        # 室内 3D 距离上限；须覆盖房间对角线，过小会导致 3D 点全部被滤掉。
        self.declare_parameter("depth_max_m", 12.0)
        # 与 Gazebo <clip><far> 一致（assist 默认 20，见 TB3_DEPTH_CLIP_FAR_M）；>= far*ratio 视为无效。
        self.declare_parameter("depth_sensor_far_m", 20.0)
        self.declare_parameter("depth_reject_near_clip_ratio", 0.98)
        self.declare_parameter("depth_sample_radius_px", 3)
        # True: message_filters 同步 RGB+Depth；False: 仅用最新深度（旧行为）
        self.declare_parameter("use_depth_sync", True)
        self.declare_parameter("depth_sync_slop_sec", 0.12)
        self.declare_parameter("depth_sync_queue_size", 10)
        # ROI 深度统计: median | trimmed_mean | min
        self.declare_parameter("depth_sample_stat", "median")
        # 深度已与 RGB 配准（aligned_depth / register 输出）时设 true
        self.declare_parameter("depth_pixels_aligned", False)
        self.declare_parameter("depth_roi_grid_n", 7)
        # 有实例分割 mask 时：中心邻域须全在 mask 内才用框心深度；否则在框垂直中线高度取横线，在 mask 上均匀采样深度再取中位数。纯检测模型无 mask 时自动退回框心。
        self.declare_parameter("target_point_use_seg_mask_depth", True)
        self.declare_parameter("target_point_mask_line_samples", 21)
        self.declare_parameter("target_point_mask_line_min_valid", 3)
        # RGB 8Hz / 深度 10Hz + YOLO 推理滞后，|Δt| 常达 0.5–0.7s，过小会整段跳过 3D。
        self.declare_parameter("max_depth_age_sec", 1.0)
        self.declare_parameter("enable_target_tracking", True)
        self.declare_parameter("track_match_max_px", 90.0)
        self.declare_parameter("track_max_age_sec", 2.0)
        self.declare_parameter("track_smoothing_alpha", 0.65)
        self.declare_parameter("min_track_hits_for_publish", 2)
        self.declare_parameter("marker_publish_only_eligible", True)
        self.declare_parameter("clear_target_point_when_lost", True)
        self.declare_parameter("max_targets_3d_per_frame", 3)
        self.declare_parameter("track_debug_topic", "/yolo_objects/target_tracks")
        self.declare_parameter("publish_target_markers", True)
        self.declare_parameter("target_objects_marker_topic", "/yolo_objects/target_objects_marker")
        # 2D 地图上用固定 z：检测框中心的深度点在相机坐标里对应家具“中间高度”，映射到 map 后会显得飘空。
        self.declare_parameter("target_markers_use_flat_map_z", True)
        self.declare_parameter("target_markers_map_z", 0.08)

        _check_numpy_compatible_with_cv_bridge()
        YOLO = _try_import_ultralytics()
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        raw_model = self.get_parameter("model_path").get_parameter_value().string_value
        model_path = resolve_model_path(self.get_logger(), raw_model)
        self._device = self._resolve_yolo_device()
        self.get_logger().info(f"加载 YOLO-Seg: {model_path}（推理设备: {self._device}）")
        self._model: Any = YOLO(model_path)

        self._conf = self.get_parameter("conf_threshold").get_parameter_value().double_value
        self._iou = self.get_parameter("iou_threshold").get_parameter_value().double_value
        self._imgsz = int(self.get_parameter("imgsz").get_parameter_value().integer_value)
        self._use_fp16 = bool(self.get_parameter("use_fp16").get_parameter_value().bool_value)
        _mode_raw = (
            self.get_parameter("annotated_overlay_mode").get_parameter_value().string_value.strip().lower()
        )
        if _mode_raw in ("full", "plot_no_masks", "boxes"):
            self._overlay_mode = _mode_raw
        else:
            self.get_logger().warning(
                f"未知 annotated_overlay_mode={_mode_raw!r}，改用 plot_no_masks（可选: full/plot_no_masks/boxes）"
            )
            self._overlay_mode = "plot_no_masks"

        self._classes = self._resolve_target_classes()

        mps = self.get_parameter("max_inferences_per_sec").get_parameter_value().double_value
        self._min_interval = 1.0 / mps if mps and mps > 0.0 else 0.0
        self._last_t = 0.0

        in_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # 输入相机/深度：与 Gazebo 常见 SensorData（Best Effort）对齐；若用 Reliable 订阅而仿真发 BE → 根本收不到图，
        # 表现为 /yolo_objects/annotated_image 永不出现、tb3_stack warm-up 超时。
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # 输出标注图：仍用 Reliable，便于 RViz2 Image 默认配置订阅。
        _pub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Image, out_topic, _pub_qos)
        self._pub_stats = bool(
            self.get_parameter("publish_detection_stats").get_parameter_value().bool_value
        )
        sp = self.get_parameter("stats_topic_prefix").get_parameter_value().string_value.rstrip("/")
        self._stats_prefix = sp
        if self._pub_stats:
            self._pub_detection_count = self.create_publisher(Int32, f"{sp}/detection_count", 10)
            self._pub_detection_max_conf = self.create_publisher(Float32, f"{sp}/detection_max_conf", 10)
            self._pub_detection_present = self.create_publisher(Bool, f"{sp}/detection_present", 10)
            self.get_logger().info(f"检测统计: {sp}/detection_count、detection_max_conf、detection_present")
        else:
            self._pub_detection_count = None
            self._pub_detection_max_conf = None
            self._pub_detection_present = None

        self._pub_azimuth = None
        self._cam_info: CameraInfo | None = None
        self._tf_buffer: tf2_ros.Buffer | None = None
        self._tf_listener = None
        self._laser_frame = ""
        self._ground_frame = ""
        self._last_cam_warn_mono = 0.0
        self._pub_az_dbg = None
        self._log_az_console = False
        self._log_az_interval = 1.0
        self._last_azimuth_console_mono = 0.0
        self._azimuth_mode = "linear_fov"
        self._linear_hfov_rad: float | None = None
        self._linear_camera_yaw_rad = 0.0
        self._linear_use_mask = False
        self._latest_depth_msg: Image | None = None
        self._synced_depth_msg: Image | None = None
        self._depth_cam_info: CameraInfo | None = None
        self._last_depth_warn_mono = 0.0
        self._depth_diag_logged = False
        self._publish_target_point_3d_enabled = bool(
            self.get_parameter("publish_target_point_3d").get_parameter_value().bool_value
        )
        self._depth_unit_divisor = float(
            self.get_parameter("depth_unit_divisor").get_parameter_value().double_value
        )
        self._depth_range_to_optical_z = bool(
            self.get_parameter("depth_range_to_optical_z").get_parameter_value().bool_value
        )
        self._depth_min_m = float(self.get_parameter("depth_min_m").get_parameter_value().double_value)
        self._depth_max_m = float(self.get_parameter("depth_max_m").get_parameter_value().double_value)
        self._depth_sensor_far_m = float(
            self.get_parameter("depth_sensor_far_m").get_parameter_value().double_value
        )
        self._depth_reject_near_clip_ratio = float(
            self.get_parameter("depth_reject_near_clip_ratio").get_parameter_value().double_value
        )
        self._depth_near_clip_reject_m = max(
            self._depth_min_m,
            self._depth_sensor_far_m * max(0.5, min(1.0, self._depth_reject_near_clip_ratio)),
        )
        self._depth_sample_radius_px = int(
            self.get_parameter("depth_sample_radius_px").get_parameter_value().integer_value
        )
        self._target_point_use_seg_mask_depth = bool(
            self.get_parameter("target_point_use_seg_mask_depth").get_parameter_value().bool_value
        )
        self._target_point_mask_line_samples = max(
            3, int(self.get_parameter("target_point_mask_line_samples").get_parameter_value().integer_value)
        )
        self._target_point_mask_line_min_valid = max(
            1, int(self.get_parameter("target_point_mask_line_min_valid").get_parameter_value().integer_value)
        )
        self._max_depth_age_sec = float(
            self.get_parameter("max_depth_age_sec").get_parameter_value().double_value
        )
        self._use_depth_sync = bool(
            self.get_parameter("use_depth_sync").get_parameter_value().bool_value
        )
        self._depth_sync_slop_sec = float(
            self.get_parameter("depth_sync_slop_sec").get_parameter_value().double_value
        )
        self._depth_sync_queue_size = max(
            2, int(self.get_parameter("depth_sync_queue_size").get_parameter_value().integer_value)
        )
        _stat_raw = (
            self.get_parameter("depth_sample_stat").get_parameter_value().string_value.strip().lower()
        )
        if _stat_raw not in ("median", "trimmed_mean", "min"):
            self.get_logger().warning(f"未知 depth_sample_stat={_stat_raw!r}，改用 median")
            _stat_raw = "median"
        self._depth_sample_stat = _stat_raw
        self._depth_pixels_aligned = bool(
            self.get_parameter("depth_pixels_aligned").get_parameter_value().bool_value
        )
        self._depth_roi_grid_n = max(
            3, int(self.get_parameter("depth_roi_grid_n").get_parameter_value().integer_value)
        )
        self._target_frame = self.get_parameter("target_frame_id").get_parameter_value().string_value
        self._pub_target_point_camera = None
        self._pub_target_point_map = None
        self._pub_target_label = None
        self._tf_buffer_3d = None
        self._tf_listener_3d = None
        self._enable_target_tracking = bool(
            self.get_parameter("enable_target_tracking").get_parameter_value().bool_value
        )
        self._track_match_max_px = float(
            self.get_parameter("track_match_max_px").get_parameter_value().double_value
        )
        self._track_max_age_sec = float(
            self.get_parameter("track_max_age_sec").get_parameter_value().double_value
        )
        self._track_smoothing_alpha = float(
            self.get_parameter("track_smoothing_alpha").get_parameter_value().double_value
        )
        self._min_track_hits_for_publish = int(
            self.get_parameter("min_track_hits_for_publish").get_parameter_value().integer_value
        )
        self._max_targets_3d_per_frame = int(
            self.get_parameter("max_targets_3d_per_frame").get_parameter_value().integer_value
        )
        self._track_debug_topic = self.get_parameter("track_debug_topic").get_parameter_value().string_value
        self._pub_track_debug = self.create_publisher(String, self._track_debug_topic, 10)
        self._publish_target_markers = bool(
            self.get_parameter("publish_target_markers").get_parameter_value().bool_value
        )
        self._markers_use_flat_map_z = bool(
            self.get_parameter("target_markers_use_flat_map_z").get_parameter_value().bool_value
        )
        self._markers_map_z = float(
            self.get_parameter("target_markers_map_z").get_parameter_value().double_value
        )
        self._pub_track_markers: Any = None
        self._pub_target_map_valid: Any = None
        self._marker_publish_only_eligible = bool(
            self.get_parameter("marker_publish_only_eligible").get_parameter_value().bool_value
        )
        self._clear_target_point_when_lost = bool(
            self.get_parameter("clear_target_point_when_lost").get_parameter_value().bool_value
        )
        self._published_marker_ids: set[int] = set()
        self._tracks: dict[int, dict[str, Any]] = {}
        self._next_track_id = 1
        self._desired_object_topic = (
            self.get_parameter("desired_object_topic").get_parameter_value().string_value.strip()
        )
        self._desired_object_ttl_sec = max(
            float(self.get_parameter("desired_object_ttl_sec").get_parameter_value().double_value),
            1.0,
        )
        self._prefer_nearest_when_filtered = bool(
            self.get_parameter("prefer_nearest_when_filtered").get_parameter_value().bool_value
        )
        self._desired_label_raw = ""
        self._desired_class_ids: set[int] | None = None
        self._desired_set_mono = 0.0
        self._last_desired_filter_log_mono = 0.0
        if self._desired_object_topic:
            self.create_subscription(
                String, self._desired_object_topic, self._on_desired_object_label, 10
            )
        self._image_topic = in_topic
        self._got_first_rgb = False
        self._no_rgb_warn_timer = self.create_timer(15.0, self._warn_if_no_rgb_yet)
        if self._publish_target_point_3d_enabled:
            ci_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
            depth_ci_topic = self.get_parameter("depth_camera_info_topic").get_parameter_value().string_value
            camera_topic = self.get_parameter("target_point_camera_topic").get_parameter_value().string_value
            map_topic = self.get_parameter("target_point_map_topic").get_parameter_value().string_value
            label_topic = self.get_parameter("target_label_topic").get_parameter_value().string_value
            self.create_subscription(CameraInfo, ci_topic, self._on_cam_info, qos)
            self.create_subscription(CameraInfo, depth_ci_topic, self._on_depth_cam_info, qos)
            self._pub_target_point_camera = self.create_publisher(PointStamped, camera_topic, 10)
            self._pub_target_point_map = self.create_publisher(PointStamped, map_topic, 10)
            self._pub_target_label = self.create_publisher(String, label_topic, 10)
            self._pub_target_map_valid = self.create_publisher(Bool, f"{sp}/target_map_valid", 10)
            self._tf_buffer_3d = tf2_ros.Buffer()
            self._tf_listener_3d = tf2_ros.TransformListener(self._tf_buffer_3d, self, spin_thread=True)
            _map_mode = "同像素(已配准)" if self._depth_pixels_aligned else "视角映射"
            _sync_mode = (
                f"ApproxSync slop={self._depth_sync_slop_sec:g}s"
                if self._use_depth_sync
                else f"最新深度 |Δt|<={self._max_depth_age_sec:g}s"
            )
            self.get_logger().info(
                f"3D目标点: depth={depth_topic}, depth_cam_info={depth_ci_topic} -> "
                f"{camera_topic}, {map_topic} (target_frame={self._target_frame}); "
                f"{_sync_mode}; RGB-Depth={_map_mode}; stat={self._depth_sample_stat}; "
                f"depth_range_to_optical_z={self._depth_range_to_optical_z}; "
                f"有效 [{self._depth_min_m},{self._depth_max_m}]m, 拒绝>={self._depth_near_clip_reject_m:.2f}m; "
                f"mask_gate={self._target_point_use_seg_mask_depth}"
            )
            if self._use_depth_sync:
                rgb_sub = message_filters.Subscriber(self, Image, in_topic, qos_profile=qos)
                depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos)
                self._depth_sync = message_filters.ApproximateTimeSynchronizer(
                    [rgb_sub, depth_sub],
                    queue_size=self._depth_sync_queue_size,
                    slop=self._depth_sync_slop_sec,
                )
                self._depth_sync.registerCallback(self._on_rgb_depth_sync)
            else:
                self._depth_sync = None
                self.create_subscription(Image, depth_topic, self._on_depth, qos)
            if self._publish_target_markers:
                mtopic = self.get_parameter("target_objects_marker_topic").get_parameter_value().string_value
                self._pub_track_markers = self.create_publisher(MarkerArray, mtopic, 10)
                self.get_logger().info(f"地图多目标 MarkerArray: {mtopic}（与 PointStamped 主目标可同时开）")
        if self._enable_target_tracking:
            self.get_logger().info(
                f"目标追踪开启: max_px={self._track_match_max_px:g}, max_age={self._track_max_age_sec:g}s, "
                f"alpha={self._track_smoothing_alpha:g}, min_hits={self._min_track_hits_for_publish}"
            )
        pub_az = self.get_parameter("publish_target_azimuths").get_parameter_value().bool_value
        if pub_az:
            ci_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            self._laser_frame = self.get_parameter("laser_frame_id").get_parameter_value().string_value
            self._ground_frame = self.get_parameter("ground_frame_id").get_parameter_value().string_value
            _raw = self.get_parameter("target_azimuth_mode").get_parameter_value().string_value.strip().lower()
            # 空串时勿回退到 tf_geometry（否则与默认 linear_fov 矛盾；常见于未重装旧参数文件）
            _mode_raw = _raw if _raw else "linear_fov"
            if _mode_raw not in ("tf_geometry", "linear_fov"):
                self.get_logger().warning(f"未知 target_azimuth_mode={_mode_raw!r}，改用 tf_geometry")
                _mode_raw = "tf_geometry"
            self._azimuth_mode = _mode_raw
            hfov_deg = float(self.get_parameter("target_azimuth_linear_hfov_deg").get_parameter_value().double_value)
            self._linear_hfov_rad = math.radians(hfov_deg) if hfov_deg > 0.0 else None
            yaw_deg = float(
                self.get_parameter("target_azimuth_linear_camera_yaw_deg").get_parameter_value().double_value
            )
            self._linear_camera_yaw_rad = math.radians(yaw_deg)
            self._linear_use_mask = bool(
                self.get_parameter("target_azimuth_linear_use_mask").get_parameter_value().bool_value
            )
            if self._azimuth_mode == "tf_geometry":
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
            else:
                self._tf_buffer = None
                self._tf_listener = None
            self.create_subscription(CameraInfo, ci_topic, self._on_cam_info, qos)
            az_topic = self.get_parameter("target_azimuth_topic").get_parameter_value().string_value
            self._pub_azimuth = self.create_publisher(JointState, az_topic, 10)
            _mode_desc = (
                "linear_fov（框宽占比×水平视场，无 TF）"
                if self._azimuth_mode == "linear_fov"
                else "tf_geometry（内参+TF+地面）"
            )
            self.get_logger().info(
                f"检测目标方位角: {az_topic}（JointState；模式: {_mode_desc}；CameraInfo: {ci_topic}）"
            )
            if self.get_parameter("publish_azimuth_debug").get_parameter_value().bool_value:
                dbg_t = self.get_parameter("azimuth_debug_topic").get_parameter_value().string_value
                self._pub_az_dbg = self.create_publisher(String, dbg_t, 10)
                self.get_logger().info(
                    f"方位角调试字符串: {dbg_t}（ros2 topic echo；与 markers 共用话题则两行交替出现）"
                )
            self._log_az_console = bool(
                self.get_parameter("log_target_azimuth_to_console").get_parameter_value().bool_value
            )
            self._log_az_interval = max(
                0.0, float(self.get_parameter("log_target_azimuth_interval_sec").get_parameter_value().double_value)
            )
            if self._log_az_console:
                _iv = self._log_az_interval
                _iv_txt = f"{_iv:g} 秒" if _iv > 0.0 else "每一推理帧（输出很密）"
                self.get_logger().info(
                    f"方位角命令行输出: 间隔 {_iv_txt}（log_target_azimuth_to_console）"
                )

        # 标注图必须走 RGB 直订；仅 depth ApproxSync 时若 RGB 话题错误则 annotated 永不发布
        self.create_subscription(Image, in_topic, self._on_image, qos)
        _fp16_s = "on" if self._use_fp16 and str(self._device).startswith("cuda") else "off"
        self.get_logger().info(
            f"订阅 {in_topic} -> 发布 {out_topic}；"
            f"叠图={self._overlay_mode} fp16(cuda)={_fp16_s} imgsz={self._imgsz}；"
            f"COCO类别过滤 {self._classes if self._classes else '全部（CPU/GPU更重）'}；"
            f"地图上多目标可看 MarkerArray topic（publish_target_markers）"
        )

    def _resolve_target_classes(self) -> list[int] | None:
        csv = self.get_parameter("target_class_ids_csv").get_parameter_value().string_value.strip()
        if csv.lower() in ("all", "*", "-1"):
            return None
        if csv:
            parsed: list[int] = []
            for part in csv.split(","):
                part = part.strip()
                if not part:
                    continue
                try:
                    parsed.append(int(part))
                except ValueError:
                    self.get_logger().warning(f"target_class_ids_csv 忽略片段: {part!r}")
            if parsed:
                return parsed
            self.get_logger().warning("target_class_ids_csv 无有效整数，回退到参数 target_class_ids")
        ids_msg = self.get_parameter("target_class_ids").get_parameter_value().integer_array_value
        return list(ids_msg) if ids_msg else None

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._cam_info = msg

    def _resolve_yolo_device(self) -> str:
        raw = self.get_parameter("device").get_parameter_value().string_value.strip()
        low = raw.lower()
        if low in ("", "auto"):
            try:
                import torch

                torch.set_num_threads(1)
                torch.set_num_interop_threads(1)
                if torch.cuda.is_available():
                    name = torch.cuda.get_device_name(0)
                    self.get_logger().info(
                        f"YOLO device=auto -> cuda:0 ({name})；torch CPU 线程限制为 1 以减轻与 rclpy 争用"
                    )
                    return "cuda:0"
            except ImportError:
                pass
            self.get_logger().info("YOLO device=auto -> cpu（未检测到 CUDA 或仅安装 CPU 版 torch）")
            return "cpu"
        self.get_logger().info(f"YOLO device（手动指定）: {raw}")
        return raw

    def _publish_detection_stats(self, count: int, max_conf: float) -> None:
        if not self._pub_stats:
            return
        assert self._pub_detection_count is not None
        self._pub_detection_count.publish(Int32(data=count))
        self._pub_detection_max_conf.publish(Float32(data=max_conf))
        self._pub_detection_present.publish(Bool(data=count > 0))

    @staticmethod
    def _detection_box_count_and_max_conf(results: list[Any]) -> tuple[int, float]:
        if not results:
            return 0, 0.0
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return 0, 0.0
        confs = boxes.conf
        if hasattr(confs, "detach"):
            arr = confs.detach().cpu().float().numpy()
            return int(len(boxes)), float(arr.max()) if arr.size else 0.0
        return int(len(boxes)), float(np.max(confs)) if confs is not None else 0.0

    def _annotate_boxes_bgr(self, img: np.ndarray, results: list[Any]) -> np.ndarray:
        """仅画框+标签（避免 Results.plot() 在分割模型上绘制整幅 mask，CPU 开销极大）。"""
        import cv2

        out = np.ascontiguousarray(img)
        if not results:
            return out
        r0 = results[0]
        boxes = getattr(r0, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return out
        xyxy = boxes.xyxy
        if hasattr(xyxy, "detach"):
            xyxy = xyxy.detach().cpu().numpy()
        confs = boxes.conf
        if hasattr(confs, "detach"):
            confs = confs.detach().cpu().numpy()
        classes = boxes.cls
        if hasattr(classes, "detach"):
            classes = classes.detach().cpu().numpy()
        names = getattr(r0, "names", {}) if hasattr(r0, "names") else {}
        color = (0, 220, 0)
        white = (255, 255, 255)
        for i in range(len(xyxy)):
            x1, y1, x2, y2 = (int(xyxy[i][j]) for j in range(4))
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)
            cls_id = int(classes[i]) if len(classes) > i else -1
            lab = str(names.get(cls_id, cls_id)) if isinstance(names, dict) else str(cls_id)
            c = float(confs[i]) if len(confs) > i else 0.0
            caption = f"{lab} {c:.2f}"
            (tw, th), bl = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ty = max(y1 - 4, th + 6)
            cv2.rectangle(out, (x1, ty - th - 6), (x1 + tw + 4, ty + bl - 2), color, -1)
            cv2.putText(out, caption, (x1 + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 1, cv2.LINE_AA)
        return out

    @staticmethod
    def _stamp_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _scaled_intrinsics(ci: CameraInfo, img_w: int, img_h: int) -> tuple[float, float, float, float]:
        if len(ci.k) < 9:
            return (0.0, 0.0, 0.0, 0.0)
        fx, cx, fy, cy = float(ci.k[0]), float(ci.k[2]), float(ci.k[4]), float(ci.k[5])
        cw, ch = int(ci.width), int(ci.height)
        if cw <= 0 or ch <= 0 or (cw == img_w and ch == img_h):
            return fx, cx, fy, cy
        sx = float(img_w) / float(cw)
        sy = float(img_h) / float(ch)
        return fx * sx, cx * sx, fy * sy, cy * sy

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth_msg = msg

    def _on_depth_cam_info(self, msg: CameraInfo) -> None:
        self._depth_cam_info = msg

    def _decode_depth_meters(self, depth_msg: Image) -> np.ndarray | None:
        try:
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            now = time.monotonic()
            if now - self._last_depth_warn_mono > 2.0:
                self._last_depth_warn_mono = now
                self.get_logger().warning(f"深度图解码失败: {e}")
            return None
        if depth is None or len(depth.shape) != 2:
            return None
        enc = (depth_msg.encoding or "").lower()
        if "16uc1" in enc or "mono16" in enc:
            divisor = self._depth_unit_divisor if self._depth_unit_divisor > 1e-6 else 1000.0
            out = depth.astype(np.float32) / float(divisor)
        else:
            out = depth.astype(np.float32)
            finite_pos = out[np.isfinite(out) & (out > 0)]
            if finite_pos.size >= 16:
                p50 = float(np.median(finite_pos))
                if p50 > 50.0:
                    out = out / 1000.0
        out[(out <= 0) | ~np.isfinite(out)] = np.nan
        return out

    def _log_depth_diagnostic_once(self, depth_msg: Image, depth_m: np.ndarray) -> None:
        if self._depth_diag_logged:
            return
        self._depth_diag_logged = True
        finite = np.isfinite(depth_m) & (depth_m > 0)
        in_range = finite & (depth_m >= self._depth_min_m) & (depth_m < self._depth_max_m)
        n_fin = int(np.count_nonzero(finite))
        n_ok = int(np.count_nonzero(in_range))
        p50 = float(np.nanmedian(depth_m[in_range])) if n_ok > 0 else float("nan")
        self.get_logger().info(
            f"深度诊断 enc={depth_msg.encoding!r} shape={depth_m.shape[1]}x{depth_m.shape[0]} "
            f"finite>0={n_fin} valid[{self._depth_min_m},{self._depth_max_m})m={n_ok} p50={p50:.3f}m"
        )
        if n_ok < max(32, depth_m.size // 200):
            hint = (
                "注册深度几乎无有效像素：请 export TB3_YOLO_DEPTH_REGISTER=0 使用原始 /tb3_depth_only；"
                "或确认 camera_rgb_optical_frame↔camera_depth_optical_frame TF 已发布"
            )
            if self._depth_pixels_aligned:
                self.get_logger().warning(hint)

    def _depth_value_plausible_m(self, z: float) -> bool:
        if not np.isfinite(z) or z < self._depth_min_m or z > self._depth_max_m:
            return False
        if z >= self._depth_near_clip_reject_m:
            return False
        return True

    def _sample_depth_from_array(self, depth_m: np.ndarray, u: int, v: int) -> float | None:
        h, w = depth_m.shape[:2]
        if w <= 0 or h <= 0:
            return None
        u0 = max(0, min(w - 1, u))
        v0 = max(0, min(h - 1, v))
        radius = max(0, int(self._depth_sample_radius_px))
        x1, x2 = max(0, u0 - radius), min(w, u0 + radius + 1)
        y1, y2 = max(0, v0 - radius), min(h, v0 + radius + 1)
        patch = depth_m[y1:y2, x1:x2]
        if patch.size == 0:
            return None
        valid = (
            np.isfinite(patch)
            & (patch > self._depth_min_m)
            & (patch < self._depth_max_m)
            & (patch < self._depth_near_clip_reject_m)
        )
        if not np.any(valid):
            return None
        # 目标定位取最近有效深度，避免 median 被同视线上的远墙/远平面拉高。
        return float(np.min(patch[valid]))

    @staticmethod
    def _rasterize_instance_mask_polygon(poly_xy: np.ndarray, iw: int, ih: int) -> np.ndarray | None:
        """实例多边形 → 与 RGB 对齐的 uint8 mask（1=前景）。"""
        import cv2

        if poly_xy is None or len(poly_xy) < 3:
            return None
        mask = np.zeros((ih, iw), dtype=np.uint8)
        pts = np.ascontiguousarray(np.round(poly_xy).astype(np.int32)).reshape(-1, 1, 2)
        cv2.fillPoly(mask, [pts], 1)
        if int(mask.max()) == 0:
            return None
        return mask

    def _neighborhood_fully_inside_mask(self, mask: np.ndarray, uc: int, vc: int, r: int) -> bool:
        ih, iw = mask.shape[:2]
        for dv in range(-r, r + 1):
            for du in range(-r, r + 1):
                uu, vv = uc + du, vc + dv
                if uu < 0 or uu >= iw or vv < 0 or vv >= ih:
                    return False
                if int(mask[vv, uu]) == 0:
                    return False
        return True

    def _depth_scalar_depth_frame(self, depth_m: np.ndarray, ud: int, vd: int) -> float | None:
        h, w = depth_m.shape[:2]
        if w <= 0 or h <= 0 or ud < 0 or ud >= w or vd < 0 or vd >= h:
            return None
        z = float(depth_m[vd, ud])
        if not self._depth_value_plausible_m(z):
            return None
        return z

    def _map_rgb_pixel_to_depth_pixel(
        self,
        u_rgb: float,
        v_rgb: float,
        iw: int,
        ih: int,
        dw: int,
        dh: int,
        depth_fx: float,
        depth_fy: float,
        depth_cx: float,
        depth_cy: float,
    ) -> tuple[int, int]:
        """RGB 与 tb3_depth 视场/HFOV 不同，禁止仅用 u*dw/iw 线性缩放（易采到远处背景深度 → map 点飞出图外）。"""
        if self._cam_info is not None and len(self._cam_info.k) >= 9:
            rfx, rcx, rfy, rcy = self._scaled_intrinsics(self._cam_info, iw, ih)
            if rfx > 1e-6 and rfy > 1e-6 and depth_fx > 1e-6 and depth_fy > 1e-6:
                theta_u = math.atan((float(u_rgb) - rcx) / rfx)
                theta_v = math.atan((float(v_rgb) - rcy) / rfy)
                u_d = int(round(depth_cx + depth_fx * math.tan(theta_u)))
                v_d = int(round(depth_cy + depth_fy * math.tan(theta_v)))
            else:
                u_d = int(round(u_rgb * float(dw) / float(iw)))
                v_d = int(round(v_rgb * float(dh) / float(ih)))
        else:
            u_d = int(round(u_rgb * float(dw) / float(iw)))
            v_d = int(round(v_rgb * float(dh) / float(ih)))
        return max(0, min(dw - 1, u_d)), max(0, min(dh - 1, v_d))

    def _depth_to_optical_z(
        self, u_d: float, v_d: float, depth_m: float, fx: float, fy: float, cx: float, cy: float
    ) -> float:
        if not self._depth_range_to_optical_z or depth_m <= 0.0:
            return float(depth_m)
        nx = (float(u_d) - cx) / fx
        ny = (float(v_d) - cy) / fy
        nz = 1.0
        denom = math.sqrt(nx * nx + ny * ny + nz * nz)
        if denom < 1e-6:
            return float(depth_m)
        return float(depth_m) * nz / denom

    def _backproj_cam_xyz(
        self, u_d: float, v_d: float, z: float, fx: float, fy: float, cx: float, cy: float
    ) -> tuple[float, float, float]:
        z_opt = self._depth_to_optical_z(u_d, v_d, z, fx, fy, cx, cy)
        x = (float(u_d) - cx) * z_opt / fx
        y = (float(v_d) - cy) * z_opt / fy
        return x, y, z_opt

    def _fallback_box_center_depth(
        self,
        depth_m: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        iw: int,
        ih: int,
        dw: int,
        dh: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> tuple[float, float, float, float, float] | None:
        u_rgb = float(0.5 * (x1 + x2))
        v_rgb = float(0.5 * (y1 + y2))
        u_d, v_d = self._map_rgb_pixel_to_depth_pixel(
            u_rgb, v_rgb, iw, ih, dw, dh, fx, fy, cx, cy
        )
        z = self._sample_depth_from_array(depth_m, u_d, v_d)
        if z is None:
            return None
        x, y, z3 = self._backproj_cam_xyz(float(u_d), float(v_d), z, fx, fy, cx, cy)
        return u_rgb, v_rgb, x, y, z3

    def _fallback_box_grid_min_depth(
        self,
        depth_m: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        iw: int,
        ih: int,
        dw: int,
        dh: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        grid_n: int = 7,
    ) -> tuple[float, float, float, float, float] | None:
        """在检测框内网格采样，取最近有效深度（mask/框心失败时，高机位俯视更稳）。"""
        xa, xb = float(min(x1, x2)), float(max(x1, x2))
        ya, yb = float(min(y1, y2)), float(max(y1, y2))
        n = max(3, int(grid_n))
        us = np.linspace(xa, xb, num=n)
        vs = np.linspace(ya, yb, num=n)
        best_z = float("inf")
        best: tuple[float, float, float, float, float] | None = None
        for u_rgb in us:
            for v_rgb in vs:
                u_d, v_d = self._map_rgb_pixel_to_depth_pixel(
                    float(u_rgb), float(v_rgb), iw, ih, dw, dh, fx, fy, cx, cy
                )
                z = self._depth_scalar_depth_frame(depth_m, u_d, v_d)
                if z is None or z >= best_z:
                    continue
                x, y, z3 = self._backproj_cam_xyz(float(u_d), float(v_d), z, fx, fy, cx, cy)
                best_z = z3
                best = (float(u_rgb), float(v_rgb), x, y, z3)
        if best is not None:
            return best
        return self._fallback_box_center_depth(
            depth_m, x1, y1, x2, y2, iw, ih, dw, dh, fx, fy, cx, cy
        )

    def _probe_box_depth_debug(
        self,
        depth_m: np.ndarray,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        iw: int,
        ih: int,
        dw: int,
        dh: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> str:
        projector = RgbDepthProjector.from_images(
            self._cam_info,
            self._depth_cam_info,
            iw,
            ih,
            dw,
            dh,
            self._depth_pixels_aligned,
        )
        u_rgb = float(0.5 * (x1 + x2))
        v_rgb = float(0.5 * (y1 + y2))
        u_d, v_d = projector.rgb_uv_to_depth_uv(u_rgb, v_rgb)
        raw = float("nan")
        if 0 <= u_d < dw and 0 <= v_d < dh:
            raw = float(depth_m[v_d, u_d])
        roi = sample_depth_roi(
            depth_m,
            projector,
            x1,
            y1,
            x2,
            y2,
            mask_rgb=None,
            depth_min_m=self._depth_min_m,
            depth_max_m=self._depth_max_m,
            near_clip_reject_m=self._depth_near_clip_reject_m,
            stat=self._depth_sample_stat,  # type: ignore[arg-type]
            grid_n=5,
        )
        z_roi = roi[4] if roi is not None else float("nan")
        return f"框心原始={raw:.3f}m ROI_{self._depth_sample_stat}={z_roi:.3f}m"

    def _resolve_camera_xyz_with_mask(
        self,
        depth_m: np.ndarray,
        mask_rgb: np.ndarray | None,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        iw: int,
        ih: int,
        dw: int,
        dh: int,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
    ) -> tuple[float, float, float, float, float] | None:
        """返回 (u_rgb,v_rgb,x,y,z) 相机系；在深度 ROI 内稳健采样。"""
        if self._depth_cam_info is None:
            return None
        use_mask = mask_rgb if self._target_point_use_seg_mask_depth else None
        projector = RgbDepthProjector.from_images(
            self._cam_info,
            self._depth_cam_info,
            iw,
            ih,
            dw,
            dh,
            self._depth_pixels_aligned,
        )
        got = None
        for mask_try in (use_mask, None) if use_mask is not None else (None,):
            got = sample_depth_roi(
                depth_m,
                projector,
                x1,
                y1,
                x2,
                y2,
                mask_rgb=mask_try,
                depth_min_m=self._depth_min_m,
                depth_max_m=self._depth_max_m,
                near_clip_reject_m=self._depth_near_clip_reject_m,
                stat=self._depth_sample_stat,  # type: ignore[arg-type]
                grid_n=self._depth_roi_grid_n,
            )
            if got is not None:
                break
        if got is None:
            fb = self._fallback_box_grid_min_depth(
                depth_m, x1, y1, x2, y2, iw, ih, dw, dh, fx, fy, cx, cy
            )
            if fb is None:
                return None
            return fb
        u_rgb, v_rgb, _x, _y, z_raw = got
        ud, vd = projector.rgb_uv_to_depth_uv(u_rgb, v_rgb)
        x, y, z3 = sample_depth_with_optical_z(
            float(ud),
            float(vd),
            z_raw,
            fx,
            fy,
            cx,
            cy,
            self._depth_range_to_optical_z,
        )
        return u_rgb, v_rgb, x, y, z3

    def _project_to_map(self, pt_cam: PointStamped) -> PointStamped | None:
        if self._tf_buffer_3d is None or not self._target_frame:
            return None
        # 仿真 map<-AMCL 常比深度帧晚数百 ms；先查最新 TF，再按深度时间戳（AMCL 已稳定时更准）。
        attempts: tuple[Time, ...] = (Time(), Time.from_msg(pt_cam.header.stamp))
        last_err: Exception | None = None
        for when in attempts:
            try:
                trans = self._tf_buffer_3d.lookup_transform(
                    self._target_frame,
                    pt_cam.header.frame_id,
                    when,
                    timeout=Duration(seconds=0, nanoseconds=500_000_000),
                )
                return do_transform_point(pt_cam, trans)
            except Exception as e:
                last_err = e
        if last_err is not None:
            now = time.monotonic()
            if now - self._last_depth_warn_mono > 2.0:
                self._last_depth_warn_mono = now
                self.get_logger().warning(
                    f"3D点TF变换失败 {pt_cam.header.frame_id}->{self._target_frame}: {last_err} "
                    f"(请确认仿真/AMCL 已发布 map<-odom<-base_link，且与 YOLO 同为 use_sim_time)"
                )
        return None

    @staticmethod
    def _rgba_for_class_id(cls_id: int) -> ColorRGBA:
        c = ColorRGBA()
        known = {
            0: (0.15, 0.75, 1.0),
            39: (0.2, 0.85, 0.35),
            41: (0.95, 0.25, 0.55),
            75: (0.75, 0.35, 0.95),
            62: (0.35, 0.9, 0.35),
        }
        if cls_id in known:
            c.r, c.g, c.b = known[cls_id]
        else:
            u = float((cls_id * 1103515245 + 12345) & 0x7FFFFFFF)
            hr = math.radians((u % 360.0))
            c.r = 0.45 + 0.45 * math.cos(hr)
            c.g = 0.45 + 0.45 * math.cos(hr + 2.094)
            c.b = 0.45 + 0.45 * math.cos(hr + 4.189)
        c.a = 0.9
        return c

    def _marker_tracks_for_display(self, now_sec: float) -> list[tuple[int, dict[str, Any]]]:
        """地图上只展示当前帧仍有效、且（有语义过滤时）符合 desired 的 track。"""
        if self._marker_publish_only_eligible:
            return [(int(tr.get("id", -1)), tr) for tr in self._eligible_tracks(now_sec)]
        out: list[tuple[int, dict[str, Any]]] = []
        for tid, tr in self._tracks.items():
            hits = int(tr.get("hits", 0))
            age = now_sec - float(tr.get("last_seen_sec", now_sec))
            if hits < self._min_track_hits_for_publish or age > self._track_max_age_sec:
                continue
            if tr.get("map_xyz") is None:
                continue
            tr_copy = dict(tr)
            tr_copy["id"] = tid
            out.append((tid, tr_copy))
        return out

    def _delete_marker_id(self, ma: MarkerArray, mstamp: Any, tid: int) -> None:
        for ns in ("yolo_obj_sphere", "yolo_obj_label"):
            m = Marker()
            m.header.frame_id = self._target_frame
            m.header.stamp = mstamp
            m.ns = ns
            m.id = int(tid)
            m.action = Marker.DELETE
            ma.markers.append(m)

    def _publish_track_marker_array(self, now_sec: float) -> None:
        pub = self._pub_track_markers
        if pub is None:
            return
        mstamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        active_ids: set[int] = set()
        for tid, tr in self._marker_tracks_for_display(now_sec):
            if tid < 0:
                continue
            map_xyz = tr.get("map_xyz")
            if map_xyz is None:
                continue
            active_ids.add(tid)
            cls_id = int(tr.get("cls_id", -1))
            label = str(tr.get("label", "?"))
            col = self._rgba_for_class_id(cls_id)

            sph = Marker()
            sph.header.frame_id = self._target_frame
            sph.header.stamp = mstamp
            sph.ns = "yolo_obj_sphere"
            sph.id = int(tid)
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position.x = float(map_xyz[0])
            sph.pose.position.y = float(map_xyz[1])
            if self._markers_use_flat_map_z:
                sph.pose.position.z = float(self._markers_map_z)
            else:
                sph.pose.position.z = float(map_xyz[2]) + 0.06
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.22
            sph.color = col
            ma.markers.append(sph)

            txt = Marker()
            txt.header.frame_id = self._target_frame
            txt.header.stamp = mstamp
            txt.ns = "yolo_obj_label"
            txt.id = int(tid)
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(map_xyz[0])
            txt.pose.position.y = float(map_xyz[1])
            if self._markers_use_flat_map_z:
                txt.pose.position.z = float(self._markers_map_z) + 0.32
            else:
                txt.pose.position.z = float(map_xyz[2]) + 0.4
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.14
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 0.95
            txt.text = f"{label} id{tid}"
            ma.markers.append(txt)

        for stale_id in self._published_marker_ids - active_ids:
            self._delete_marker_id(ma, mstamp, stale_id)
        self._published_marker_ids = active_ids
        if not active_ids:
            for ns in ("yolo_obj_sphere", "yolo_obj_label"):
                clr = Marker()
                clr.header.frame_id = self._target_frame
                clr.header.stamp = mstamp
                clr.ns = ns
                clr.action = Marker.DELETEALL
                ma.markers.append(clr)
        pub.publish(ma)

    def _publish_target_map_valid(self, valid: bool) -> None:
        if self._pub_target_map_valid is None:
            return
        self._pub_target_map_valid.publish(Bool(data=valid))

    def _cleanup_tracks(self, now_sec: float) -> None:
        stale = [
            tid
            for tid, tr in self._tracks.items()
            if (now_sec - float(tr.get("last_seen_sec", now_sec))) > self._track_max_age_sec
        ]
        for tid in stale:
            del self._tracks[tid]

    def _match_track(self, cls_id: int, u: float, v: float, now_sec: float, used: set[int]) -> int | None:
        best_tid = None
        best_d = float("inf")
        for tid, tr in self._tracks.items():
            if tid in used:
                continue
            if int(tr.get("cls_id", -1)) != int(cls_id):
                continue
            if (now_sec - float(tr.get("last_seen_sec", 0.0))) > self._track_max_age_sec:
                continue
            du = float(tr.get("u", u)) - u
            dv = float(tr.get("v", v)) - v
            d = math.hypot(du, dv)
            if d < self._track_match_max_px and d < best_d:
                best_d = d
                best_tid = tid
        return best_tid

    def _update_track(self, tid: int, cand: dict[str, Any], now_sec: float) -> None:
        tr = self._tracks.get(tid)
        alpha = max(0.0, min(1.0, self._track_smoothing_alpha))
        if tr is None:
            self._tracks[tid] = {
                "id": int(tid),
                "cls_id": cand["cls_id"],
                "label": cand["label"],
                "conf": cand["conf"],
                "u": cand["u"],
                "v": cand["v"],
                "cam_xyz": cand["cam_xyz"],
                "map_xyz": cand.get("map_xyz"),
                "hits": 1,
                "last_seen_sec": now_sec,
            }
            return
        tr["label"] = cand["label"]
        tr["conf"] = cand["conf"]
        tr["u"] = cand["u"]
        tr["v"] = cand["v"]
        prev_cam = tr.get("cam_xyz")
        if prev_cam is None:
            tr["cam_xyz"] = cand["cam_xyz"]
        else:
            tr["cam_xyz"] = tuple(
                alpha * float(cand["cam_xyz"][i]) + (1.0 - alpha) * float(prev_cam[i])
                for i in range(3)
            )
        if cand.get("map_xyz") is not None:
            prev_map = tr.get("map_xyz")
            if prev_map is None:
                tr["map_xyz"] = cand["map_xyz"]
            else:
                tr["map_xyz"] = tuple(
                    alpha * float(cand["map_xyz"][i]) + (1.0 - alpha) * float(prev_map[i])
                    for i in range(3)
                )
        tr["id"] = int(tid)
        tr["hits"] = int(tr.get("hits", 0)) + 1
        tr["last_seen_sec"] = now_sec

    def _desired_filter_active(self, now_sec: float) -> bool:
        if self._desired_class_ids is None:
            return False
        if not self._desired_label_raw:
            return False
        if (now_sec - self._desired_set_mono) > self._desired_object_ttl_sec:
            return False
        return True

    def _on_desired_object_label(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        now_sec = time.monotonic()
        if not raw:
            self._desired_label_raw = ""
            self._desired_class_ids = None
            self._desired_set_mono = 0.0
            self.get_logger().info("[yolo_object_seg] 已清除语义目标过滤（desired_object_label 为空）")
            return
        ids = resolve_desired_coco_class_ids(raw)
        self._desired_label_raw = raw
        self._desired_class_ids = ids
        self._desired_set_mono = now_sec
        if ids is None:
            self.get_logger().info(
                f"[yolo_object_seg] 语义目标={raw!r} → 不按类别过滤（在 target_class_ids 内按置信度选主目标）"
            )
        elif not ids:
            self.get_logger().warning(
                f"[yolo_object_seg] 语义目标={raw!r} 无法映射到 COCO 类，暂停发布绿点直至收到有效标签"
            )
        else:
            self.get_logger().info(
                f"[yolo_object_seg] 语义目标={raw!r} → 仅 class_ids={sorted(ids)}；"
                f"{'视野最近(深度)' if self._prefer_nearest_when_filtered else '置信度最高'}"
            )

    def _eligible_tracks(self, now_sec: float) -> list[dict[str, Any]]:
        pool: list[dict[str, Any]] = []
        for tid, tr in self._tracks.items():
            age = now_sec - float(tr.get("last_seen_sec", now_sec))
            hits = int(tr.get("hits", 0))
            if age > self._track_max_age_sec or hits < self._min_track_hits_for_publish:
                continue
            if tr.get("map_xyz") is None:
                continue
            tr_out = dict(tr)
            tr_out["id"] = int(tid)
            pool.append(tr_out)
        if self._desired_filter_active(now_sec) and self._desired_class_ids is not None:
            if len(self._desired_class_ids) == 0:
                return []
            pool = [tr for tr in pool if int(tr.get("cls_id", -1)) in self._desired_class_ids]
        return pool

    def _choose_primary_track(self, now_sec: float) -> dict[str, Any] | None:
        pool = self._eligible_tracks(now_sec)
        if not pool:
            if self._desired_filter_active(now_sec) and self._desired_class_ids:
                if now_sec - self._last_desired_filter_log_mono > 3.0:
                    self._last_desired_filter_log_mono = now_sec
                    active = [
                        f"id={tid},cls={tr.get('label')}"
                        for tid, tr in self._tracks.items()
                        if (now_sec - float(tr.get("last_seen_sec", 0.0)))
                        <= self._track_max_age_sec
                    ]
                    self.get_logger().warning(
                        f"语义目标={self._desired_label_raw!r} 需要 class_ids={sorted(self._desired_class_ids)}，"
                        f"当前无匹配 track（活跃: {', '.join(active[:8]) or '无'}）"
                    )
            return None

        if self._desired_filter_active(now_sec) and self._prefer_nearest_when_filtered:
            def _nearest_key(tr: dict[str, Any]) -> tuple[float, float, int]:
                cam = tr.get("cam_xyz")
                z = float(cam[2]) if cam is not None else 1e9
                return (z, -float(tr.get("conf", 0.0)), -int(tr.get("hits", 0)))

            return min(pool, key=_nearest_key)

        best = None
        best_key = None
        for tr in pool:
            key = (float(tr.get("conf", 0.0)), int(tr.get("hits", 0)))
            if best_key is None or key > best_key:
                best_key = key
                best = tr
        return best

    def _publish_track_debug(self, now_sec: float) -> None:
        if self._pub_track_debug is None:
            return
        chunks: list[str] = []
        for tid in sorted(self._tracks):
            tr = self._tracks[tid]
            age = now_sec - float(tr.get("last_seen_sec", now_sec))
            chunks.append(
                f"id={tid},label={tr.get('label','')},conf={float(tr.get('conf',0.0)):.2f},"
                f"hits={int(tr.get('hits',0))},age={age:.2f}s"
            )
        self._pub_track_debug.publish(String(data=" | ".join(chunks) if chunks else "no_active_tracks"))

    def _publish_target_point_3d(
        self,
        image_msg: Image,
        results: list[Any],
        depth_msg: Image | None = None,
    ) -> None:
        if not self._publish_target_point_3d_enabled:
            return
        if self._depth_cam_info is None or not results:
            return
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return
        if depth_msg is None:
            depth_msg = self._latest_depth_msg
            if depth_msg is None:
                return
            stamp_rgb = self._stamp_sec(image_msg.header.stamp)
            stamp_dep = self._stamp_sec(depth_msg.header.stamp)
            stamp_skew = abs(stamp_rgb - stamp_dep)
            if stamp_skew > self._max_depth_age_sec:
                now = time.monotonic()
                if now - self._last_depth_warn_mono > 2.0:
                    self._last_depth_warn_mono = now
                    self.get_logger().warning(
                        f"RGB/Depth 时间差过大，跳过3D: |Δt|={stamp_skew:.3f}s > "
                        f"{self._max_depth_age_sec:.3f}s (rgb={stamp_rgb:.3f} depth={stamp_dep:.3f})"
                    )
                return
        depth_m = self._decode_depth_meters(depth_msg)
        if depth_m is None:
            return
        self._log_depth_diagnostic_once(depth_msg, depth_m)

        xyxy = boxes.xyxy
        confs = boxes.conf
        classes = boxes.cls
        if hasattr(xyxy, "detach"):
            xyxy = xyxy.detach().cpu().numpy()
        if hasattr(confs, "detach"):
            confs = confs.detach().cpu().numpy()
        if hasattr(classes, "detach"):
            classes = classes.detach().cpu().numpy()
        if len(xyxy) == 0:
            now_sec = time.monotonic()
            self._cleanup_tracks(now_sec)
            self._publish_track_debug(now_sec)
            self._publish_track_marker_array(now_sec)
            if self._clear_target_point_when_lost:
                self._publish_target_map_valid(False)
            return
        dh, dw = int(depth_m.shape[0]), int(depth_m.shape[1])
        fx, cx, fy, cy = self._scaled_intrinsics(self._depth_cam_info, dw, dh)
        if fx < 1e-6 or fy < 1e-6:
            return
        names = results[0].names if hasattr(results[0], "names") else {}
        order = np.argsort(-confs.astype(np.float32))
        max_n = max(1, int(self._max_targets_3d_per_frame))
        iw = max(1, int(image_msg.width))
        ih = max(1, int(image_msg.height))
        r0 = results[0]
        poly_src = getattr(getattr(r0, "masks", None), "xy", None)
        poly_list: list[Any] | None = poly_src if poly_src is not None else None
        candidates: list[dict[str, Any]] = []
        n_box = min(len(order), max_n)
        for idx in order[:max_n]:
            i = int(idx)
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            mask_rgb = None
            if poly_list is not None and i < len(poly_list) and self._target_point_use_seg_mask_depth:
                poly = np.asarray(poly_list[i], dtype=np.float64)
                mask_rgb = self._rasterize_instance_mask_polygon(poly, iw, ih)
            got = self._resolve_camera_xyz_with_mask(
                depth_m, mask_rgb, x1, y1, x2, y2, iw, ih, dw, dh, fx, fy, cx, cy
            )
            if got is None:
                continue
            u_rgb, v_rgb, x, y, z = got
            cls_id = int(classes[i]) if len(classes) > i else -1
            if isinstance(names, dict):
                label = str(names.get(cls_id, cls_id))
            else:
                label = str(cls_id)
            pt_cam = PointStamped()
            pt_cam.header = depth_msg.header
            pt_cam.point.x = float(x)
            pt_cam.point.y = float(y)
            pt_cam.point.z = float(z)
            pt_map = self._project_to_map(pt_cam)
            map_xyz = None if pt_map is None else (float(pt_map.point.x), float(pt_map.point.y), float(pt_map.point.z))
            candidates.append(
                {
                    "u": float(u_rgb),
                    "v": float(v_rgb),
                    "conf": float(confs[i]) if len(confs) > i else 0.0,
                    "cls_id": cls_id,
                    "label": label,
                    "cam_xyz": (float(x), float(y), float(z)),
                    "map_xyz": map_xyz,
                }
            )
        if n_box > 0 and not candidates:
            now = time.monotonic()
            if now - self._last_depth_warn_mono > 3.0:
                self._last_depth_warn_mono = now
                i0 = int(order[0])
                x1, y1, x2, y2 = [float(v) for v in xyxy[i0]]
                dbg = self._probe_box_depth_debug(
                    depth_m, x1, y1, x2, y2, iw, ih, dw, dh, fx, fy, cx, cy
                )
                self.get_logger().warning(
                    f"检测到 {n_box} 个目标但无有效3D点(有效深度"
                    f" [{self._depth_min_m},{self._depth_max_m}]m, 拒绝>={self._depth_near_clip_reject_m:.1f}m); "
                    f"首目标 {dbg}"
                )
        now_sec = time.monotonic()
        self._cleanup_tracks(now_sec)
        used_tracks: set[int] = set()
        for cand in candidates:
            tid = None
            if self._enable_target_tracking:
                tid = self._match_track(int(cand["cls_id"]), float(cand["u"]), float(cand["v"]), now_sec, used_tracks)
            if tid is None:
                tid = self._next_track_id
                self._next_track_id += 1
            used_tracks.add(tid)
            self._update_track(tid, cand, now_sec)
        primary = self._choose_primary_track(now_sec)
        self._publish_track_debug(now_sec)
        self._publish_track_marker_array(now_sec)
        if primary is None:
            if self._clear_target_point_when_lost:
                self._publish_target_map_valid(False)
            return
        self._publish_target_map_valid(True)
        cam_xyz = primary.get("cam_xyz")
        if cam_xyz is None:
            return
        pt_cam = PointStamped()
        pt_cam.header = depth_msg.header
        pt_cam.point.x = float(cam_xyz[0])
        pt_cam.point.y = float(cam_xyz[1])
        pt_cam.point.z = float(cam_xyz[2])
        if self._pub_target_point_camera is not None:
            self._pub_target_point_camera.publish(pt_cam)
        if self._pub_target_label is not None:
            want = self._desired_label_raw if self._desired_filter_active(time.monotonic()) else ""
            self._pub_target_label.publish(
                String(
                    data=(
                        f"track_id={int(primary.get('id',-1))};label={primary.get('label','')};"
                        f"conf={float(primary.get('conf',0.0)):.3f};hits={int(primary.get('hits',0))};"
                        f"desired={want!r};cls_id={int(primary.get('cls_id',-1))};"
                        f"camera_xyz=({pt_cam.point.x:.3f},{pt_cam.point.y:.3f},{pt_cam.point.z:.3f})"
                    )
                )
            )
        if self._pub_target_point_map is None:
            return
        map_xyz = primary.get("map_xyz")
        if map_xyz is None:
            return
        pt_map = PointStamped()
        pt_map.header.stamp = self.get_clock().now().to_msg()
        pt_map.header.frame_id = self._target_frame
        pt_map.point.x = float(map_xyz[0])
        pt_map.point.y = float(map_xyz[1])
        pt_map.point.z = float(map_xyz[2])
        self._pub_target_point_map.publish(pt_map)

    def _on_rgb_depth_sync(self, _rgb_msg: Image, depth_msg: Image) -> None:
        # 仅缓存同步深度；推理与 annotated 由 _on_image 统一触发，避免双份 predict
        self._synced_depth_msg = depth_msg

    def _on_image(self, msg: Image) -> None:
        depth_msg: Image | None = None
        if self._publish_target_point_3d_enabled:
            depth_msg = self._synced_depth_msg if self._use_depth_sync else self._latest_depth_msg
        self._process_rgb_frame(msg, depth_msg)

    def _warn_if_no_rgb_yet(self) -> None:
        if self._got_first_rgb:
            return
        self.get_logger().warning(
            f"15s 内仍未收到 RGB（当前订阅 {self._image_topic}）。"
            "一体仿真请用 /camera/image_raw + /camera/camera_info；"
            "可 ros2 topic hz /camera/image_raw 确认 Gazebo 是否在发图。"
        )

    def _process_rgb_frame(self, msg: Image, depth_msg: Image | None) -> None:
        if not self._got_first_rgb:
            self._got_first_rgb = True
            self.get_logger().info(
                f"已收到首帧 RGB: topic={self._image_topic} encoding={msg.encoding} "
                f"{msg.width}x{msg.height}"
            )
        now = time.monotonic()
        if self._min_interval > 0.0 and (now - self._last_t) < self._min_interval:
            return
        self._last_t = now

        try:
            if msg.encoding in ("rgb8", "bgr8", "8UC3"):
                enc = "bgr8" if msg.encoding != "rgb8" else "rgb8"
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
                if enc == "rgb8":
                    import cv2

                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"图像解码失败 encoding={msg.encoding}: {e}")
            self._publish_detection_stats(0, 0.0)
            self._publish_target_azimuth_ranges(msg, None, 0)
            return

        kwargs: dict[str, Any] = {
            "conf": self._conf,
            "iou": self._iou,
            "imgsz": self._imgsz,
            "verbose": False,
        }
        kwargs["device"] = self._device
        if self._classes is not None and len(self._classes) > 0:
            kwargs["classes"] = self._classes
        if self._use_fp16 and str(self._device).startswith("cuda"):
            kwargs["half"] = True

        try:
            import torch

            infer_ctx = torch.inference_mode()
        except ImportError:
            infer_ctx = nullcontext()
        with infer_ctx:
            results = self._model.predict(source=img, **kwargs)
        n_box, mx_c = self._detection_box_count_and_max_conf(results)
        self._publish_detection_stats(n_box, mx_c)
        self._publish_target_point_3d(msg, results, depth_msg)
        boxes = results[0].boxes if results else None
        masks_xy = None
        if self._pub_azimuth is not None and self._linear_use_mask:
            if results and results[0].masks is not None:
                try:
                    masks_xy = results[0].masks.xy
                except Exception:
                    masks_xy = None
        image_wh = (int(img.shape[1]), int(img.shape[0]))
        self._publish_target_azimuth_ranges(msg, boxes, n_box, image_wh=image_wh, masks_xy=masks_xy)
        if not results:
            return
        r0 = results[0]
        if self._overlay_mode == "full":
            annotated = r0.plot()
        elif self._overlay_mode == "plot_no_masks":
            annotated = r0.plot(masks=False)
        else:
            annotated = self._annotate_boxes_bgr(img, results)
        try:
            out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"编码输出失败: {e}")
            return
        out.header = msg.header
        self._pub.publish(out)

    def _publish_azimuth_debug_str(self, js: JointState, reason: str) -> None:
        data = list(js.position)
        n = len(data) // 2
        deg_chunks: list[str] = []
        for i in range(0, len(data) - 1, 2):
            lo, hi = float(data[i]), float(data[i + 1])
            deg_chunks.append(f"{math.degrees(norm_angle(lo)):.1f}~{math.degrees(norm_angle(hi)):.1f}°")
        ci_ok = self._cam_info is not None
        line = (
            f"[目标方位角] {reason} laser={self._laser_frame} pairs={n} "
            f"cam_info={'ok' if ci_ok else 'MISSING'} "
            f"deg=[{', '.join(deg_chunks)}] rad={data!r}"
        )
        if self._pub_az_dbg is not None:
            self._pub_az_dbg.publish(String(data=line))
        if self._log_az_console:
            now = time.monotonic()
            if self._log_az_interval <= 0.0 or (now - self._last_azimuth_console_mono) >= self._log_az_interval:
                self._last_azimuth_console_mono = now
                self.get_logger().info(line)

    def _publish_target_azimuth_ranges(
        self,
        image_msg: Image,
        boxes: Any,
        n_boxes: int,
        image_wh: tuple[int, int] | None = None,
        masks_xy: list[Any] | None = None,
    ) -> None:
        if self._pub_azimuth is None:
            return
        js = JointState()
        js.header = image_msg.header
        js.name = []
        if n_boxes == 0 or boxes is None or len(boxes) == 0:
            js.position = []
            self._pub_azimuth.publish(js)
            self._publish_azimuth_debug_str(js, "no_detection_boxes")
            return
        if self._azimuth_mode != "linear_fov" and self._cam_info is None:
            now = time.monotonic()
            if now - self._last_cam_warn_mono > 5.0:
                self._last_cam_warn_mono = now
                self.get_logger().warning("尚未收到 CameraInfo，无法发布 target_azimuth_ranges（tf_geometry 模式必需）")
            js.position = []
            self._pub_azimuth.publish(js)
            self._publish_azimuth_debug_str(js, "no_camera_info")
            return
        if self._azimuth_mode == "tf_geometry" and self._tf_buffer is None:
            return
        from human_yolo_seg.utils.target_azimuth import boxes_to_azimuth_data

        data = boxes_to_azimuth_data(
            self._tf_buffer,
            self._cam_info,
            image_msg,
            boxes,
            self._laser_frame,
            self._ground_frame,
            self.get_logger(),
            image_wh=image_wh,
            masks_xy=masks_xy,
            mode=self._azimuth_mode,
            linear_hfov_rad=self._linear_hfov_rad,
            linear_camera_yaw_rad=self._linear_camera_yaw_rad,
            linear_use_mask=self._linear_use_mask,
        )
        js.position = [float(x) for x in data]
        self._pub_azimuth.publish(js)
        _reason = "ok_linear_fov" if self._azimuth_mode == "linear_fov" else "ok_tf_geometry"
        self._publish_azimuth_debug_str(js, _reason)


def main() -> None:
    rclpy.init()
    node = YoloObjectSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
