# pyright: reportMissingImports=false
"""订阅相机 RGB，Ultralytics YOLO-Seg 定位多类物体（默认检测类别包含 COCO 的「人」(0) 与椅子 (56)）。

发布：标注图像、地图 MarkerArray（多目标）、主目标 PointStamped、检测统计话题（默认前缀 /yolo_objects）。

默认权重文件名 yolo26n-seg.pt：放在包内 models/ 目录后 colcon build，或 model_path 传绝对路径。
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

import tf2_ros

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
        # COCO 类别示例：索引 0 常标「人」，56 常标椅子；也可用 target_class_ids_csv 任意组合。
        self.declare_parameter("target_class_ids", [0, 56])
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
        self.declare_parameter("target_frame_id", "map")
        self.declare_parameter("depth_unit_divisor", 1000.0)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 4.5)
        self.declare_parameter("depth_sample_radius_px", 3)
        self.declare_parameter("max_depth_age_sec", 0.35)
        self.declare_parameter("enable_target_tracking", True)
        self.declare_parameter("track_match_max_px", 90.0)
        self.declare_parameter("track_max_age_sec", 2.0)
        self.declare_parameter("track_smoothing_alpha", 0.65)
        self.declare_parameter("min_track_hits_for_publish", 2)
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

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # 与 Reliable 发布端对齐。depth=10 时推理慢会在队列里堆旧帧，越跑越「只剩约 1Hz」；depth=1 只处理最新图。
        qos_rgb = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # 输出图：用 Reliable。RViz2 Image 常见默认 Reliable；若发布端为 Best Effort 则 QoS 不兼容 → 一直 No image，
        # 而 ros2 topic hz/echo 可能仍能看到（工具侧订阅配置不同）。depth 略大以缓冲 1080p + 慢推理。
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
        self._depth_cam_info: CameraInfo | None = None
        self._last_depth_warn_mono = 0.0
        self._publish_target_point_3d_enabled = bool(
            self.get_parameter("publish_target_point_3d").get_parameter_value().bool_value
        )
        self._depth_unit_divisor = float(
            self.get_parameter("depth_unit_divisor").get_parameter_value().double_value
        )
        self._depth_min_m = float(self.get_parameter("depth_min_m").get_parameter_value().double_value)
        self._depth_max_m = float(self.get_parameter("depth_max_m").get_parameter_value().double_value)
        self._depth_sample_radius_px = int(
            self.get_parameter("depth_sample_radius_px").get_parameter_value().integer_value
        )
        self._max_depth_age_sec = float(
            self.get_parameter("max_depth_age_sec").get_parameter_value().double_value
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
        self._tracks: dict[int, dict[str, Any]] = {}
        self._next_track_id = 1
        if self._publish_target_point_3d_enabled:
            ci_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
            depth_ci_topic = self.get_parameter("depth_camera_info_topic").get_parameter_value().string_value
            camera_topic = self.get_parameter("target_point_camera_topic").get_parameter_value().string_value
            map_topic = self.get_parameter("target_point_map_topic").get_parameter_value().string_value
            label_topic = self.get_parameter("target_label_topic").get_parameter_value().string_value
            self.create_subscription(CameraInfo, ci_topic, self._on_cam_info, qos_rgb)
            self.create_subscription(CameraInfo, depth_ci_topic, self._on_depth_cam_info, qos)
            self.create_subscription(Image, depth_topic, self._on_depth, qos)
            self._pub_target_point_camera = self.create_publisher(PointStamped, camera_topic, 10)
            self._pub_target_point_map = self.create_publisher(PointStamped, map_topic, 10)
            self._pub_target_label = self.create_publisher(String, label_topic, 10)
            self._tf_buffer_3d = tf2_ros.Buffer()
            self._tf_listener_3d = tf2_ros.TransformListener(self._tf_buffer_3d, self, spin_thread=True)
            self.get_logger().info(
                f"3D目标点: depth={depth_topic}, depth_cam_info={depth_ci_topic} -> "
                f"{camera_topic}, {map_topic} (target_frame={self._target_frame})"
            )
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
            self.create_subscription(CameraInfo, ci_topic, self._on_cam_info, qos_rgb)
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

        self.create_subscription(Image, in_topic, self._on_image, qos_rgb)
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
            return depth.astype(np.float32) / float(divisor)
        return depth.astype(np.float32)

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
        valid = np.isfinite(patch) & (patch > self._depth_min_m) & (patch < self._depth_max_m)
        if not np.any(valid):
            return None
        return float(np.median(patch[valid]))

    def _project_to_map(self, pt_cam: PointStamped) -> PointStamped | None:
        if self._tf_buffer_3d is None or not self._target_frame:
            return None
        # 先按传感器时间戳查 TF；仿真里与 /tf 时间微偏时易失败 -> map_xyz 空、RViz 无球体。再试 Time()=最新变换。
        attempts: tuple[Time, ...] = (Time.from_msg(pt_cam.header.stamp), Time())
        last_err: Exception | None = None
        for when in attempts:
            try:
                trans = self._tf_buffer_3d.lookup_transform(
                    self._target_frame,
                    pt_cam.header.frame_id,
                    when,
                    timeout=Duration(seconds=0, nanoseconds=250_000_000),
                )
                return do_transform_point(pt_cam, trans)
            except Exception as e:
                last_err = e
        if last_err is not None:
            now = time.monotonic()
            if now - self._last_depth_warn_mono > 2.0:
                self._last_depth_warn_mono = now
                self.get_logger().warning(
                    f"3D点TF变换失败 {pt_cam.header.frame_id}->{self._target_frame}: {last_err}"
                )
        return None

    @staticmethod
    def _rgba_for_class_id(cls_id: int) -> ColorRGBA:
        c = ColorRGBA()
        known = {
            0: (0.15, 0.75, 1.0),
            56: (1.0, 0.52, 0.12),
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

    def _publish_track_marker_array(self, now_sec: float) -> None:
        pub = self._pub_track_markers
        if pub is None:
            return
        # 用当前仿真/墙钟时间，避免 Fixed Frame=map 时 RViz 按旧图像 stamp 做 TF 过滤丢 Marker
        mstamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clr_sphere = Marker()
        clr_sphere.header.frame_id = self._target_frame
        clr_sphere.header.stamp = mstamp
        clr_sphere.ns = "yolo_obj_sphere"
        clr_sphere.action = Marker.DELETEALL
        ma.markers.append(clr_sphere)

        clr_text = Marker()
        clr_text.header.frame_id = self._target_frame
        clr_text.header.stamp = mstamp
        clr_text.ns = "yolo_obj_label"
        clr_text.action = Marker.DELETEALL
        ma.markers.append(clr_text)

        for tid, tr in self._tracks.items():
            hits = int(tr.get("hits", 0))
            age = now_sec - float(tr.get("last_seen_sec", now_sec))
            if hits < self._min_track_hits_for_publish or age > self._track_max_age_sec:
                continue
            map_xyz = tr.get("map_xyz")
            if map_xyz is None:
                continue
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

        pub.publish(ma)

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
                "id": tid,
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
        tr["hits"] = int(tr.get("hits", 0)) + 1
        tr["last_seen_sec"] = now_sec

    def _choose_primary_track(self, now_sec: float) -> dict[str, Any] | None:
        best = None
        best_key = None
        for tr in self._tracks.values():
            age = now_sec - float(tr.get("last_seen_sec", now_sec))
            hits = int(tr.get("hits", 0))
            if age > self._track_max_age_sec or hits < self._min_track_hits_for_publish:
                continue
            key = (float(tr.get("conf", 0.0)), hits)
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

    def _publish_target_point_3d(self, image_msg: Image, results: list[Any]) -> None:
        if not self._publish_target_point_3d_enabled:
            return
        if self._depth_cam_info is None or not results:
            return
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return
        depth_msg = self._latest_depth_msg
        if depth_msg is None:
            return
        age = abs(self._stamp_sec(image_msg.header.stamp) - self._stamp_sec(depth_msg.header.stamp))
        if age > self._max_depth_age_sec:
            now = time.monotonic()
            if now - self._last_depth_warn_mono > 2.0:
                self._last_depth_warn_mono = now
                self.get_logger().warning(
                    f"RGB/Depth不同步，跳过3D定位: age={age:.3f}s > {self._max_depth_age_sec:.3f}s"
                )
            return
        depth_m = self._decode_depth_meters(depth_msg)
        if depth_m is None:
            return

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
            ts = image_msg.header.stamp
            now_sec = time.monotonic()
            self._cleanup_tracks(now_sec)
            self._publish_track_debug(now_sec)
            self._publish_track_marker_array(now_sec)
            return
        dh, dw = int(depth_m.shape[0]), int(depth_m.shape[1])
        fx, cx, fy, cy = self._scaled_intrinsics(self._depth_cam_info, dw, dh)
        if fx < 1e-6 or fy < 1e-6:
            return
        names = results[0].names if hasattr(results[0], "names") else {}
        order = np.argsort(-confs.astype(np.float32))
        max_n = max(1, int(self._max_targets_3d_per_frame))
        candidates: list[dict[str, Any]] = []
        for idx in order[:max_n]:
            i = int(idx)
            x1, y1, x2, y2 = [float(v) for v in xyxy[i]]
            u_rgb = float(0.5 * (x1 + x2))
            v_rgb = float(0.5 * (y1 + y2))
            iw = max(1, int(image_msg.width))
            ih = max(1, int(image_msg.height))
            u_d = int(round(u_rgb * float(dw) / float(iw)))
            v_d = int(round(v_rgb * float(dh) / float(ih)))
            u_d = max(0, min(dw - 1, u_d))
            v_d = max(0, min(dh - 1, v_d))
            z = self._sample_depth_from_array(depth_m, u_d, v_d)
            if z is None:
                continue
            x = (float(u_d) - cx) * z / fx
            y = (float(v_d) - cy) * z / fy
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
            return
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
            self._pub_target_label.publish(
                String(
                    data=(
                        f"track_id={int(primary.get('id',-1))};label={primary.get('label','')};"
                        f"conf={float(primary.get('conf',0.0)):.3f};hits={int(primary.get('hits',0))};"
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

    def _on_image(self, msg: Image) -> None:
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
        self._publish_target_point_3d(msg, results)
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
