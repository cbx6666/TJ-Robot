# pyright: reportMissingImports=false
"""订阅相机 RGB，用 Ultralytics YOLO-Seg（默认 COCO、仅 person=0）推理，发布叠加分割/框的图像。

依赖（需 pip，见包内 requirements.txt）：
  pip install -r src/human_yolo_seg/requirements.txt

默认权重文件名 yolo26n-seg.pt：放在包内 models/ 目录后 colcon build，或 model_path 传绝对路径。
"""

from __future__ import annotations

import os
import sys
import time
from typing import Any

import numpy as np

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, Int32

import tf2_ros


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


class YoloPersonSegNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_person_seg_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/human_yolo/annotated_image")
        self.declare_parameter("model_path", "yolo26n-seg.pt")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_ids", [0])
        # auto：有 NVIDIA+CUDA 版 PyTorch 时用 cuda:0，否则 cpu（无需手改代码）
        self.declare_parameter("device", "auto")
        self.declare_parameter("max_inferences_per_sec", 0.0)
        self.declare_parameter("publish_detection_stats", True)
        self.declare_parameter("stats_topic_prefix", "/human_yolo")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("publish_person_azimuths", True)
        self.declare_parameter("person_azimuth_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter("laser_frame_id", "base_scan")
        self.declare_parameter("ground_frame_id", "base_footprint")

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

        ids_msg = self.get_parameter("target_class_ids").get_parameter_value().integer_array_value
        self._classes = list(ids_msg) if ids_msg else None

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
        self._pub = self.create_publisher(Image, out_topic, 10)
        self._pub_stats = bool(
            self.get_parameter("publish_detection_stats").get_parameter_value().bool_value
        )
        sp = self.get_parameter("stats_topic_prefix").get_parameter_value().string_value.rstrip("/")
        self._stats_prefix = sp
        if self._pub_stats:
            self._pub_person_count = self.create_publisher(Int32, f"{sp}/person_count", 10)
            self._pub_person_max_conf = self.create_publisher(Float32, f"{sp}/person_max_conf", 10)
            self._pub_person_present = self.create_publisher(Bool, f"{sp}/person_present", 10)
            self.get_logger().info(
                f"检测统计: {sp}/person_count、person_max_conf、person_present（供建模验收与 ros2 topic echo）"
            )
        else:
            self._pub_person_count = None
            self._pub_person_max_conf = None
            self._pub_person_present = None

        self._pub_azimuth = None
        self._cam_info: CameraInfo | None = None
        self._tf_buffer: tf2_ros.Buffer | None = None
        self._tf_listener = None
        self._laser_frame = ""
        self._ground_frame = ""
        self._last_cam_warn_mono = 0.0
        pub_az = self.get_parameter("publish_person_azimuths").get_parameter_value().bool_value
        if pub_az:
            az_topic = self.get_parameter("person_azimuth_topic").get_parameter_value().string_value
            ci_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
            self._laser_frame = self.get_parameter("laser_frame_id").get_parameter_value().string_value
            self._ground_frame = self.get_parameter("ground_frame_id").get_parameter_value().string_value
            self._pub_azimuth = self.create_publisher(JointState, az_topic, 10)
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
            self.create_subscription(CameraInfo, ci_topic, self._on_cam_info, qos)
            self.get_logger().info(
                f"人物方位角: {az_topic}（JointState，header=RGB 时间戳，position=[lo,hi,...]；CameraInfo: {ci_topic}）"
            )

        self.create_subscription(Image, in_topic, self._on_image, qos)
        self.get_logger().info(
            f"订阅 {in_topic} -> 发布 {out_topic}；类别过滤 {self._classes if self._classes else '全部'}"
        )

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._cam_info = msg

    def _resolve_yolo_device(self) -> str:
        raw = self.get_parameter("device").get_parameter_value().string_value.strip()
        low = raw.lower()
        if low in ("", "auto"):
            try:
                import torch

                if torch.cuda.is_available():
                    name = torch.cuda.get_device_name(0)
                    self.get_logger().info(f"YOLO device=auto -> cuda:0 ({name})")
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
        assert self._pub_person_count is not None
        self._pub_person_count.publish(Int32(data=count))
        self._pub_person_max_conf.publish(Float32(data=max_conf))
        self._pub_person_present.publish(Bool(data=count > 0))

    @staticmethod
    def _person_count_and_max_conf(results: list[Any]) -> tuple[int, float]:
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
            self._publish_person_azimuths(msg, None, 0)
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

        results = self._model.predict(source=img, **kwargs)
        n_p, mx_c = self._person_count_and_max_conf(results)
        self._publish_detection_stats(n_p, mx_c)
        boxes = results[0].boxes if results else None
        self._publish_person_azimuths(msg, boxes, n_p)
        if not results:
            return
        annotated = results[0].plot()
        try:
            out = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except Exception as e:
            self.get_logger().warning(f"编码输出失败: {e}")
            return
        out.header = msg.header
        self._pub.publish(out)

    def _publish_person_azimuths(self, image_msg: Image, boxes: Any, n_person: int) -> None:
        if self._pub_azimuth is None:
            return
        js = JointState()
        js.header = image_msg.header
        js.name = []
        if n_person == 0 or boxes is None or len(boxes) == 0:
            js.position = []
            self._pub_azimuth.publish(js)
            return
        if self._cam_info is None:
            now = time.monotonic()
            if now - self._last_cam_warn_mono > 5.0:
                self._last_cam_warn_mono = now
                self.get_logger().warning("尚未收到 CameraInfo，无法发布 person_azimuth_ranges")
            js.position = []
            self._pub_azimuth.publish(js)
            return
        assert self._tf_buffer is not None
        from human_yolo_seg.person_azimuth import boxes_to_azimuth_data

        data = boxes_to_azimuth_data(
            self._tf_buffer,
            self._cam_info,
            image_msg,
            boxes,
            self._laser_frame,
            self._ground_frame,
            self.get_logger(),
        )
        js.position = [float(x) for x in data]
        self._pub_azimuth.publish(js)


def main() -> None:
    rclpy.init()
    node = YoloPersonSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
