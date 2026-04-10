# pyright: reportMissingImports=false
"""建图阶段：把「YOLO 人物方位角」与 /scan 对齐后，对角域内每一束有效激光做 map 融合并打标记。

不修改 /scan（不剔除点），仅在 map 系累积标记点云 + YAML 清障圆心；与 scan_person_filter 的
时间同步 / hold 逻辑一致。建图用原始 /scan 见 mapper_params_online_async_full_scan.yaml。

PointCloud2（/human_yolo/person_laser_map_cloud，frame=map）：带 rgb 字段，默认亮黄，与 /map 黑格、/scan 红区
分开；需在 RViz 勾选该显示且 Color Transformer=RGB8（见 test1.rviz）。Transient Local。_cells 只增不减。
建图结束后 strip_saved_map_person_free
按 YAML 圆擦除对应栅格。若要在**不擦除**的前提下肉眼看人物与 SLAM 对齐，用
annotate_saved_map_person_overlay 生成 *_person_overlay.png（洋红圆 + 黄十字）。
未起本节点时可用 snapshot_person_regions_from_cloud 从 /human_yolo/person_laser_map_cloud 拉一帧生成同格式 YAML（需该话题曾有人发布、Transient Local 有缓存）。

输出文件示例见 strip_saved_map_person_free --help。
"""
from __future__ import annotations

import math
import os
import struct
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray

import tf2_ros
from geometry_msgs.msg import PointStamped

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs import do_transform_point  # type: ignore

import yaml

from human_yolo_seg.person_scan_sync_utils import (
    angle_in_interval,
    is_zero_stamp,
    merge_intervals,
    norm_angle,
)

_QOS_LASER_MAP_CLOUD = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _angle_in_any_sector(theta: float, sectors: List[Tuple[float, float]]) -> bool:
    return any(angle_in_interval(theta, lo, hi) for lo, hi in sectors)


def _pack_rgb_uint32(r: int, g: int, b: int) -> int:
    return ((int(r) & 255) << 16) | ((int(g) & 255) << 8) | (int(b) & 255)


def _xyzrgb_pointcloud2(
    frame_id: str,
    stamp,
    xy_points: List[Tuple[float, float]],
    z: float,
    rgb_packed: int,
) -> PointCloud2:
    """x,y,z float32 + rgb uint32（PCL 常用打包），供 RViz PointCloud2 的 RGB8 着色。"""
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(xy_points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = msg.point_step * max(msg.width, 1)
    msg.is_dense = True
    buf = bytearray()
    for mx, my in xy_points:
        buf.extend(struct.pack("<fffI", float(mx), float(my), float(z), rgb_packed))
    msg.data = bytes(buf)
    return msg


class PersonStripRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("person_strip_recorder")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("azimuth_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter("azimuth_msg_type", "joint_state")
        self.declare_parameter("angle_margin_rad", 0.08)
        self.declare_parameter("hold_seconds", 0.45)
        # 仿真下激光常 5Hz（周期间隔可达 0.2s），再叠加 YOLO 推理与调度，0.15s 过严会导致角域与 /scan 永远对不齐、_cells 始终为空
        self.declare_parameter("sync_max_scan_image_delay_sec", 0.5)
        self.declare_parameter("skip_time_sync_if_zero_stamp", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter(
            "output_path",
            str(Path.home() / ".ros" / "tj_person_strip_regions.yaml"),
        )
        # 与常见 SLAM 栅格 0.05m 同量级：清图时小圆擦除激光打到的人
        self.declare_parameter("strip_radius_m", 0.08)
        self.declare_parameter("dedupe_grid_m", 0.05)
        self.declare_parameter("autosave_interval_sec", 20.0)
        self.declare_parameter("publish_laser_map_cloud", True)
        self.declare_parameter("laser_map_cloud_topic", "/human_yolo/person_laser_map_cloud")
        self.declare_parameter("laser_map_cloud_z_m", 0.05)
        self.declare_parameter("laser_map_cloud_publish_hz", 5.0)
        self.declare_parameter("laser_map_cloud_color_r", 255)
        self.declare_parameter("laser_map_cloud_color_g", 255)
        self.declare_parameter("laser_map_cloud_color_b", 0)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        az_topic = self.get_parameter("azimuth_topic").get_parameter_value().string_value
        az_type = (
            self.get_parameter("azimuth_msg_type").get_parameter_value().string_value.strip().lower()
        )

        self._pairs: List[Tuple[float, float]] = []
        self._pairs_image_time: Optional[Time] = None
        self._azimuth_recv_time: Optional[Time] = None
        self._last_float_mono = 0.0

        self._hold = float(self.get_parameter("hold_seconds").get_parameter_value().double_value)
        self._margin = float(self.get_parameter("angle_margin_rad").get_parameter_value().double_value)
        self._sync_max = float(
            self.get_parameter("sync_max_scan_image_delay_sec").get_parameter_value().double_value
        )
        self._skip_sync_zero = bool(
            self.get_parameter("skip_time_sync_if_zero_stamp").get_parameter_value().bool_value
        )
        self._az_type = az_type
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        out = self.get_parameter("output_path").get_parameter_value().string_value
        self._output_path = os.path.expanduser(out)
        self._strip_r = max(float(self.get_parameter("strip_radius_m").get_parameter_value().double_value), 0.02)
        self._dedupe = max(float(self.get_parameter("dedupe_grid_m").get_parameter_value().double_value), 0.02)
        self._autosave = max(float(self.get_parameter("autosave_interval_sec").get_parameter_value().double_value), 1.0)

        pub_cloud = bool(self.get_parameter("publish_laser_map_cloud").get_parameter_value().bool_value)
        self._pub_laser_cloud = None
        self._cloud_z = float(self.get_parameter("laser_map_cloud_z_m").get_parameter_value().double_value)
        self._cloud_rgb = _pack_rgb_uint32(
            int(self.get_parameter("laser_map_cloud_color_r").get_parameter_value().integer_value),
            int(self.get_parameter("laser_map_cloud_color_g").get_parameter_value().integer_value),
            int(self.get_parameter("laser_map_cloud_color_b").get_parameter_value().integer_value),
        )
        hz = max(float(self.get_parameter("laser_map_cloud_publish_hz").get_parameter_value().double_value), 0.5)
        if pub_cloud:
            ct = self.get_parameter("laser_map_cloud_topic").get_parameter_value().string_value
            self._pub_laser_cloud = self.create_publisher(PointCloud2, ct, _QOS_LASER_MAP_CLOUD)
            self.get_logger().info(
                f"人物角域激光→map 标记云: {ct}（角域内全部有效点融合；Transient Local）"
            )
            self.create_timer(1.0 / hz, self._publish_laser_map_cloud)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        # map 系去重栅格 -> 代表点；角域内多束激光融合后写入，只增不减
        self._cells: Dict[Tuple[int, int], Tuple[float, float]] = {}

        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        if az_type in ("float64", "float64_multiarray", "multiarray"):
            self.create_subscription(Float64MultiArray, az_topic, self._on_azimuth_array, 10)
            self.get_logger().info(f"记录人物 map 点：{scan_topic} + Float64MultiArray {az_topic}")
        else:
            self.create_subscription(JointState, az_topic, self._on_azimuth_joint, 10)
            self.get_logger().info(
                f"记录人物 map 点：{scan_topic} + JointState {az_topic}（scan–图像时差≤{self._sync_max}s）"
            )
        self.get_logger().info(
            f"输出 {self._output_path}；strip 半径={self._strip_r}m；autosave={self._autosave}s"
        )
        self.create_timer(self._autosave, self._autosave_cb)

    def _set_pairs_from_positions(
        self, positions: List[float], image_stamp, recv_mono: Optional[float] = None
    ) -> None:
        pairs: List[Tuple[float, float]] = []
        for i in range(0, len(positions) - 1, 2):
            pairs.append((float(positions[i]), float(positions[i + 1])))
        self._pairs = merge_intervals(pairs)
        if image_stamp is not None and not is_zero_stamp(image_stamp):
            self._pairs_image_time = Time.from_msg(image_stamp)
        else:
            self._pairs_image_time = None
        self._azimuth_recv_time = self.get_clock().now()
        if recv_mono is not None:
            self._last_float_mono = recv_mono

    def _on_azimuth_joint(self, msg: JointState) -> None:
        self._set_pairs_from_positions(list(msg.position), msg.header.stamp)

    def _on_azimuth_array(self, msg: Float64MultiArray) -> None:
        self._set_pairs_from_positions([float(x) for x in msg.data], None, time.monotonic())

    def _hold_expired(self) -> bool:
        if self._az_type in ("float64", "float64_multiarray", "multiarray"):
            return (time.monotonic() - self._last_float_mono) > self._hold
        if self._azimuth_recv_time is None:
            return True
        return (self.get_clock().now() - self._azimuth_recv_time) > Duration(seconds=self._hold)

    def _stamp_sync_allows_mask(self, scan_msg: LaserScan) -> bool:
        if self._sync_max <= 0.0:
            return True
        if self._az_type in ("float64", "float64_multiarray", "multiarray"):
            return True
        if self._pairs_image_time is None:
            return True
        if is_zero_stamp(scan_msg.header.stamp) and self._skip_sync_zero:
            return True
        if is_zero_stamp(scan_msg.header.stamp) and not self._skip_sync_zero:
            return False
        t_scan = Time.from_msg(scan_msg.header.stamp)
        dt = t_scan - self._pairs_image_time
        sec = abs(dt.nanoseconds) / 1e9
        return sec <= self._sync_max

    def _active_pairs(self, scan_msg: LaserScan) -> List[Tuple[float, float]]:
        if self._hold_expired():
            return []
        if not self._stamp_sync_allows_mask(scan_msg):
            return []
        out = []
        for lo, hi in self._pairs:
            out.append(
                (
                    norm_angle(lo - self._margin),
                    norm_angle(hi + self._margin),
                )
            )
        return merge_intervals(out)

    def _fuse_person_sector_scan_to_map(
        self, scan_msg: LaserScan, active: List[Tuple[float, float]]
    ) -> None:
        """人物角域内每一道有效激光点变换到 map，写入去重栅格（不删原 /scan）。"""
        laser_frame = scan_msg.header.frame_id or "base_scan"
        stamp = scan_msg.header.stamp
        t = Time.from_msg(stamp)
        try:
            trans = self._tf_buffer.lookup_transform(
                self._map_frame, laser_frame, t, timeout=Duration(seconds=0.2)
            )
        except Exception:
            try:
                trans = self._tf_buffer.lookup_transform(
                    self._map_frame, laser_frame, Time(), timeout=Duration(seconds=0.2)
                )
            except Exception:
                return

        ang = float(scan_msg.angle_min)
        inc = float(scan_msg.angle_increment)
        rmin = float(scan_msg.range_min)
        rmax = float(scan_msg.range_max)

        for sample in scan_msg.ranges:
            a = norm_angle(ang)
            v = float(sample)
            ang += inc
            if not math.isfinite(v) or not (rmin <= v <= rmax):
                continue
            if not _angle_in_any_sector(a, active):
                continue

            pt = PointStamped()
            pt.header.frame_id = laser_frame
            pt.header.stamp = stamp
            pt.point.x = float(v * math.cos(a))
            pt.point.y = float(v * math.sin(a))
            pt.point.z = 0.0
            try:
                p_map = do_transform_point(pt, trans)
                mx = float(p_map.point.x)
                my = float(p_map.point.y)
            except Exception:
                continue
            gx = int(round(mx / self._dedupe))
            gy = int(round(my / self._dedupe))
            self._cells[(gx, gy)] = (mx, my)

    def _publish_laser_map_cloud(self) -> None:
        if self._pub_laser_cloud is None:
            return
        now = self.get_clock().now().to_msg()
        pts = sorted(self._cells.values(), key=lambda p: (p[0], p[1]))
        self._pub_laser_cloud.publish(
            _xyzrgb_pointcloud2(self._map_frame, now, pts, self._cloud_z, self._cloud_rgb)
        )

    def _on_scan(self, msg: LaserScan) -> None:
        active = self._active_pairs(msg)
        if active:
            self._fuse_person_sector_scan_to_map(msg, active)

    def _regions_payload(self) -> dict:
        regions = [{"x": float(x), "y": float(y), "radius": float(self._strip_r)} for x, y in self._cells.values()]
        regions.sort(key=lambda r: (r["x"], r["y"]))
        return {
            "map_frame": self._map_frame,
            "regions": regions,
        }

    def _write_file(self) -> None:
        path = Path(self._output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            f.write(
                "# TJ-Robot：人物在 map 系下的清障圆（米）。"
                "建图保存后执行: ros2 run human_yolo_seg strip_saved_map_person_free ...\n"
            )
            yaml.safe_dump(self._regions_payload(), f, allow_unicode=True, default_flow_style=False, sort_keys=False)

    def _autosave_cb(self) -> None:
        if self._cells:
            try:
                self._write_file()
            except OSError as e:
                self.get_logger().warning(f"autosave 失败: {e}")

    def destroy_node(self) -> bool:
        if self._cells:
            try:
                self._write_file()
                self.get_logger().info(f"已写入 {len(self._cells)} 个去重区域 -> {self._output_path}")
            except OSError as e:
                self.get_logger().error(f"退出时保存失败: {e}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PersonStripRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
