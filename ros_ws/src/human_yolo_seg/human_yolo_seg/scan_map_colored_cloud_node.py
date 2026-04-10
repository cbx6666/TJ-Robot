# pyright: reportMissingImports=false
"""将 /scan 整帧投影到 map，在 RViz 用 PointCloud2 按束着色：人物角域内激光为「人色」，其余为「普通扫描色」（默认红）。

LaserScan 无法逐点设色；本节点等价于「YOLO+RGB 角域整合后再显示」的即时效果。QoS 与激光一致
（Volatile），RViz 里设 Decay Time 可与原 LaserScan 一样拖尾后消失。

人物 map 点仍由 person_strip_recorder 单独累积；本节点只负责显示。"""
from __future__ import annotations

import math
import struct
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray

import tf2_ros

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs import do_transform_point  # type: ignore

from human_yolo_seg.person_scan_sync_utils import (
    angle_in_interval,
    is_zero_stamp,
    merge_intervals,
    norm_angle,
)


def _angle_in_any_sector(theta: float, sectors: List[Tuple[float, float]]) -> bool:
    return any(angle_in_interval(theta, lo, hi) for lo, hi in sectors)


def _pack_rgb_uint32(r: int, g: int, b: int) -> int:
    return ((int(r) & 255) << 16) | ((int(g) & 255) << 8) | (int(b) & 255)


def _xyzrgb_pointcloud2(
    frame_id: str,
    stamp,
    points_rgb: List[Tuple[float, float, float, int]],
) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points_rgb)
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
    for mx, my, mz, rgb in points_rgb:
        buf.extend(struct.pack("<fffI", float(mx), float(my), float(mz), rgb))
    msg.data = bytes(buf)
    return msg


class ScanMapColoredCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_map_colored_cloud")
        try:
            self.declare_parameter("use_sim_time", False)
        except ParameterAlreadyDeclaredException:
            pass

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("azimuth_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter("azimuth_msg_type", "joint_state")
        self.declare_parameter("angle_margin_rad", 0.08)
        self.declare_parameter("hold_seconds", 0.45)
        self.declare_parameter("sync_max_scan_image_delay_sec", 0.5)
        self.declare_parameter("skip_time_sync_if_zero_stamp", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_cloud_topic", "/human_yolo/scan_map_colored_cloud")
        self.declare_parameter("z_in_map_m", 0.02)
        self.declare_parameter("color_scan_b", 255)
        self.declare_parameter("color_scan_g", 0)
        self.declare_parameter("color_scan_r", 0)
        self.declare_parameter("color_person_b", 0)
        self.declare_parameter("color_person_g", 255)
        self.declare_parameter("color_person_r", 255)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        az_topic = self.get_parameter("azimuth_topic").get_parameter_value().string_value
        az_type = (
            self.get_parameter("azimuth_msg_type").get_parameter_value().string_value.strip().lower()
        )
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
        out_topic = self.get_parameter("output_cloud_topic").get_parameter_value().string_value
        self._z_map = float(self.get_parameter("z_in_map_m").get_parameter_value().double_value)

        def _tri(prefix: str) -> Tuple[int, int, int]:
            b = int(self.get_parameter(f"color_{prefix}_b").get_parameter_value().integer_value)
            g = int(self.get_parameter(f"color_{prefix}_g").get_parameter_value().integer_value)
            r = int(self.get_parameter(f"color_{prefix}_r").get_parameter_value().integer_value)
            return max(0, min(255, b)), max(0, min(255, g)), max(0, min(255, r))

        sb, sg, sr = _tri("scan")
        pb, pg, pr = _tri("person")
        self._rgb_scan = _pack_rgb_uint32(sr, sg, sb)
        self._rgb_person = _pack_rgb_uint32(pr, pg, pb)

        self._pairs: List[Tuple[float, float]] = []
        self._pairs_image_time: Optional[Time] = None
        self._azimuth_recv_time: Optional[Time] = None
        self._last_float_mono = 0.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self._pub = self.create_publisher(PointCloud2, out_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)
        if az_type in ("float64", "float64_multiarray", "multiarray"):
            self.create_subscription(Float64MultiArray, az_topic, self._on_azimuth_array, 10)
            self.get_logger().info(
                f"scan→map 彩色点云: {scan_topic} + Float64MultiArray {az_topic} -> {out_topic}"
            )
        else:
            self.create_subscription(JointState, az_topic, self._on_azimuth_joint, 10)
            self.get_logger().info(
                f"scan→map 彩色点云: {scan_topic} + JointState {az_topic} -> {out_topic} "
                f"（RViz: PointCloud2 RGB8，Decay 与 LaserScan 类似可调）"
            )

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
        out: List[Tuple[float, float]] = []
        for lo, hi in self._pairs:
            out.append((norm_angle(lo - self._margin), norm_angle(hi + self._margin)))
        return merge_intervals(out)

    def _on_scan(self, msg: LaserScan) -> None:
        active = self._active_pairs(msg)
        laser_frame = msg.header.frame_id or "base_scan"
        stamp = msg.header.stamp
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

        ang = float(msg.angle_min)
        inc = float(msg.angle_increment)
        rmin = float(msg.range_min)
        rmax = float(msg.range_max)

        pts: List[Tuple[float, float, float, int]] = []
        for sample in msg.ranges:
            a = norm_angle(ang)
            v = float(sample)
            ang += inc
            if not math.isfinite(v) or not (rmin <= v <= rmax):
                continue
            is_person = bool(active) and _angle_in_any_sector(a, active)
            rgb = self._rgb_person if is_person else self._rgb_scan

            pt = PointStamped()
            pt.header.frame_id = laser_frame
            pt.header.stamp = stamp
            pt.point.x = float(v * math.cos(a))
            pt.point.y = float(v * math.sin(a))
            pt.point.z = 0.0
            try:
                p_map = do_transform_point(pt, trans)
                pts.append((float(p_map.point.x), float(p_map.point.y), self._z_map, rgb))
            except Exception:
                continue

        if not pts:
            return
        cloud = _xyzrgb_pointcloud2(self._map_frame, stamp, pts)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = ScanMapColoredCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
