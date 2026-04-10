# pyright: reportMissingImports=false
"""将 /scan 中落在人物方位角区间内的读数替换为无效值后发布 /scan_filtered。

默认可用 **inf**（与 Gazebo 激光「无回波」一致），部分后端对 **nan** 处理差会导致 slam_toolbox 建图异常；见参数 **masked_range_mode**。

订阅：
  - sensor_msgs/LaserScan  scan_in
  - 角域话题（二选一，见参数 azimuth_msg_type）：
      joint_state：sensor_msgs/JointState，header 为 **与检测对应的 RGB 图像时间戳**，
                  position 为 [lo0,hi0,lo1,hi1,...]（弧度）；
      float64_multiarray：std_msgs/Float64MultiArray（无时间戳，不推荐；不做 scan–图像同步）

时间同步（joint_state 且 sync_max_scan_image_delay_sec>0）：
  仅当 |t_scan - t_image| <= 阈值时才掩膜，避免把「上一帧人」的角度套到「当前激光」上。
  hold_seconds 使用节点时钟（仿真下为 /clock），与 use_sim_time 一致。
"""
from __future__ import annotations

import math
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray

import tf2_ros
from geometry_msgs.msg import PointStamped

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

# slam_toolbox 等对 /scan 的订阅常为 Reliable；输出与 Gazebo 的 Best Effort 一致易导致无订阅者
_QOS_SCAN_OUT = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class ScanPersonFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_person_filter")

        self.declare_parameter("scan_in", "/scan")
        self.declare_parameter("scan_out", "/scan_filtered")
        self.declare_parameter("azimuth_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter(
            "azimuth_msg_type",
            "joint_state",
        )
        self.declare_parameter("angle_margin_rad", 0.08)
        self.declare_parameter("hold_seconds", 0.45)
        self.declare_parameter(
            "sync_max_scan_image_delay_sec",
            0.5,
        )
        self.declare_parameter("skip_time_sync_if_zero_stamp", True)
        # nan：几何上表示「忽略」但部分 SLAM 实现不兼容；inf：与 TB3 Gazebo 无回波一致，利于 slam_toolbox；range_max：等价于量程末端
        self.declare_parameter("masked_range_mode", "inf")
        # 仅把“相机最大水平视角”覆盖到的激光扇区喂给 SLAM（其余方向统一填充为 masked_range_mode）
        # 目的：当 YOLO 只能覆盖前向视角时，避免把背后/侧后的人形障碍建进图里。
        # 注意：这会降低建图覆盖率（只用一个扇区），但能显著减少“背后的人被画进地图”。
        self.declare_parameter("limit_scan_to_fov", False)
        # 以激光坐标系为基准的扇区范围（度）。例如 -70..70 代表只保留前向约 140°。
        self.declare_parameter("fov_min_deg", -70.0)
        self.declare_parameter("fov_max_deg", 70.0)

        # 人位置记忆 keepout：用于解决 YOLO 盲区/瞬时丢检时，人形仍被激光建进图的问题。
        # 逻辑：当有 active_pairs 时，在对应角域内用激光取最近点，变换到 map 后缓存一段时间；
        # 后续即使无 active_pairs，也对缓存点位对应角域继续掩膜（/scan_filtered 仅供 SLAM）。
        self.declare_parameter("enable_person_keepout_memory", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("person_keepout_memory_sec", 8.0)
        self.declare_parameter("person_keepout_radius_m", 1.1)

        scan_in = self.get_parameter("scan_in").get_parameter_value().string_value
        scan_out = self.get_parameter("scan_out").get_parameter_value().string_value
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
        _mr = self.get_parameter("masked_range_mode").get_parameter_value().string_value.strip().lower()
        if _mr in ("range_max", "max"):
            self._masked_fill = "range_max"
        elif _mr == "nan":
            self._masked_fill = "nan"
        else:
            self._masked_fill = "inf"
        self.get_logger().info(f"掩膜读数填充: {self._masked_fill}（masked_range_mode）")

        self._limit_fov = bool(self.get_parameter("limit_scan_to_fov").get_parameter_value().bool_value)
        self._fov_min_deg = float(self.get_parameter("fov_min_deg").get_parameter_value().double_value)
        self._fov_max_deg = float(self.get_parameter("fov_max_deg").get_parameter_value().double_value)
        if self._limit_fov:
            self.get_logger().warning(
                f"limit_scan_to_fov=TRUE：只保留激光扇区 [{self._fov_min_deg:.1f}°, {self._fov_max_deg:.1f}°] 用于 /scan_filtered（其余方向将被掩膜）"
            )

        self._enable_keepout = bool(
            self.get_parameter("enable_person_keepout_memory").get_parameter_value().bool_value
        )
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._keepout_sec = float(
            self.get_parameter("person_keepout_memory_sec").get_parameter_value().double_value
        )
        self._keepout_radius = float(
            self.get_parameter("person_keepout_radius_m").get_parameter_value().double_value
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)
        self._keepout_points_map: List[Tuple[float, float, int]] = []  # (x,y,expiry_ns)

        self._pub = self.create_publisher(LaserScan, scan_out, _QOS_SCAN_OUT)
        self.create_subscription(LaserScan, scan_in, self._on_scan, qos_profile_sensor_data)
        if az_type in ("float64", "float64_multiarray", "multiarray"):
            self.create_subscription(Float64MultiArray, az_topic, self._on_azimuth_array, 10)
            self.get_logger().info(
                f"{scan_in} -> {scan_out}；角域 {az_topic} 为 Float64MultiArray（无图像时间戳，不做 scan 同步）"
            )
        else:
            self.create_subscription(JointState, az_topic, self._on_azimuth_joint, 10)
            self.get_logger().info(
                f"{scan_in} -> {scan_out}；角域 {az_topic} 为 JointState（RGB header.stamp + scan 时差≤{self._sync_max}s）"
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
        if sec > self._sync_max:
            return False
        return True

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

    def _expire_keepout(self, now_ns: int) -> None:
        if not self._keepout_points_map:
            return
        self._keepout_points_map = [p for p in self._keepout_points_map if p[2] > now_ns]

    def _learn_keepout_from_pairs(self, scan_msg: LaserScan, active: List[Tuple[float, float]]) -> None:
        if not self._enable_keepout or not active or self._keepout_sec <= 0.0:
            return
        now_ns = self.get_clock().now().nanoseconds
        expiry = now_ns + int(self._keepout_sec * 1e9)
        laser_frame = scan_msg.header.frame_id or "base_scan"

        # 对每个角域，扫描激光找到最近的有效点，然后变换到 map 缓存
        for lo, hi in active:
            best_r = float("inf")
            best_ang: Optional[float] = None
            ang = float(scan_msg.angle_min)
            for sample in scan_msg.ranges:
                a = norm_angle(ang)
                v = float(sample)
                if angle_in_interval(a, lo, hi) and math.isfinite(v):
                    if scan_msg.range_min <= v <= scan_msg.range_max:
                        if v < best_r:
                            best_r = v
                            best_ang = a
                ang += float(scan_msg.angle_increment)
            if best_ang is None or not math.isfinite(best_r):
                continue

            # 激光系点
            px = best_r * math.cos(best_ang)
            py = best_r * math.sin(best_ang)
            pt = PointStamped()
            pt.header.frame_id = laser_frame
            pt.header.stamp = scan_msg.header.stamp
            pt.point.x = float(px)
            pt.point.y = float(py)
            pt.point.z = 0.0

            try:
                t = Time.from_msg(scan_msg.header.stamp)
                trans = self._tf_buffer.lookup_transform(self._map_frame, laser_frame, t, timeout=Duration(seconds=0.2))
                p_map = do_transform_point(pt, trans)
                self._keepout_points_map.append(
                    (float(p_map.point.x), float(p_map.point.y), expiry)
                )
            except Exception:
                # TF 不可用时不学习 keepout
                continue

    def _apply_keepout_memory_mask(self, scan_msg: LaserScan, ranges: List[float]) -> None:
        if not self._enable_keepout or not self._keepout_points_map:
            return
        now_ns = self.get_clock().now().nanoseconds
        self._expire_keepout(now_ns)
        if not self._keepout_points_map:
            return

        laser_frame = scan_msg.header.frame_id or "base_scan"
        # map->laser 变换，用于把缓存点位投到当前激光扇区
        try:
            t = Time.from_msg(scan_msg.header.stamp)
            trans = self._tf_buffer.lookup_transform(laser_frame, self._map_frame, t, timeout=Duration(seconds=0.2))
        except Exception:
            try:
                trans = self._tf_buffer.lookup_transform(laser_frame, self._map_frame, Time(), timeout=Duration(seconds=0.2))
            except Exception:
                return

        # 对每个缓存点，算出在激光系的 bearing 和角宽，然后掩膜
        for mx, my, _expiry in self._keepout_points_map:
            pt = PointStamped()
            pt.header.frame_id = self._map_frame
            pt.header.stamp = scan_msg.header.stamp
            pt.point.x = float(mx)
            pt.point.y = float(my)
            pt.point.z = 0.0
            try:
                p_l = do_transform_point(pt, trans)
            except Exception:
                continue
            x = float(p_l.point.x)
            y = float(p_l.point.y)
            dist = math.hypot(x, y)
            if dist < 1e-3:
                continue
            bearing = math.atan2(y, x)
            half = math.atan2(max(self._keepout_radius, 0.05), dist)
            lo = norm_angle(bearing - half - self._margin)
            hi = norm_angle(bearing + half + self._margin)

            for i in range(len(ranges)):
                ang = scan_msg.angle_min + float(i) * scan_msg.angle_increment
                if angle_in_interval(ang, lo, hi):
                    if self._masked_fill == "nan":
                        ranges[i] = float("nan")
                    elif self._masked_fill == "range_max":
                        ranges[i] = float(scan_msg.range_max)
                    else:
                        ranges[i] = float("inf")

    def _on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = list(msg.ranges)
        # 先做 FOV 限制（视角外统一掩膜），再做“人物角域掩膜”
        if self._limit_fov:
            lo = norm_angle(math.radians(self._fov_min_deg))
            hi = norm_angle(math.radians(self._fov_max_deg))
            for i in range(len(ranges)):
                ang = msg.angle_min + float(i) * msg.angle_increment
                if not angle_in_interval(ang, lo, hi):
                    if self._masked_fill == "nan":
                        ranges[i] = float("nan")
                    elif self._masked_fill == "range_max":
                        ranges[i] = float(msg.range_max)
                    else:
                        ranges[i] = float("inf")

        active = self._active_pairs(msg)
        # 学习 keepout（只在 active_pairs 可靠时更新）
        self._learn_keepout_from_pairs(msg, active)
        if active:
            for i in range(len(ranges)):
                ang = msg.angle_min + float(i) * msg.angle_increment
                if any(angle_in_interval(ang, lo, hi) for lo, hi in active):
                    if self._masked_fill == "nan":
                        ranges[i] = float("nan")
                    elif self._masked_fill == "range_max":
                        ranges[i] = float(msg.range_max)
                    else:
                        ranges[i] = float("inf")

        # keepout 记忆掩膜（即使 YOLO 当前无输出）
        self._apply_keepout_memory_mask(msg, ranges)
        out.ranges = ranges
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanPersonFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
