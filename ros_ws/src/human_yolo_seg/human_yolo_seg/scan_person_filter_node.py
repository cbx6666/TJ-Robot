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

# slam_toolbox 等对 /scan 的订阅常为 Reliable；输出与 Gazebo 的 Best Effort 一致易导致无订阅者
_QOS_SCAN_OUT = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


def _norm_angle(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


def _angle_in_interval(theta: float, lo: float, hi: float) -> bool:
    t = _norm_angle(theta)
    lo = _norm_angle(lo)
    hi = _norm_angle(hi)
    if lo <= hi:
        return lo <= t <= hi
    return t >= lo or t <= hi


def _merge_intervals(pairs: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not pairs:
        return []
    flat = sorted((min(a, b), max(a, b)) for a, b in pairs)
    out: List[Tuple[float, float]] = []
    for lo, hi in flat:
        if hi - lo > math.pi * 0.95:
            continue
        if not out or lo > out[-1][1]:
            out.append((lo, hi))
        else:
            prev_lo, prev_hi = out[-1]
            out[-1] = (prev_lo, max(prev_hi, hi))
    return out


def _is_zero_stamp(stamp) -> bool:
    return int(stamp.sec) == 0 and int(stamp.nanosec) == 0


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
            0.15,
        )
        self.declare_parameter("skip_time_sync_if_zero_stamp", True)
        # nan：几何上表示「忽略」但部分 SLAM 实现不兼容；inf：与 TB3 Gazebo 无回波一致，利于 slam_toolbox；range_max：等价于量程末端
        self.declare_parameter("masked_range_mode", "inf")

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
        self._pairs = _merge_intervals(pairs)
        if image_stamp is not None and not _is_zero_stamp(image_stamp):
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
        if _is_zero_stamp(scan_msg.header.stamp) and self._skip_sync_zero:
            return True
        if _is_zero_stamp(scan_msg.header.stamp) and not self._skip_sync_zero:
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
                    _norm_angle(lo - self._margin),
                    _norm_angle(hi + self._margin),
                )
            )
        return _merge_intervals(out)

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
        active = self._active_pairs(msg)
        if active:
            for i in range(len(ranges)):
                ang = msg.angle_min + float(i) * msg.angle_increment
                if any(_angle_in_interval(ang, lo, hi) for lo, hi in active):
                    if self._masked_fill == "nan":
                        ranges[i] = float("nan")
                    elif self._masked_fill == "range_max":
                        ranges[i] = float(msg.range_max)
                    else:
                        ranges[i] = float("inf")
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
