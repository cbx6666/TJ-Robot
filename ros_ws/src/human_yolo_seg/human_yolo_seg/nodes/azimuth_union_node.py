# pyright: reportMissingImports=false
"""合并多路 YOLO 的人物方位角区间（JointState.position=[lo,hi,...]）为一路输出。

用途：前/后（或多路）相机 + 多个 yolo_person_seg_node -> 合并角域，让 scan_person_filter 仍只订阅一个话题。
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState


def _norm_angle(a: float) -> float:
    while a <= -math.pi:
        a += 2.0 * math.pi
    while a > math.pi:
        a -= 2.0 * math.pi
    return a


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


def _pairs_from_joint_state(msg: JointState, margin_rad: float) -> List[Tuple[float, float]]:
    p = list(msg.position)
    pairs: List[Tuple[float, float]] = []
    for i in range(0, len(p) - 1, 2):
        lo = _norm_angle(float(p[i]) - margin_rad)
        hi = _norm_angle(float(p[i + 1]) + margin_rad)
        pairs.append((lo, hi))
    return pairs


class AzimuthUnionNode(Node):
    def __init__(self) -> None:
        super().__init__("azimuth_union")

        self.declare_parameter("in_topics", ["/human_yolo_front/person_azimuth_ranges", "/human_yolo_rear/person_azimuth_ranges"])
        self.declare_parameter("out_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter("hold_seconds", 0.7)
        self.declare_parameter("angle_margin_rad", 0.06)
        self.declare_parameter("publish_rate_hz", 15.0)

        in_topics = list(self.get_parameter("in_topics").get_parameter_value().string_array_value)
        self._out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        self._hold = float(self.get_parameter("hold_seconds").get_parameter_value().double_value)
        self._margin = float(self.get_parameter("angle_margin_rad").get_parameter_value().double_value)
        hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)

        self._last_msgs: dict[str, JointState] = {}
        self._last_recv: dict[str, Time] = {}

        qos = QoSProfile(depth=10)
        for t in in_topics:
            self.create_subscription(JointState, t, lambda m, tt=t: self._on_in(tt, m), qos)
        self._pub = self.create_publisher(JointState, self._out_topic, qos)
        self.create_timer(1.0 / max(hz, 1.0), self._tick)

        self.get_logger().info(f"azimuth_union: {in_topics} -> {self._out_topic} (hold={self._hold}s)")

    def _on_in(self, topic: str, msg: JointState) -> None:
        self._last_msgs[topic] = msg
        self._last_recv[topic] = self.get_clock().now()

    def _tick(self) -> None:
        now = self.get_clock().now()
        pairs: List[Tuple[float, float]] = []
        best_stamp: Optional[Time] = None

        for topic, msg in list(self._last_msgs.items()):
            t_recv = self._last_recv.get(topic)
            if t_recv is None:
                continue
            if (now - t_recv) > Duration(seconds=self._hold):
                continue
            pairs.extend(_pairs_from_joint_state(msg, self._margin))
            try:
                t_img = Time.from_msg(msg.header.stamp)
                if best_stamp is None or t_img.nanoseconds > best_stamp.nanoseconds:
                    best_stamp = t_img
            except Exception:
                pass

        out = JointState()
        if best_stamp is not None:
            out.header.stamp = best_stamp.to_msg()
        out.name = []
        merged = _merge_intervals(pairs)
        flat: List[float] = []
        for lo, hi in merged:
            flat.extend([float(lo), float(hi)])
        out.position = flat
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = AzimuthUnionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

