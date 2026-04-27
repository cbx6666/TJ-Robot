# pyright: reportMissingImports=false
"""订阅人物方位角区间，在 laser 系（默认 base_scan）发布圆弧 Marker：半径 R 上 [lo,hi] 一段 LINE_STRIP。

与 /scan 一致：x 前、y 左，atan2(y,x) 左侧为正。无填充、无文字。
若 /human_yolo/person_azimuth_markers 的 Publisher>1，说明有多个本节点残留进程，需停干净再启。"""
from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import Point
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from human_yolo_seg.utils.person_scan_sync_utils import norm_angle

_NS = 'person_azimuth_arc'
_MAX_SECTORS = 16


def _arc_thetas(lo: float, hi: float, n: int) -> List[float]:
    lo = norm_angle(lo)
    hi = norm_angle(hi)
    n = max(8, n)
    if lo <= hi:
        return [lo + (hi - lo) * i / (n - 1) for i in range(n)]
    n1 = max(4, n // 2)
    n2 = max(4, n - n1)
    left = [lo + (math.pi - lo) * i / max(1, n1 - 1) for i in range(n1)]
    right = [-math.pi + (hi - (-math.pi)) * i / max(1, n2 - 1) for i in range(n2)]
    return [float(norm_angle(t)) for t in left + right]


class PersonAzimuthMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__('person_azimuth_markers')
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('azimuth_topic', '/human_yolo/person_azimuth_ranges')
        self.declare_parameter('azimuth_msg_type', 'joint_state')
        self.declare_parameter('marker_topic', '/human_yolo/person_azimuth_markers')
        self.declare_parameter('laser_frame_id', 'base_scan')
        self.declare_parameter('arc_radius_m', 1.0)
        self.declare_parameter('arc_z_m', 0.10)
        self.declare_parameter('arc_points', 48)
        self.declare_parameter('line_width_m', 0.085)
        self.declare_parameter('color_r', 0.15)
        self.declare_parameter('color_g', 1.0)
        self.declare_parameter('color_b', 0.35)
        self.declare_parameter('color_a', 0.95)

        self._frame = self.get_parameter('laser_frame_id').get_parameter_value().string_value or 'base_scan'
        self._r = max(float(self.get_parameter('arc_radius_m').get_parameter_value().double_value), 0.15)
        self._z = float(self.get_parameter('arc_z_m').get_parameter_value().double_value)
        self._npt = max(8, int(self.get_parameter('arc_points').get_parameter_value().integer_value))
        self._lw = max(0.005, float(self.get_parameter('line_width_m').get_parameter_value().double_value))
        cr = float(self.get_parameter('color_r').get_parameter_value().double_value)
        cg = float(self.get_parameter('color_g').get_parameter_value().double_value)
        cb = float(self.get_parameter('color_b').get_parameter_value().double_value)
        ca = float(self.get_parameter('color_a').get_parameter_value().double_value)
        self._color = ColorRGBA(r=cr, g=cg, b=cb, a=max(0.05, min(1.0, ca)))

        mt = self.get_parameter('marker_topic').get_parameter_value().string_value
        _mqos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = self.create_publisher(MarkerArray, mt, _mqos)

        az_topic = self.get_parameter('azimuth_topic').get_parameter_value().string_value
        az_type = self.get_parameter('azimuth_msg_type').get_parameter_value().string_value.strip().lower()
        if az_type in ('float64', 'float64_multiarray', 'multiarray'):
            self.create_subscription(Float64MultiArray, az_topic, self._on_float_array, 10)
        else:
            self.create_subscription(JointState, az_topic, self._on_joint, 10)

        self._prev_n = 0
        self._clear_arc_ns_once()
        self.get_logger().info(
            f'人物方位圆弧: {az_topic} -> {mt} frame={self._frame} R={self._r}m（仅 LINE_STRIP）'
        )

    def _clear_arc_ns_once(self) -> None:
        viz = self.get_clock().now().to_msg()
        cl = Marker()
        cl.header.frame_id = self._frame
        cl.header.stamp = viz
        cl.ns = _NS
        cl.action = Marker.DELETEALL
        self._pub.publish(MarkerArray(markers=[cl]))

    def _on_float_array(self, msg: Float64MultiArray) -> None:
        self._publish(list(msg.data))

    def _on_joint(self, msg: JointState) -> None:
        self._publish(list(msg.position))

    def _publish(self, positions: List[float]) -> None:
        viz_stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        n = min(_MAX_SECTORS, max(0, len(positions) // 2))
        prev = self._prev_n
        for s in range(n, prev):
            arr.markers.append(self._mk_delete(viz_stamp, s + 1))
        for si in range(n):
            lo, hi = float(positions[si * 2]), float(positions[si * 2 + 1])
            thetas = _arc_thetas(lo, hi, self._npt)
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = viz_stamp
            m.ns = _NS
            m.id = si + 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = self._lw
            m.pose.orientation.w = 1.0
            m.color = self._color
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            for th in thetas:
                m.points.append(Point(x=self._r * math.cos(th), y=self._r * math.sin(th), z=self._z))
            arr.markers.append(m)
        self._prev_n = n
        if arr.markers:
            self._pub.publish(arr)

    def _mk_delete(self, viz_stamp, marker_id: int) -> Marker:
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = viz_stamp
        m.ns = _NS
        m.id = marker_id
        m.action = Marker.DELETE
        return m


def main() -> None:
    rclpy.init()
    node = PersonAzimuthMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
