# pyright: reportMissingImports=false
"""订阅人物方位角区间，在 laser frame（默认 base_scan）内发布 MarkerArray，供 RViz 对照激光角域。

外观：以激光系原点为圆心、在 xy 平面上一段「扇形」——半透明浅绿底 + 外沿亮绿弧线（与 /scan 的 atan2 约定一致：x 前、y 左）。
角度字：TEXT_VIEW_FACING，默认变换到 map 系（与 RViz Fixed Frame=map 一致），黄色大字；在 RViz 主 3D 视图俯视机器人附近查看，非 Image 面板。"""
from __future__ import annotations

import math
from typing import List

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped
from rclpy.duration import Duration
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray

from human_yolo_seg.person_scan_sync_utils import norm_angle

try:
    from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
except ImportError:
    from tf2_geometry_msgs import do_transform_point  # type: ignore


def _arc_thetas(lo: float, hi: float, n: int) -> List[float]:
    """在 [-pi,pi] 上从 lo 沿短弧采样到 hi（可跨 ±pi 边界）。"""
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
        self.declare_parameter('publish_sector_fill', True)
        self.declare_parameter('fill_alpha', 0.28)
        self.declare_parameter('color_r', 0.15)
        self.declare_parameter('color_g', 1.0)
        self.declare_parameter('color_b', 0.35)
        self.declare_parameter('color_a', 0.95)
        self.declare_parameter('publish_angle_labels', True)
        self.declare_parameter('label_radius_scale', 1.18)
        self.declare_parameter('label_z_offset_m', 0.28)
        self.declare_parameter('label_stagger_m', 0.06)
        self.declare_parameter('label_text_scale_z', 0.42)
        self.declare_parameter('label_color_r', 1.0)
        self.declare_parameter('label_color_g', 0.92)
        self.declare_parameter('label_color_b', 0.15)
        self.declare_parameter('label_color_a', 1.0)
        # 非空则把角度文字变换到该 frame（默认 map，与 RViz Fixed Frame 一致，俯视时好找）
        self.declare_parameter('label_reference_frame', 'map')
        self.declare_parameter('publish_debug_string', True)
        self.declare_parameter('debug_string_topic', '/human_yolo/person_azimuth_debug')
        self.declare_parameter('publish_debug_hud_marker', True)
        self.declare_parameter('debug_hud_map_x', 0.0)
        self.declare_parameter('debug_hud_map_y', 0.0)
        self.declare_parameter('debug_hud_map_z', 4.5)
        self.declare_parameter('debug_hud_text_scale', 0.48)

        self._frame = self.get_parameter('laser_frame_id').get_parameter_value().string_value or 'base_scan'
        self._r = max(float(self.get_parameter('arc_radius_m').get_parameter_value().double_value), 0.15)
        self._z = float(self.get_parameter('arc_z_m').get_parameter_value().double_value)
        self._npt = max(8, int(self.get_parameter('arc_points').get_parameter_value().integer_value))
        self._lw = max(0.005, float(self.get_parameter('line_width_m').get_parameter_value().double_value))
        self._do_fill = bool(self.get_parameter('publish_sector_fill').get_parameter_value().bool_value)
        self._fill_a = max(0.0, min(1.0, float(self.get_parameter('fill_alpha').get_parameter_value().double_value)))
        cr = float(self.get_parameter('color_r').get_parameter_value().double_value)
        cg = float(self.get_parameter('color_g').get_parameter_value().double_value)
        cb = float(self.get_parameter('color_b').get_parameter_value().double_value)
        ca = float(self.get_parameter('color_a').get_parameter_value().double_value)
        self._color_line = ColorRGBA(r=cr, g=cg, b=cb, a=max(0.05, min(1.0, ca)))
        self._color_fill = ColorRGBA(r=cr, g=cg, b=cb, a=self._fill_a)
        self._pub_labels = bool(self.get_parameter('publish_angle_labels').get_parameter_value().bool_value)
        self._label_r_scale = max(0.5, float(self.get_parameter('label_radius_scale').get_parameter_value().double_value))
        self._label_z_off = float(self.get_parameter('label_z_offset_m').get_parameter_value().double_value)
        self._label_stagger = max(0.0, float(self.get_parameter('label_stagger_m').get_parameter_value().double_value))
        self._label_sz = max(0.04, float(self.get_parameter('label_text_scale_z').get_parameter_value().double_value))
        lr = float(self.get_parameter('label_color_r').get_parameter_value().double_value)
        lg = float(self.get_parameter('label_color_g').get_parameter_value().double_value)
        lb = float(self.get_parameter('label_color_b').get_parameter_value().double_value)
        la = float(self.get_parameter('label_color_a').get_parameter_value().double_value)
        self._color_label = ColorRGBA(
            r=max(0.0, min(1.0, lr)),
            g=max(0.0, min(1.0, lg)),
            b=max(0.0, min(1.0, lb)),
            a=max(0.05, min(1.0, la)),
        )
        self._label_ref = (self.get_parameter('label_reference_frame').get_parameter_value().string_value or '').strip()

        self._tf_buffer: tf2_ros.Buffer | None = None
        self._tf_listener: tf2_ros.TransformListener | None = None
        if self._pub_labels and self._label_ref:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self, spin_thread=True)

        self._pub_dbg_str = None
        if self.get_parameter('publish_debug_string').get_parameter_value().bool_value:
            dtopic = self.get_parameter('debug_string_topic').get_parameter_value().string_value
            self._pub_dbg_str = self.create_publisher(String, dtopic, 10)
        self._dbg_hud = bool(self.get_parameter('publish_debug_hud_marker').get_parameter_value().bool_value)
        self._hud_x = float(self.get_parameter('debug_hud_map_x').get_parameter_value().double_value)
        self._hud_y = float(self.get_parameter('debug_hud_map_y').get_parameter_value().double_value)
        self._hud_z = float(self.get_parameter('debug_hud_map_z').get_parameter_value().double_value)
        self._hud_scale = max(0.12, float(self.get_parameter('debug_hud_text_scale').get_parameter_value().double_value))

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
            self.get_logger().info(
                f'人物方位扇形: Float64MultiArray {az_topic} -> {mt} (frame={self._frame}, R={self._r}m)；'
                '调试: ros2 topic echo /human_yolo/person_azimuth_debug；'
                'RViz 同话题下 Namespace person_azimuth_hud 为地图上方汇总字'
            )
        else:
            self.create_subscription(JointState, az_topic, self._on_joint, 10)
            self.get_logger().info(
                f'人物方位扇形: JointState {az_topic} -> {mt} (frame={self._frame}, R={self._r}m)；'
                '调试: ros2 topic echo /human_yolo/person_azimuth_debug；'
                'RViz 同话题下 Namespace person_azimuth_hud 为地图上方汇总字'
            )

        self._logged_nonempty = False

    @staticmethod
    def _fmt_deg(rad: float) -> str:
        return f'{math.degrees(norm_angle(rad)):.1f}'

    def _text_frame_and_xyz(
        self, lx: float, ly: float, lz: float, stamp
    ) -> tuple[str, float, float, float]:
        """角度文字用的坐标系与位置；优先变换到 label_reference_frame。"""
        if not self._label_ref or self._tf_buffer is None:
            return self._frame, lx, ly, lz
        ps = PointStamped()
        ps.header.frame_id = self._frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.point.x = lx
        ps.point.y = ly
        ps.point.z = lz
        try:
            trans = self._tf_buffer.lookup_transform(
                self._label_ref,
                self._frame,
                self.get_clock().now(),
                timeout=Duration(seconds=0, nanoseconds=250_000_000),
            )
            p2 = do_transform_point(ps, trans)
            return self._label_ref, float(p2.point.x), float(p2.point.y), float(p2.point.z)
        except Exception:
            return self._frame, lx, ly, lz

    def _on_float_array(self, msg: Float64MultiArray) -> None:
        self._publish(list(msg.data), self.get_clock().now().to_msg())

    def _on_joint(self, msg: JointState) -> None:
        self._publish(list(msg.position), msg.header.stamp)

    def _publish(self, positions: List[float], stamp) -> None:
        # 可视化用当前仿真/墙钟时间，避免旧 JointState 时间戳导致 RViz TF 外推报错牵连 Map 等显示
        viz_stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self._frame
        clear.header.stamp = viz_stamp
        clear.ns = 'person_azimuth_arc'
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        if self._pub_labels:
            for cfr in (self._frame, 'map'):
                if cfr == 'map' and not self._label_ref:
                    continue
                clear_txt = Marker()
                clear_txt.header.frame_id = cfr
                clear_txt.header.stamp = viz_stamp
                clear_txt.ns = 'person_azimuth_text'
                clear_txt.action = Marker.DELETEALL
                arr.markers.append(clear_txt)

        if self._dbg_hud:
            ch = Marker()
            ch.header.frame_id = 'map'
            ch.header.stamp = viz_stamp
            ch.ns = 'person_azimuth_hud'
            ch.action = Marker.DELETEALL
            arr.markers.append(ch)

        mid = 1
        tid = 0
        n_sectors = max(0, (len(positions) // 2))
        for i in range(0, len(positions) - 1, 2):
            lo, hi = float(positions[i]), float(positions[i + 1])
            thetas = _arc_thetas(lo, hi, self._npt)
            if self._do_fill and self._fill_a > 0.01 and len(thetas) >= 2:
                fan = Marker()
                fan.header.frame_id = self._frame
                fan.header.stamp = viz_stamp
                fan.ns = 'person_azimuth_arc'
                fan.id = mid
                mid += 1
                fan.type = Marker.TRIANGLE_FAN
                fan.action = Marker.ADD
                fan.scale.x = 1.0
                fan.scale.y = 1.0
                fan.scale.z = 1.0
                fan.pose.orientation.w = 1.0
                fan.color = self._color_fill
                fan.lifetime.sec = 0
                fan.lifetime.nanosec = 0
                fan.points.append(Point(x=0.0, y=0.0, z=self._z))
                for th in thetas:
                    fan.points.append(
                        Point(
                            x=self._r * math.cos(th),
                            y=self._r * math.sin(th),
                            z=self._z,
                        )
                    )
                arr.markers.append(fan)

            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = viz_stamp
            m.ns = 'person_azimuth_arc'
            m.id = mid
            mid += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = self._lw
            m.pose.orientation.w = 1.0
            m.color = self._color_line
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            for th in thetas:
                p = Point()
                p.x = self._r * math.cos(th)
                p.y = self._r * math.sin(th)
                p.z = self._z
                m.points.append(p)
            arr.markers.append(m)

            if self._pub_labels and len(thetas) >= 1:
                bisect = thetas[len(thetas) // 2]
                lr = self._r * self._label_r_scale
                zt = self._z + self._label_z_off + (i // 2) * self._label_stagger
                lx = lr * math.cos(bisect)
                ly = lr * math.sin(bisect)
                tfr, px, py, pz = self._text_frame_and_xyz(lx, ly, zt, stamp)
                tm = Marker()
                tm.header.frame_id = tfr
                tm.header.stamp = viz_stamp
                tm.ns = 'person_azimuth_text'
                tm.id = tid
                tid += 1
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = px
                tm.pose.position.y = py
                tm.pose.position.z = pz
                tm.pose.orientation.w = 1.0
                tm.scale.z = self._label_sz
                tm.color = self._color_label
                tm.lifetime.sec = 0
                tm.lifetime.nanosec = 0
                tm.text = (
                    f'人{tid} {self._fmt_deg(lo)}° ~ {self._fmt_deg(hi)}°'
                )
                arr.markers.append(tm)

        if self._pub_labels and n_sectors == 0:
            lz = self._z + self._label_z_off + 0.12
            tfr, px, py, pz = self._text_frame_and_xyz(0.0, 0.0, lz, stamp)
            tm = Marker()
            tm.header.frame_id = tfr
            tm.header.stamp = viz_stamp
            tm.ns = 'person_azimuth_text'
            tm.id = 0
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose.position.x = px
            tm.pose.position.y = py
            tm.pose.position.z = pz
            tm.pose.orientation.w = 1.0
            tm.scale.z = self._label_sz * 0.9
            tm.color = self._color_label
            tm.lifetime.sec = 0
            tm.lifetime.nanosec = 0
            tm.text = '方位角: 无人'
            arr.markers.append(tm)

        if self._pub_dbg_str is not None:
            parts_deg: list[str] = []
            for i in range(0, len(positions) - 1, 2):
                lo, hi = float(positions[i]), float(positions[i + 1])
                parts_deg.append(f"{self._fmt_deg(lo)}~{self._fmt_deg(hi)}°")
            self._pub_dbg_str.publish(
                String(
                    data=(
                        f"[markers] rx_len={len(positions)} pairs={n_sectors} laser_frame={self._frame} "
                        f"deg=[{', '.join(parts_deg)}] rad={positions!r}"
                    )
                )
            )

        if self._dbg_hud:
            lines = [
                '=== person_azimuth 调试 HUD (map 固定点) ===',
                '扇形在机器人 laser 原点；此处仅汇总数值',
                f'laser_frame: {self._frame}',
                f'收到区间数: {n_sectors}  raw_floats={len(positions)}',
            ]
            if n_sectors == 0:
                lines.append('无区间: 看终端 ros2 topic echo 里 [yolo] 是否也为 pairs=0')
            else:
                for i in range(0, len(positions) - 1, 2):
                    lo, hi = float(positions[i]), float(positions[i + 1])
                    lines.append(f'  #{i // 2 + 1} {self._fmt_deg(lo)} ~ {self._fmt_deg(hi)} deg')
            hud = Marker()
            hud.header.frame_id = 'map'
            hud.header.stamp = viz_stamp
            hud.ns = 'person_azimuth_hud'
            hud.id = 0
            hud.type = Marker.TEXT_VIEW_FACING
            hud.action = Marker.ADD
            hud.pose.position.x = self._hud_x
            hud.pose.position.y = self._hud_y
            hud.pose.position.z = self._hud_z
            hud.pose.orientation.w = 1.0
            hud.scale.z = self._hud_scale
            hud.color = ColorRGBA(r=0.05, g=0.95, b=1.0, a=1.0)
            hud.lifetime.sec = 0
            hud.lifetime.nanosec = 0
            hud.text = '\n'.join(lines)
            arr.markers.append(hud)

        self._pub.publish(arr)

        if n_sectors > 0 and not self._logged_nonempty:
            self.get_logger().info(
                f'已发布 {n_sectors} 个方位扇形（俯视：机器人位置附近浅绿扇区 + 外沿亮线）'
            )
            self._logged_nonempty = True


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
