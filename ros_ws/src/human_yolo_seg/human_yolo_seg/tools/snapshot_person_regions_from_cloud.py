# pyright: reportMissingImports=false
"""从 /human_yolo/person_laser_map_cloud（map 系、Transient Local）拉一帧点云，写成与 person_strip_recorder 同格式的 regions YAML。

用于未起 person_strip_recorder 时，仍可用 annotate_saved_map_person_overlay 在存图上叠人物点。
在 RViz 里该话题与 LaserScan 一样叠在地图上（PointCloud2 + Fixed Frame=map）。"""

from __future__ import annotations

import math
import struct
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
import yaml


def _parse_pointcloud2_xy(msg: PointCloud2) -> List[Tuple[float, float]]:
    """按 fields 找 x/y（float32），兼容 person_strip_recorder 的 xyz+rgb 布局。"""
    x_off = y_off = None
    for f in msg.fields:
        if f.name == 'x':
            x_off = int(f.offset)
        elif f.name == 'y':
            y_off = int(f.offset)
    if x_off is None or y_off is None:
        return []
    endian = '>' if msg.is_bigendian else '<'
    ps = int(msg.point_step)
    n = int(msg.width) * int(msg.height)
    out: List[Tuple[float, float]] = []
    raw = memoryview(msg.data)
    for i in range(n):
        base = i * ps
        try:
            x = struct.unpack_from(endian + 'f', raw, base + x_off)[0]
            y = struct.unpack_from(endian + 'f', raw, base + y_off)[0]
        except struct.error:
            continue
        if math.isfinite(x) and math.isfinite(y):
            out.append((float(x), float(y)))
    return out


def _dedupe_to_regions(
    xy: List[Tuple[float, float]],
    dedupe_m: float,
    strip_r: float,
    map_frame: str,
) -> dict:
    d = max(float(dedupe_m), 0.02)
    cells: Dict[Tuple[int, int], Tuple[float, float]] = {}
    for mx, my in xy:
        gx = int(round(mx / d))
        gy = int(round(my / d))
        cells[(gx, gy)] = (mx, my)
    regions = [{'x': float(x), 'y': float(y), 'radius': float(strip_r)} for x, y in cells.values()]
    regions.sort(key=lambda r: (r['x'], r['y']))
    return {'map_frame': map_frame, 'regions': regions}


class _SnapNode(Node):
    def __init__(self) -> None:
        super().__init__('snapshot_person_regions_from_cloud')
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter(
            'output_path',
            str(Path.home() / '.ros' / 'tj_person_regions_cloud_snapshot.yaml'),
        )
        self.declare_parameter('cloud_topic', '/human_yolo/person_laser_map_cloud')
        self.declare_parameter('timeout_sec', 10.0)
        self.declare_parameter('strip_radius_m', 0.08)
        self.declare_parameter('dedupe_grid_m', 0.05)
        self.declare_parameter('map_frame', 'map')

        out = self.get_parameter('output_path').get_parameter_value().string_value
        self._output_path = str(Path(out).expanduser())
        topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        self._timeout = max(float(self.get_parameter('timeout_sec').get_parameter_value().double_value), 0.5)
        self._strip_r = max(float(self.get_parameter('strip_radius_m').get_parameter_value().double_value), 0.02)
        self._dedupe = max(float(self.get_parameter('dedupe_grid_m').get_parameter_value().double_value), 0.02)
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value or 'map'

        self._msg: Optional[PointCloud2] = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.get_logger().info(
            f'等待 PointCloud2 {topic}（Transient Local），最长 {self._timeout}s -> {self._output_path}'
        )

    def _cb(self, msg: PointCloud2) -> None:
        if self._msg is None:
            self._msg = msg


def main() -> None:
    rclpy.init()
    node = _SnapNode()
    deadline = time.monotonic() + node._timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and node._msg is None:
            rclpy.spin_once(node, timeout_sec=0.15)
    finally:
        path = Path(node._output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if node._msg is not None:
            fr = node._msg.header.frame_id or node._map_frame
            xy = _parse_pointcloud2_xy(node._msg)
            doc = _dedupe_to_regions(xy, node._dedupe, node._strip_r, fr)
            node.get_logger().info(f'收到 {len(xy)} 点 -> {len(doc["regions"])} 个去重区域')
        else:
            doc = _dedupe_to_regions([], node._dedupe, node._strip_r, node._map_frame)
            node.get_logger().warning('超时未收到点云，写入空 regions（需 person_strip_recorder 或仿真中曾发布该话题）')
        with path.open('w', encoding='utf-8') as f:
            f.write(
                '# TJ-Robot：由 snapshot_person_regions_from_cloud 从 person_laser_map_cloud 生成\n'
            )
            yaml.safe_dump(doc, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        print(f'写入 {path}（regions={len(doc["regions"])}）')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
