#!/usr/bin/env python3
"""基于激光扫描的静态地图覆盖率监控。

回答一个实际问题："已知空闲空间中，有多少已被机器人 2D 激光观测过？"
以 /map 中的空闲栅格为总量，当 /scan 射线穿过某个栅格时将其标记为已覆盖。
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CoverageMonitor(Node):
    """覆盖率监控节点。

    核心思路：维护两张栅格表 —— eligible（可通行空闲栅格）和 covered（已观测栅格）。
    收到 /scan 后通过 TF 变换将激光点映射到地图坐标系，沿射线步进标记覆盖。
    定时发布覆盖率百分比、完成标志以及可视化栅格图，供上层调度（如 navigation_manager）
    判断是否继续巡逻或切换区域。
    """

    def __init__(self) -> None:
        super().__init__("coverage_monitor")

        # use_sim_time 是 ROS 2 的内置参数；launch 传入后节点已自动声明，重复声明会导致节点启动失败。
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("coverage_grid_topic", "/navigation/coverage_grid")
        self.declare_parameter("coverage_status_topic", "/navigation/coverage_status")
        self.declare_parameter("coverage_done_topic", "/navigation/coverage_done")
        self.declare_parameter("free_occupancy_threshold", 25)
        self.declare_parameter("unknown_as_free", False)
        self.declare_parameter("ray_stride", 3)
        self.declare_parameter("scan_throttle_sec", 0.25)
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("required_coverage_percent", 95.0)
        self.declare_parameter("max_range_override_m", 0.0)
        self.declare_parameter("mark_robot_radius_m", 0.18)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._free_threshold = int(self.get_parameter("free_occupancy_threshold").value)
        self._unknown_as_free = bool(self.get_parameter("unknown_as_free").value)
        self._ray_stride = max(int(self.get_parameter("ray_stride").value), 1)
        self._scan_throttle_sec = max(float(self.get_parameter("scan_throttle_sec").value), 0.0)
        publish_period = max(float(self.get_parameter("publish_period_sec").value), 0.2)
        self._required_percent = max(
            float(self.get_parameter("required_coverage_percent").value), 0.0
        )
        self._max_range_override_m = max(
            float(self.get_parameter("max_range_override_m").value), 0.0
        )
        self._mark_robot_radius_m = max(
            float(self.get_parameter("mark_robot_radius_m").value), 0.0
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._grid_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("coverage_grid_topic").value),
            status_qos,
        )
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter("coverage_status_topic").value),
            status_qos,
        )
        self._done_pub = self.create_publisher(
            Bool,
            str(self.get_parameter("coverage_done_topic").value),
            status_qos,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._map_info: MapMetaData | None = None
        self._map_geometry: tuple[int, int, float, float, float, float] | None = None
        self._eligible = bytearray()
        self._covered = bytearray()
        self._free_count = 0
        self._covered_count = 0
        self._last_scan_time = 0.0
        self._last_tf_warn_time = 0.0
        self._last_log_time = 0.0

        self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, map_qos)
        self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(publish_period, self._publish_status)

        self.get_logger().info(
            "coverage_monitor ready: "
            f"map={self._map_topic} scan={self._scan_topic} "
            f"required={self._required_percent:.1f}%"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        """接收静态地图，构建 eligible 与 covered 两个栅格数组。

        eligible 记录所有"可通行空闲栅格"，作为覆盖率计算的分母。
        covered 记录已被激光扫过的栅格，作为分子。
        若地图几何不变则保留已有的 covered 数据，避免重复建图导致覆盖率归零。
        """
        info = msg.info
        cell_count = int(info.width * info.height)
        origin = info.origin
        origin_yaw = yaw_from_quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )
        geometry = (
            int(info.width),
            int(info.height),
            float(info.resolution),
            float(origin.position.x),
            float(origin.position.y),
            float(origin_yaw),
        )

        same_geometry = self._map_geometry == geometry and len(self._covered) == cell_count
        covered = self._covered if same_geometry else bytearray(cell_count)
        eligible = bytearray(cell_count)
        free_count = 0

        for index, occ in enumerate(msg.data):
            is_free = (occ == -1 and self._unknown_as_free) or (0 <= occ <= self._free_threshold)
            if is_free:
                eligible[index] = 1
                free_count += 1
            else:
                covered[index] = 0

        self._map_info = info
        self._map_geometry = geometry
        self._eligible = eligible
        self._covered = covered
        self._free_count = free_count
        self._covered_count = sum(
            1 for index in range(cell_count) if eligible[index] and covered[index]
        )

        self.get_logger().info(
            f"Loaded coverage map: {info.width}x{info.height}, free_cells={self._free_count}"
        )

    def _on_scan(self, scan: LaserScan) -> None:
        """处理激光扫描帧：TF 变换 → 原点标记 → 射线遍历。

        每条射线从激光原点出发，沿扫描方向以地图分辨率步进，将经过的空闲栅格标记为已覆盖。
        ray_stride 控制跳帧采样（每隔 N 条射线取一条），以降低 CPU 开销。
        """
        if self._map_info is None or not self._eligible:
            return
        now = time.monotonic()
        if now - self._last_scan_time < self._scan_throttle_sec:
            return
        self._last_scan_time = now

        scan_frame = scan.header.frame_id.strip()
        if not scan_frame:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                scan_frame,
                Time(),
            )
        except TransformException as exc:
            if now - self._last_tf_warn_time > 5.0:
                self._last_tf_warn_time = now
                self.get_logger().warn(
                    f"Waiting for TF {self._map_frame} <- {scan_frame}: {exc}"
                )
            return

        trans = transform.transform.translation
        rot = transform.transform.rotation
        origin_x = float(trans.x)
        origin_y = float(trans.y)
        yaw = yaw_from_quaternion(rot.x, rot.y, rot.z, rot.w)

        self._mark_robot_disk(origin_x, origin_y)

        max_range = self._max_range_override_m or float(scan.range_max)
        if not math.isfinite(max_range) or max_range <= 0.0:
            max_range = 8.0
        min_range = max(float(scan.range_min), 0.0)

        angle = float(scan.angle_min)
        angle_increment = float(scan.angle_increment)
        for index in range(0, len(scan.ranges), self._ray_stride):
            measured = float(scan.ranges[index])
            if math.isnan(measured) or measured < min_range:
                angle += angle_increment * self._ray_stride
                continue
            distance = max_range if math.isinf(measured) else min(measured, max_range)
            self._mark_ray(origin_x, origin_y, yaw + angle, distance)
            angle += angle_increment * self._ray_stride

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """世界坐标 → 地图栅格索引，支持带旋转的地图原点。"""
        if self._map_geometry is None:
            return None
        width, height, resolution, origin_x, origin_y, origin_yaw = self._map_geometry
        dx = x - origin_x
        dy = y - origin_y
        cos_yaw = math.cos(-origin_yaw)
        sin_yaw = math.sin(-origin_yaw)
        map_x = (cos_yaw * dx - sin_yaw * dy) / resolution
        map_y = (sin_yaw * dx + cos_yaw * dy) / resolution
        cell_x = int(math.floor(map_x))
        cell_y = int(math.floor(map_y))
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return None
        return cell_x, cell_y

    def _mark_robot_disk(self, x: float, y: float) -> None:
        """以机器人为中心标记一个圆形区域为已覆盖，消除自身占据带来的误判。"""
        cell = self._world_to_cell(x, y)
        if cell is None or self._map_info is None:
            return
        radius_cells = int(math.ceil(self._mark_robot_radius_m / self._map_info.resolution))
        cx, cy = cell
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                self._mark_cell(cx + dx, cy + dy)

    def _mark_ray(self, origin_x: float, origin_y: float, angle: float, distance: float) -> None:
        """从激光原点沿指定角度步进，将射线穿过的所有栅格标记为已覆盖。

        步长为分辨率的 0.75 倍（最小 3cm），保证不会跨过窄栅格。
        遇到地图边界外的点则提前终止射线。
        """
        if self._map_info is None:
            return
        step = max(float(self._map_info.resolution) * 0.75, 0.03)
        steps = max(int(math.ceil(distance / step)), 1)
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        last_cell: tuple[int, int] | None = None

        for step_index in range(steps + 1):
            d = min(step_index * step, distance)
            cell = self._world_to_cell(origin_x + cos_a * d, origin_y + sin_a * d)
            if cell is None:
                if step_index > 0:
                    break
                continue
            if cell == last_cell:
                continue
            last_cell = cell
            self._mark_cell(cell[0], cell[1])

    def _mark_cell(self, cell_x: int, cell_y: int) -> None:
        """标记单个栅格为已覆盖，同时更新计数器。非空闲栅格或已覆盖栅格会被跳过。"""
        if self._map_geometry is None:
            return
        width, height, *_ = self._map_geometry
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return
        index = cell_y * width + cell_x
        if not self._eligible[index] or self._covered[index]:
            return
        self._covered[index] = 1
        self._covered_count += 1

    def _coverage_percent(self) -> float:
        if self._free_count <= 0:
            return 0.0
        return 100.0 * float(self._covered_count) / float(self._free_count)

    def _publish_status(self) -> None:
        """定时发布覆盖率状态、完成标志与可视化栅格图。

        输出三个话题：
        - coverage_status: JSON 字符串，含百分比、栅格计数等
        - coverage_done: Bool，是否达到 required_coverage_percent
        - coverage_grid: OccupancyGrid，用于 RViz 可视化（-1=非空闲, 0=未扫, 100=已扫）
        """
        if self._map_info is None:
            return

        percent = self._coverage_percent()
        done = self._free_count > 0 and percent >= self._required_percent
        status = {
            "coverage_percent": round(percent, 2),
            "required_coverage_percent": round(self._required_percent, 2),
            "done": done,
            "covered_cells": int(self._covered_count),
            "total_free_cells": int(self._free_count),
            "uncovered_cells": int(max(self._free_count - self._covered_count, 0)),
            "map_topic": self._map_topic,
            "scan_topic": self._scan_topic,
        }
        self._status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        self._done_pub.publish(Bool(data=done))
        self._publish_grid()

        now = time.monotonic()
        if now - self._last_log_time > 5.0 or done:
            self._last_log_time = now
            self.get_logger().info(
                f"coverage={percent:.1f}% required={self._required_percent:.1f}% "
                f"covered={self._covered_count}/{self._free_count} done={done}"
            )

    def _publish_grid(self) -> None:
        if self._map_info is None or self._map_geometry is None:
            return
        width, height, *_ = self._map_geometry
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._map_frame
        grid.info = self._map_info

        data: list[int] = []
        cell_count = width * height
        for index in range(cell_count):
            if not self._eligible[index]:
                data.append(-1)
            elif self._covered[index]:
                data.append(100)
            else:
                data.append(0)
        grid.data = data
        self._grid_pub.publish(grid)


def main() -> None:
    rclpy.init()
    node = CoverageMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
