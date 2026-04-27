"""先直行直到激光看到墙，再沿墙行走（右手/左手规则）。

用于小空间快速建图：不依赖 Nav2，只用 /scan + /odom 发 /cmd_vel。

说明：
- 「覆盖整个地图」若只靠单圈沿墙，通常只能可靠覆盖**外轮廓**；房间内部需多圈或换策略。
- 可选：绕回起点附近且已走够长距离时自动停止（近似一圈）。
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan

from robot_navigation.utils.nav_math import Pose2D, clamp, normalize_angle, yaw_from_quaternion


def _angle_in_interval(theta: float, lo: float, hi: float) -> bool:
    t = normalize_angle(theta)
    lo = normalize_angle(lo)
    hi = normalize_angle(hi)
    if lo <= hi:
        return lo <= t <= hi
    return t >= lo or t <= hi


def _intervals_from_joint_state(msg: JointState) -> List[Tuple[float, float]]:
    p = list(msg.position)
    pairs: List[Tuple[float, float]] = []
    for i in range(0, len(p) - 1, 2):
        pairs.append((float(p[i]), float(p[i + 1])))
    return pairs


def _angular_intervals_overlap(lo_a: float, hi_a: float, lo_b: float, hi_b: float) -> bool:
    for i in range(9):
        t = lo_a + (hi_a - lo_a) * (i / 8.0)
        if _angle_in_interval(normalize_angle(t), lo_b, hi_b):
            return True
    for i in range(9):
        t = lo_b + (hi_b - lo_b) * (i / 8.0)
        if _angle_in_interval(normalize_angle(t), lo_a, hi_a):
            return True
    return False


class Phase(Enum):
    PRE_ROTATE = auto()  # 可选：原地转向指定角
    SEEK_WALL = auto()  # 直行直到前方足够近
    FOLLOW_WALL = auto()  # 沿墙
    DONE = auto()


class WallFollowCoverage(Node):
    def __init__(self) -> None:
        super().__init__("wall_follow_coverage")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_rate_hz", 20.0)

        # 启动后先原地转 pre_seek_rotate_deg（度），再开始直行找墙；0 表示不转
        self.declare_parameter("pre_seek_rotate_deg", 0.0)
        self.declare_parameter("pre_rotate_angular_speed", 0.8)

        self.declare_parameter("seek_linear_speed", 0.22)
        self.declare_parameter("wall_found_distance", 0.42)  # 前方认为“碰到墙”

        self.declare_parameter("follow_linear_speed", 0.18)
        self.declare_parameter("follow_angular_speed", 0.85)
        self.declare_parameter("wall_follow_side", "right")  # right | left

        # 沿墙期望与侧向墙的距离（米），仅作大致引导
        self.declare_parameter("wall_target_range", 0.55)
        self.declare_parameter("wall_lost_range", 1.15)

        self.declare_parameter("front_block_distance", 0.38)
        self.declare_parameter("corner_extra_turn_deg", 12.0)

        # 结束条件
        self.declare_parameter("max_runtime_sec", 0.0)  # 0 = 不限制
        self.declare_parameter("stop_on_lap", False)
        self.declare_parameter("lap_radius_m", 0.55)
        self.declare_parameter("min_lap_traveled_m", 10.0)

        # 与 coverage_patrol 一致：YOLO 人物角域 + 激光虚拟距离，避免贴撞静止人形
        self.declare_parameter("enable_people_avoidance", True)
        self.declare_parameter("person_azimuth_topic", "/human_yolo/person_azimuth_ranges")
        self.declare_parameter("person_hold_seconds", 0.7)
        self.declare_parameter("person_virtual_range_m", 1.15)
        self.declare_parameter("person_front_stop_m", 0.55)

        odom_topic = str(self.get_parameter("odom_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        rate_hz = float(self.get_parameter("control_rate_hz").value)

        self._pre_seek_rotate_deg = float(self.get_parameter("pre_seek_rotate_deg").value)
        self._pre_rotate_w = float(self.get_parameter("pre_rotate_angular_speed").value)
        self._seek_v = float(self.get_parameter("seek_linear_speed").value)
        self._wall_found = float(self.get_parameter("wall_found_distance").value)
        self._follow_v = float(self.get_parameter("follow_linear_speed").value)
        self._follow_w = float(self.get_parameter("follow_angular_speed").value)
        side = str(self.get_parameter("wall_follow_side").value).strip().lower()
        self._right_hand = side != "left"
        self._wall_target = float(self.get_parameter("wall_target_range").value)
        self._wall_lost = float(self.get_parameter("wall_lost_range").value)
        self._front_block = float(self.get_parameter("front_block_distance").value)
        self._corner_extra = math.radians(float(self.get_parameter("corner_extra_turn_deg").value))

        self._max_runtime = float(self.get_parameter("max_runtime_sec").value)
        self._stop_on_lap = bool(self.get_parameter("stop_on_lap").value)
        self._lap_r = float(self.get_parameter("lap_radius_m").value)
        self._min_lap = float(self.get_parameter("min_lap_traveled_m").value)

        self._enable_people = bool(self.get_parameter("enable_people_avoidance").value)
        self._person_topic = str(self.get_parameter("person_azimuth_topic").value)
        self._person_hold = float(self.get_parameter("person_hold_seconds").value)
        self._person_virtual = float(self.get_parameter("person_virtual_range_m").value)
        self._person_stop = float(self.get_parameter("person_front_stop_m").value)

        self._pose: Optional[Pose2D] = None
        self._last_scan: Optional[LaserScan] = None
        self._person_intervals: List[Tuple[float, float]] = []
        self._person_last_ns: Optional[int] = None
        self._phase = Phase.PRE_ROTATE if abs(self._pre_seek_rotate_deg) > 1e-3 else Phase.SEEK_WALL
        self._pre_rotate_target_yaw: Optional[float] = None
        self._start_xy: Optional[tuple[float, float]] = None
        self._last_xy: Optional[tuple[float, float]] = None
        self._traveled_m = 0.0
        self._start_ns: Optional[int] = None

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        if self._enable_people:
            self.create_subscription(JointState, self._person_topic, self._on_person_azimuth, 10)
        self.create_timer(1.0 / max(rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f"wall_follow_coverage: phase={self._phase.name} side={'right' if self._right_hand else 'left'} "
            f"people_avoidance={self._enable_people}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._pose = Pose2D(
            float(p.x),
            float(p.y),
            normalize_angle(yaw_from_quaternion(o.z, o.w)),
        )
        if self._start_ns is None:
            self._start_ns = self.get_clock().now().nanoseconds
        if self._start_xy is None and self._pose is not None:
            self._start_xy = (self._pose.x, self._pose.y)
            self._last_xy = self._start_xy
        if self._pose is not None and self._last_xy is not None:
            dx = self._pose.x - self._last_xy[0]
            dy = self._pose.y - self._last_xy[1]
            self._traveled_m += math.hypot(dx, dy)
            self._last_xy = (self._pose.x, self._pose.y)

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _on_person_azimuth(self, msg: JointState) -> None:
        self._person_intervals = _intervals_from_joint_state(msg)
        self._person_last_ns = self.get_clock().now().nanoseconds

    def _person_recent(self) -> bool:
        if not self._enable_people or self._person_last_ns is None:
            return False
        age = (self.get_clock().now().nanoseconds - self._person_last_ns) / 1e9
        return age <= max(self._person_hold, 0.0)

    def _person_ranges_overlap_sector(self, min_deg: float, max_deg: float) -> bool:
        if not self._person_recent():
            return False
        a_lo = math.radians(min_deg)
        a_hi = math.radians(max_deg)
        for lo, hi in self._person_intervals:
            if _angular_intervals_overlap(a_lo, a_hi, lo, hi):
                return True
        return False

    def _sector_min(self, min_deg: float, max_deg: float) -> Optional[float]:
        if self._last_scan is None:
            return None
        msg = self._last_scan
        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        angle = float(msg.angle_min)
        best = float("inf")
        found = False
        for sample in msg.ranges:
            a = normalize_angle(angle)
            v = float(sample)
            if min_rad <= a <= max_rad and math.isfinite(v):
                if msg.range_min <= v <= msg.range_max:
                    best = min(best, v)
                    found = True
            angle += float(msg.angle_increment)
        return best if found else None

    def _sector_min_effective(self, min_deg: float, max_deg: float) -> Optional[float]:
        raw = self._sector_min(min_deg, max_deg)
        if not self._enable_people or not self._person_ranges_overlap_sector(min_deg, max_deg):
            return raw
        v = max(self._person_virtual, 0.05)
        if raw is None:
            return v
        return min(raw, v)

    def _twist_avoid_person_if_needed(self) -> Optional[Twist]:
        """前方扇区有人（YOLO+激光）且过近：原地转向更空的一侧，避免顶人。"""
        if not self._enable_people or not self._person_ranges_overlap_sector(-42.0, 42.0):
            return None
        front = self._sector_min_effective(-35.0, 35.0)
        if front is None or front >= self._person_stop:
            return None
        left = self._sector_min(35.0, 110.0)
        right = self._sector_min(-110.0, -35.0)
        lf = left if left is not None else 10.0
        rt = right if right is not None else 10.0
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = 0.85 if lf > rt else -0.85
        return tw

    def _maybe_finish(self) -> bool:
        if self._max_runtime > 0.0 and self._start_ns is not None:
            if (self.get_clock().now().nanoseconds - self._start_ns) / 1e9 >= self._max_runtime:
                self.get_logger().info("max_runtime_sec reached, stopping.")
                return True
        if self._stop_on_lap and self._phase == Phase.FOLLOW_WALL and self._pose is not None and self._start_xy is not None:
            if self._traveled_m >= self._min_lap:
                d = math.hypot(self._pose.x - self._start_xy[0], self._pose.y - self._start_xy[1])
                if d < self._lap_r:
                    self.get_logger().info("Lap closure: near start after enough travel, stopping.")
                    return True
        return False

    def _tick(self) -> None:
        if self._phase == Phase.DONE:
            self._pub.publish(Twist())
            return
        if self._maybe_finish():
            self._phase = Phase.DONE
            self._pub.publish(Twist())
            return
        if self._pose is None or self._last_scan is None:
            return

        tw = Twist()

        if self._phase == Phase.PRE_ROTATE:
            if self._pre_rotate_target_yaw is None:
                self._pre_rotate_target_yaw = normalize_angle(
                    self._pose.yaw + math.radians(self._pre_seek_rotate_deg)
                )
            err = normalize_angle(self._pre_rotate_target_yaw - self._pose.yaw)
            if abs(err) < 0.08:
                self._phase = Phase.SEEK_WALL
                self.get_logger().info("PRE_ROTATE done -> SEEK_WALL")
                return
            tw.angular.z = clamp(self._pre_rotate_w * err, -1.2, 1.2)
            self._pub.publish(tw)
            return

        if self._phase == Phase.SEEK_WALL:
            avoid = self._twist_avoid_person_if_needed()
            if avoid is not None:
                self._pub.publish(avoid)
                return
            front = self._sector_min_effective(-28.0, 28.0)
            if front is not None and front < self._wall_found:
                self._phase = Phase.FOLLOW_WALL
                self.get_logger().info("SEEK_WALL -> FOLLOW_WALL (front range ok)")
                return
            tw.linear.x = self._seek_v
            # 轻微修正前方障碍（斜向接近墙）
            if front is not None and front < self._wall_found * 1.8:
                tw.angular.z = 0.25 if self._right_hand else -0.25
            self._pub.publish(tw)
            return

        # FOLLOW_WALL: 右手规则：墙在右侧；左手则镜像
        avoid = self._twist_avoid_person_if_needed()
        if avoid is not None:
            self._pub.publish(avoid)
            return
        front = self._sector_min_effective(-32.0, 32.0)
        if self._right_hand:
            side = self._sector_min_effective(-105.0, -40.0)  # 右侧墙
            lost_turn = 1.0
            close_turn = -1.0
        else:
            side = self._sector_min_effective(40.0, 105.0)  # 左侧墙
            lost_turn = -1.0
            close_turn = 1.0

        if front is not None and front < self._front_block:
            # 前方有墙：转弯（右手法则左转）
            tw.linear.x = 0.06
            tw.angular.z = self._follow_w * (1.0 if self._right_hand else -1.0)
            tw.angular.z += self._corner_extra * (1.0 if self._right_hand else -1.0)
        elif side is None or side > self._wall_lost:
            # 失去侧向墙：向墙一侧转寻找
            tw.linear.x = 0.10
            tw.angular.z = self._follow_w * 0.55 * lost_turn
        elif side < self._wall_target * 0.55:
            # 离墙太近：略转向外
            tw.linear.x = self._follow_v * 0.65
            tw.angular.z = self._follow_w * 0.45 * close_turn
        elif side > self._wall_target * 1.35:
            # 离墙太远：略转向墙
            tw.linear.x = self._follow_v * 0.85
            tw.angular.z = self._follow_w * 0.35 * (-lost_turn)
        else:
            tw.linear.x = self._follow_v
            tw.angular.z = 0.0

        tw.angular.z = clamp(tw.angular.z, -1.3, 1.3)
        tw.linear.x = clamp(tw.linear.x, 0.0, 0.28)
        self._pub.publish(tw)


def main() -> None:
    rclpy.init()
    node = WallFollowCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
