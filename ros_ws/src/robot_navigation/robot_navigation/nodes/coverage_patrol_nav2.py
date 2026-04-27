"""Coverage patrol using Nav2 NavigateToPose.

This keeps the existing "hand-authored waypoints" idea, but delegates motion,
local obstacle avoidance, and recovery behaviors to Nav2 (costmaps + controller).
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from robot_navigation.utils.nav_math import (
    Pose2D,
    clamp,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


Waypoint = Tuple[float, float, str]


class CoveragePatrolNav2(Node):
    def __init__(self) -> None:
        super().__init__("coverage_patrol_nav2")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")

        # same route shaping as coverage_patrol
        self.declare_parameter("room_min_x", -7.0)
        self.declare_parameter("room_max_x", 7.0)
        self.declare_parameter("room_min_y", -7.0)
        self.declare_parameter("room_max_y", 7.0)
        self.declare_parameter("route_margin", 0.7)
        self.declare_parameter("edge_margin", 1.35)
        self.declare_parameter("corner_margin", 1.15)
        self.declare_parameter("include_people_passes", True)

        self.declare_parameter("goal_timeout_sec", 35.0)
        self.declare_parameter("server_wait_sec", 20.0)
        # face_next：每个点对准“下一段”方向，易在 Nav2 里触发位置+朝向反复微调（原地晃）。
        # 覆盖巡航默认只关心到达航点位置；朝向用当前朝向即可（配合 YAML 里放宽 yaw_goal_tolerance）。
        self.declare_parameter("final_yaw_mode", "keep")  # none|face_next|keep

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._action_name = str(self.get_parameter("nav2_action_name").value)

        self._room_min_x = float(self.get_parameter("room_min_x").value)
        self._room_max_x = float(self.get_parameter("room_max_x").value)
        self._room_min_y = float(self.get_parameter("room_min_y").value)
        self._room_max_y = float(self.get_parameter("room_max_y").value)
        self._route_margin = float(self.get_parameter("route_margin").value)
        self._edge_margin = float(self.get_parameter("edge_margin").value)
        self._corner_margin = float(self.get_parameter("corner_margin").value)
        self._include_people_passes = bool(self.get_parameter("include_people_passes").value)
        self._goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self._server_wait_sec = float(self.get_parameter("server_wait_sec").value)
        self._final_yaw_mode = str(self.get_parameter("final_yaw_mode").value).strip().lower()

        self._current_pose: Optional[Pose2D] = None
        self._waypoints: List[Waypoint] = []
        self._idx = 0
        self._route_ready = False
        self._done = False

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._client = ActionClient(self, NavigateToPose, self._action_name)
        self._goal_handle = None
        self._deadline_ns: Optional[int] = None

        self._timer = self.create_timer(0.2, self._tick)
        self.get_logger().info(f"coverage_patrol_nav2 started (action=/{self._action_name}, global_frame={self._global_frame})")
        self._boot_ns = self.get_clock().now().nanoseconds
        self._last_wait_log_ns: Optional[int] = None

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._current_pose = Pose2D(
            x=float(p.x),
            y=float(p.y),
            yaw=normalize_angle(yaw_from_quaternion(o.z, o.w)),
        )
        if not self._route_ready:
            self._build_route()

    def _build_route(self) -> None:
        if self._current_pose is None:
            return

        edge_offset = max(self._route_margin, self._edge_margin)
        left = self._room_min_x + edge_offset
        right = self._room_max_x - edge_offset
        bottom = self._room_min_y + edge_offset
        top = self._room_max_y - edge_offset

        corner_offset = max(self._corner_margin, edge_offset)
        left_corner = self._room_min_x + corner_offset
        right_corner = self._room_max_x - corner_offset
        bottom_corner = self._room_min_y + corner_offset
        top_corner = self._room_max_y - corner_offset

        center_x = 0.0
        center_y = 0.0
        upper_lane = 2.6
        lower_lane = -2.6
        entry_y = clamp(self._current_pose.y, bottom + 0.8, top - 0.8)

        route: List[Waypoint] = [
            (left, entry_y, "left edge"),
            (left_corner, bottom_corner, "bottom-left edge"),
            (center_x, bottom, "bottom center"),
            (right_corner, bottom_corner, "bottom-right edge"),
            (right, center_y, "right center"),
            (right_corner, top_corner, "top-right edge"),
            (center_x, top, "top center"),
        ]
        if self._include_people_passes:
            route.extend(
                [
                    (3.2, 3.2, "walking actor lane"),
                    (-5.7, 4.8, "standing person corner"),
                ]
            )
        route.extend(
            [
                (left_corner, top_corner, "top-left edge"),
                (left, center_y, "left center"),
                (right - 1.0, upper_lane, "upper sweep right"),
                (left + 1.0, upper_lane, "upper sweep left"),
                (left + 1.0, lower_lane, "lower sweep left"),
                (right - 1.0, lower_lane, "lower sweep right"),
                (center_x, center_y, "center finish"),
            ]
        )
        self._waypoints = route
        self._route_ready = True
        self.get_logger().info(f"Route ready ({len(self._waypoints)} waypoints).")

    def _goal_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _desired_yaw(self, wp_idx: int) -> float:
        if self._final_yaw_mode in ("none", "no", "off"):
            if self._current_pose is not None:
                return self._current_pose.yaw
            return 0.0
        if self._final_yaw_mode in ("keep", "current") and self._current_pose is not None:
            return self._current_pose.yaw
        # face_next
        if wp_idx + 1 < len(self._waypoints):
            x, y, _n = self._waypoints[wp_idx]
            nx, ny, _nn = self._waypoints[wp_idx + 1]
            return math.atan2(ny - y, nx - x)
        if self._current_pose is not None:
            return self._current_pose.yaw
        return 0.0

    def _send_current_goal(self) -> None:
        x, y, name = self._waypoints[self._idx]
        yaw = self._desired_yaw(self._idx)
        goal = NavigateToPose.Goal()
        goal.pose = self._goal_pose(x, y, yaw)

        self.get_logger().info(f"Nav2 goal -> {name} ({self._idx+1}/{len(self._waypoints)}) ({x:.2f},{y:.2f})")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)
        self._deadline_ns = self.get_clock().now().nanoseconds + int(max(self._goal_timeout_sec, 5.0) * 1e9)

    def _on_goal_accepted(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected goal. Skipping waypoint.")
            self._goal_handle = None
            self._deadline_ns = None
            self._idx += 1
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        res = future.result().result
        status = future.result().status
        _ = res
        name = self._waypoints[self._idx][2] if self._idx < len(self._waypoints) else "unknown"
        self.get_logger().info(f"Nav2 result status={status} for {name}")
        self._goal_handle = None
        self._deadline_ns = None
        self._idx += 1

    def _tick(self) -> None:
        if self._done:
            return
        if not self._route_ready:
            return
        if self._idx >= len(self._waypoints):
            self._done = True
            self.get_logger().info("Coverage route complete (Nav2).")
            return

        if not self._client.server_is_ready():
            now_ns = self.get_clock().now().nanoseconds
            if self._last_wait_log_ns is None or (now_ns - self._last_wait_log_ns) > int(2.0 * 1e9):
                self._last_wait_log_ns = now_ns
                waited = (now_ns - self._boot_ns) / 1e9
                self.get_logger().warning(
                    f"Waiting for Nav2 action server /{self._action_name} ... waited {waited:.1f}s"
                )
            if (now_ns - self._boot_ns) / 1e9 > max(self._server_wait_sec, 1.0):
                self.get_logger().error(
                    f"Nav2 action server /{self._action_name} not ready after {self._server_wait_sec:.1f}s. "
                    "Check bt_navigator lifecycle activation and /tf (map->odom, odom->base_footprint)."
                )
                self._done = True
            return

        if self._goal_handle is None:
            self._send_current_goal()
            return

        if self._deadline_ns is not None and self.get_clock().now().nanoseconds > self._deadline_ns:
            self.get_logger().warning("Nav2 goal timeout; cancel and skip waypoint.")
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
            self._deadline_ns = None
            self._idx += 1


def main() -> None:
    rclpy.init()
    node = CoveragePatrolNav2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

