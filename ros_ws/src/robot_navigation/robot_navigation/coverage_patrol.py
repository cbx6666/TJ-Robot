"""Deterministic coverage patrol for the current Gazebo room.

This node keeps raw /scan for local obstacle avoidance. When YOLO is enabled,
slam_toolbox can use /scan_filtered in parallel for cleaner mapping.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


Waypoint = Tuple[float, float, str]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class CoveragePatrol(Node):
    """Drive a hand-authored room coverage route with lightweight recovery logic.

    The stack may map with /scan_filtered, but this node intentionally keeps
    raw /scan so nearby people and dynamic obstacles are still treated as
    collision risks.
    """

    def __init__(self) -> None:
        super().__init__('coverage_patrol')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('room_min_x', -7.0)
        self.declare_parameter('room_max_x', 7.0)
        self.declare_parameter('room_min_y', -7.0)
        self.declare_parameter('room_max_y', 7.0)
        self.declare_parameter('route_margin', 0.7)
        self.declare_parameter('edge_margin', 1.35)
        self.declare_parameter('corner_margin', 1.15)
        self.declare_parameter('escape_step', 1.0)
        self.declare_parameter('detour_step', 1.4)
        self.declare_parameter('progress_epsilon', 0.12)
        self.declare_parameter('goal_stuck_sec', 4.0)
        self.declare_parameter('position_tolerance', 0.22)
        self.declare_parameter('heading_tolerance', 0.38)
        self.declare_parameter('linear_speed_limit', 0.30)
        self.declare_parameter('angular_speed_limit', 1.4)
        self.declare_parameter('linear_kp', 1.2)
        self.declare_parameter('angular_kp', 3.0)
        self.declare_parameter('drive_heading_kp', 1.6)
        self.declare_parameter('obstacle_distance', 0.50)
        self.declare_parameter('blocked_goal_distance', 0.45)
        self.declare_parameter('avoidance_stuck_sec', 3.0)
        self.declare_parameter('avoid_turn_speed', 0.9)
        self.declare_parameter('reverse_speed', 0.10)
        self.declare_parameter('include_people_passes', True)

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._room_min_x = float(self.get_parameter('room_min_x').value)
        self._room_max_x = float(self.get_parameter('room_max_x').value)
        self._room_min_y = float(self.get_parameter('room_min_y').value)
        self._room_max_y = float(self.get_parameter('room_max_y').value)
        self._route_margin = float(self.get_parameter('route_margin').value)
        self._edge_margin = float(self.get_parameter('edge_margin').value)
        self._corner_margin = float(self.get_parameter('corner_margin').value)
        self._escape_step = float(self.get_parameter('escape_step').value)
        self._detour_step = float(self.get_parameter('detour_step').value)
        self._progress_epsilon = float(self.get_parameter('progress_epsilon').value)
        self._goal_stuck_sec = float(self.get_parameter('goal_stuck_sec').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        self._linear_speed_limit = float(self.get_parameter('linear_speed_limit').value)
        self._angular_speed_limit = float(self.get_parameter('angular_speed_limit').value)
        self._linear_kp = float(self.get_parameter('linear_kp').value)
        self._angular_kp = float(self.get_parameter('angular_kp').value)
        self._drive_heading_kp = float(self.get_parameter('drive_heading_kp').value)
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._blocked_goal_distance = float(self.get_parameter('blocked_goal_distance').value)
        self._avoidance_stuck_sec = float(self.get_parameter('avoidance_stuck_sec').value)
        self._avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self._reverse_speed = float(self.get_parameter('reverse_speed').value)
        self._include_people_passes = bool(self.get_parameter('include_people_passes').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self._current_pose: Optional[Pose2D] = None
        self._last_scan: Optional[LaserScan] = None
        self._waypoints: List[Waypoint] = []
        self._current_index = 0
        self._route_ready = False
        self._completed = False
        self._last_status = ''
        self._avoid_goal_index = -1
        self._avoid_start_ns: Optional[int] = None
        self._progress_goal_index = -1
        self._best_goal_distance = float('inf')
        self._last_progress_ns: Optional[int] = None

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self._timer = self.create_timer(1.0 / control_rate_hz, self._on_timer)

        self.get_logger().info(
            'Coverage patrol started '
            f'(odom={self._odom_topic}, scan={self._scan_topic}, cmd_vel={self._cmd_vel_topic})'
        )

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._current_pose = Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=normalize_angle(yaw_from_quaternion(orientation.z, orientation.w)),
        )
        if not self._route_ready:
            self._build_route()

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _build_route(self) -> None:
        if self._current_pose is None:
            return

        # Keep wall-facing waypoints inset from the room boundary so the robot
        # scans the perimeter without scraping along the wall mesh.
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
            (left, entry_y, 'left edge'),
            (left_corner, bottom_corner, 'bottom-left edge'),
            (center_x, bottom, 'bottom center'),
            (right_corner, bottom_corner, 'bottom-right edge'),
            (right, center_y, 'right center'),
            (right_corner, top_corner, 'top-right edge'),
            (center_x, top, 'top center'),
        ]

        if self._include_people_passes:
            route.extend([
                (3.2, 3.2, 'walking actor lane'),
                (-5.7, 4.8, 'standing person corner'),
            ])

        route.extend([
            (left_corner, top_corner, 'top-left edge'),
            (left, center_y, 'left center'),
            (right - 1.0, upper_lane, 'upper sweep right'),
            (left + 1.0, upper_lane, 'upper sweep left'),
            (left + 1.0, lower_lane, 'lower sweep left'),
            (right - 1.0, lower_lane, 'lower sweep right'),
            (center_x, center_y, 'center finish'),
        ])

        self._waypoints = route
        self._route_ready = True
        self.get_logger().info(f'Coverage route ready ({len(self._waypoints)} waypoints).')

    def _on_timer(self) -> None:
        if self._completed:
            self._publish_stop()
            return

        if self._current_pose is None:
            self._set_status('Waiting for /odom')
            return

        if not self._route_ready:
            self._build_route()
            if not self._route_ready:
                self._set_status('Waiting for initial pose to build coverage route')
                return

        if self._current_index >= len(self._waypoints):
            self._completed = True
            self._publish_stop()
            self.get_logger().info('Coverage route complete.')
            return

        now_ns = self.get_clock().now().nanoseconds
        goal_x, goal_y, goal_name = self._waypoints[self._current_index]
        dx = goal_x - self._current_pose.x
        dy = goal_y - self._current_pose.y
        distance = math.hypot(dx, dy)

        # Track the best distance reached for the active waypoint. Recovery uses
        # this instead of a pure avoidance timer so heading/avoidance oscillation
        # cannot hide a lack of real progress.
        if self._progress_goal_index != self._current_index:
            self._progress_goal_index = self._current_index
            self._best_goal_distance = distance
            self._last_progress_ns = now_ns
        elif distance < self._best_goal_distance - self._progress_epsilon:
            self._best_goal_distance = distance
            self._last_progress_ns = now_ns

        if distance <= self._position_tolerance:
            self._advance_waypoint(goal_name)
            return

        if self._goal_blocked_but_close(distance):
            self._advance_waypoint(goal_name, reason='blocked but close to waypoint')
            return

        if self._maybe_recover_from_no_progress(goal_name, distance, now_ns):
            return

        avoidance = self._avoidance_command(goal_name, distance)
        if avoidance is not None:
            if self._avoid_goal_index != self._current_index:
                self._avoid_goal_index = self._current_index
                self._avoid_start_ns = now_ns
            elif self._avoid_start_ns is None:
                self._avoid_start_ns = now_ns
            elif (now_ns - self._avoid_start_ns) / 1e9 >= self._avoidance_stuck_sec:
                if distance <= max(self._blocked_goal_distance + 0.2, 0.8):
                    self._advance_waypoint(goal_name, reason='stuck near waypoint')
                    return
                if self._maybe_recover_from_no_progress(goal_name, distance, now_ns):
                    return

            self._set_status(f'Avoiding obstacle on way to {goal_name}')
            self._cmd_pub.publish(avoidance)
            return

        self._avoid_goal_index = -1
        self._avoid_start_ns = None

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - self._current_pose.yaw)
        command = Twist()
        if abs(heading_error) > self._heading_tolerance:
            command.angular.z = clamp(
                self._angular_kp * heading_error,
                -self._angular_speed_limit,
                self._angular_speed_limit,
            )
        else:
            command.linear.x = clamp(
                self._linear_kp * distance,
                0.0,
                self._linear_speed_limit,
            )
            command.angular.z = clamp(
                self._drive_heading_kp * heading_error,
                -self._angular_speed_limit,
                self._angular_speed_limit,
            )

        self._set_status(
            f'Heading to {goal_name} ({self._current_index + 1}/{len(self._waypoints)}) '
            f'goal=({goal_x:.2f}, {goal_y:.2f}) distance={distance:.2f}m'
        )
        self._cmd_pub.publish(command)

    def _avoidance_command(self, goal_name: str, distance: float) -> Optional[Twist]:
        front = self._sector_min(-22.0, 22.0)
        if front is None or front >= self._obstacle_distance:
            return None

        left = self._sector_min(30.0, 110.0)
        right = self._sector_min(-110.0, -30.0)
        left_blocked = left is not None and left < self._obstacle_distance
        right_blocked = right is not None and right < self._obstacle_distance

        # Recovery waypoints should back out from the wall instead of trying to
        # squeeze forward through the same narrow slot again.
        prefer_reverse = (
            goal_name.startswith('escape from')
            or goal_name.startswith('detour from')
            or distance <= max(self._blocked_goal_distance + 0.25, 0.7)
        )

        command = Twist()
        if prefer_reverse or (left_blocked and right_blocked):
            command.linear.x = -self._reverse_speed
            if left is None and right is None:
                command.angular.z = self._avoid_turn_speed
            elif right is None or (left is not None and left >= right):
                command.angular.z = self._avoid_turn_speed
            else:
                command.angular.z = -self._avoid_turn_speed
            return command

        if left is None and right is None:
            command.angular.z = self._avoid_turn_speed
        elif right is None or (left is not None and left >= right):
            command.angular.z = self._avoid_turn_speed
        else:
            command.angular.z = -self._avoid_turn_speed
        return command

    def _goal_blocked_but_close(self, distance: float) -> bool:
        if distance > self._blocked_goal_distance:
            return False
        front = self._sector_min(-18.0, 18.0)
        return front is not None and front < self._obstacle_distance

    def _maybe_recover_from_no_progress(self, goal_name: str, distance: float, now_ns: int) -> bool:
        if self._last_progress_ns is None:
            return False

        no_progress_sec = (now_ns - self._last_progress_ns) / 1e9
        if no_progress_sec < self._goal_stuck_sec:
            return False

        # First try a detour back toward room center. If even a recovery waypoint
        # makes no progress, skip it instead of cycling forever.
        if goal_name.startswith('escape from') or goal_name.startswith('detour from'):
            self._advance_waypoint(goal_name, reason='recovery waypoint made no progress')
            return True

        detour_waypoint = self._make_detour_waypoint(goal_name)
        if detour_waypoint is None:
            self._advance_waypoint(goal_name, reason='no progress toward waypoint')
            return True

        self._waypoints.insert(self._current_index, detour_waypoint)
        self._avoid_goal_index = -1
        self._avoid_start_ns = None
        self._progress_goal_index = -1
        self._best_goal_distance = float('inf')
        self._last_progress_ns = None
        self._publish_stop()
        self.get_logger().warning(
            f'Inserted {detour_waypoint[2]} before {goal_name} after {no_progress_sec:.1f}s without progress'
        )
        return True

    def _advance_waypoint(self, goal_name: str, reason: Optional[str] = None) -> None:
        goal_x, goal_y, _ = self._waypoints[self._current_index]
        self._current_index += 1
        self._avoid_goal_index = -1
        self._avoid_start_ns = None
        self._progress_goal_index = -1
        self._best_goal_distance = float('inf')
        self._last_progress_ns = None
        self._publish_stop()

        if reason is not None and self._current_pose is not None:
            escape_waypoint = self._make_escape_waypoint(goal_x, goal_y, goal_name)
            if escape_waypoint is not None:
                self._waypoints.insert(self._current_index, escape_waypoint)

        if self._current_index < len(self._waypoints):
            next_goal = self._waypoints[self._current_index]
            if reason is None:
                self.get_logger().info(
                    f'Reached {goal_name}; next={next_goal[2]} ({self._current_index + 1}/{len(self._waypoints)})'
                )
            else:
                self.get_logger().warning(
                    f'Advancing past {goal_name} ({reason}); next={next_goal[2]} ({self._current_index + 1}/{len(self._waypoints)})'
                )
        else:
            self.get_logger().info('Reached final waypoint.')

    def _make_detour_waypoint(self, goal_name: str) -> Optional[Waypoint]:
        if self._current_pose is None:
            return None

        center_dx = -self._current_pose.x
        center_dy = -self._current_pose.y
        norm = math.hypot(center_dx, center_dy)
        if norm < 1e-6:
            return None

        x = self._current_pose.x + center_dx / norm * self._detour_step
        y = self._current_pose.y + center_dy / norm * self._detour_step
        x = clamp(x, self._room_min_x + self._route_margin + 0.5, self._room_max_x - self._route_margin - 0.5)
        y = clamp(y, self._room_min_y + self._route_margin + 0.5, self._room_max_y - self._route_margin - 0.5)

        if math.hypot(x - self._current_pose.x, y - self._current_pose.y) < 0.3:
            return None
        return (x, y, f'detour from {goal_name}')

    def _make_escape_waypoint(self, goal_x: float, goal_y: float, goal_name: str) -> Optional[Waypoint]:
        if self._current_pose is None:
            return None

        x = self._current_pose.x
        y = self._current_pose.y
        boundary_band = self._corner_margin + 0.35

        if goal_x >= self._room_max_x - boundary_band:
            x = min(x, self._room_max_x - (self._route_margin + self._escape_step))
        elif goal_x <= self._room_min_x + boundary_band:
            x = max(x, self._room_min_x + (self._route_margin + self._escape_step))

        if goal_y >= self._room_max_y - boundary_band:
            y = min(y, self._room_max_y - (self._route_margin + self._escape_step))
        elif goal_y <= self._room_min_y + boundary_band:
            y = max(y, self._room_min_y + (self._route_margin + self._escape_step))

        x = clamp(x, self._room_min_x + self._route_margin + 0.5, self._room_max_x - self._route_margin - 0.5)
        y = clamp(y, self._room_min_y + self._route_margin + 0.5, self._room_max_y - self._route_margin - 0.5)

        if math.hypot(x - self._current_pose.x, y - self._current_pose.y) < 0.25:
            return None
        return (x, y, f'escape from {goal_name}')

    def _sector_min(self, min_deg: float, max_deg: float) -> Optional[float]:
        if self._last_scan is None:
            return None

        min_rad = math.radians(min_deg)
        max_rad = math.radians(max_deg)
        angle = float(self._last_scan.angle_min)
        best = float('inf')
        found = False
        for sample in self._last_scan.ranges:
            normalized = normalize_angle(angle)
            value = float(sample)
            if min_rad <= normalized <= max_rad and math.isfinite(value):
                if self._last_scan.range_min <= value <= self._last_scan.range_max:
                    best = min(best, value)
                    found = True
            angle += float(self._last_scan.angle_increment)

        if not found:
            return None
        return best

    def _publish_stop(self) -> None:
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def _set_status(self, status: str) -> None:
        if status != self._last_status:
            self._last_status = status
            self.get_logger().info(status)

    def destroy_node(self) -> bool:
        self._publish_stop()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CoveragePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

