"""RViz 巡视诊断 Marker。

发布当前目标、已完成点、跳过点和卡死位置，便于在 RViz 中直接看到
巡航状态，而不是只翻日志。
"""

from __future__ import annotations

from dataclasses import dataclass

from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_utils import Pose2D, Waypoint


@dataclass(frozen=True)
class _Color:
    r: float
    g: float
    b: float
    a: float = 1.0


class PatrolMarkerPublisher:
    def __init__(self, node: Node, topic: str = "/navigation/patrol_markers") -> None:
        self._node = node
        self._pub = node.create_publisher(MarkerArray, topic, 10)
        self._seq = 0

    def publish_goal(self, waypoint: Waypoint) -> None:
        marker = self._sphere("current_goal", 1, waypoint.x, waypoint.y, _Color(0.1, 0.4, 1.0))
        marker.scale.x = 0.32
        marker.scale.y = 0.32
        marker.scale.z = 0.32
        self._pub.publish(MarkerArray(markers=[marker, self._label("goal_label", 2, waypoint)]))

    def publish_done(self, waypoint: Waypoint) -> None:
        self._pub.publish(
            MarkerArray(markers=[self._sphere("done", self._next_id(), waypoint.x, waypoint.y, _Color(0.0, 0.8, 0.2))])
        )

    def publish_skipped(self, waypoint: Waypoint) -> None:
        self._pub.publish(
            MarkerArray(markers=[self._sphere("skipped", self._next_id(), waypoint.x, waypoint.y, _Color(1.0, 0.2, 0.1))])
        )

    def publish_stuck(self, pose: Pose2D | None, reason: str) -> None:
        if pose is None:
            return
        marker = self._sphere("stuck", self._next_id(), pose.x, pose.y, _Color(1.0, 0.55, 0.0))
        label = self._text("stuck_text", self._next_id(), pose.x, pose.y, reason, _Color(1.0, 0.55, 0.0))
        self._pub.publish(MarkerArray(markers=[marker, label]))

    def _sphere(self, namespace: str, marker_id: int, x: float, y: float, color: _Color) -> Marker:
        marker = self._base(namespace, marker_id, Marker.SPHERE, color)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        return marker

    def _label(self, namespace: str, marker_id: int, waypoint: Waypoint) -> Marker:
        return self._text(namespace, marker_id, waypoint.x, waypoint.y, waypoint.name, _Color(0.1, 0.4, 1.0))

    def _text(
        self,
        namespace: str,
        marker_id: int,
        x: float,
        y: float,
        text: str,
        color: _Color,
    ) -> Marker:
        marker = self._base(namespace, marker_id, Marker.TEXT_VIEW_FACING, color)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.45
        marker.scale.z = 0.22
        marker.text = text[:80]
        return marker

    def _base(self, namespace: str, marker_id: int, marker_type: int, color: _Color) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.color.r = color.r
        marker.color.g = color.g
        marker.color.b = color.b
        marker.color.a = color.a
        return marker

    def _next_id(self) -> int:
        self._seq += 1
        return self._seq
