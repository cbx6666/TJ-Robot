"""导航 waypoint 与位姿转换工具。

这个文件不依赖 Nav2 action，只放通用数据结构和几何函数。这样主导航流程可以专注于
状态机和恢复策略，不需要在业务逻辑里混入 quaternion、角度归一化等细节。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


@dataclass(frozen=True)
class Pose2D:
    """二维位姿。

    yaw 单位为弧度，坐标默认使用 map frame。
    """

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    """巡航目标点。

    name 用于日志和 marker；x/y/yaw 用于生成 PoseStamped 或候选 approach goal。
    """

    name: str
    x: float
    y: float
    yaw: float = 0.0

    def pose2d(self) -> Pose2D:
        """转换成轻量 Pose2D。"""

        return Pose2D(self.x, self.y, self.yaw)


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]。"""

    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z: float, w: float) -> float:
    """从 2D 场景常用的 quaternion z/w 分量恢复 yaw。"""

    return normalize_angle(2.0 * math.atan2(z, w))


def make_pose_stamped(node: Node, waypoint: Waypoint, frame_id: str = "map") -> PoseStamped:
    """把 Waypoint 转成 Nav2 使用的 PoseStamped。"""

    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0

    half_yaw = waypoint.yaw * 0.5
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose
