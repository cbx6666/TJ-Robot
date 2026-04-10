"""几何与姿态工具（无 ROS 依赖），供各导航节点复用。"""

from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    yaw = normalize_angle(yaw)
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
