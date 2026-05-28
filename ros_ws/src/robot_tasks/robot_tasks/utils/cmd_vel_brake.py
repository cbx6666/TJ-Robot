"""导航结束后向 /cmd_vel 发布若干帧零速度，避免 DWB 残余角速度导致缓慢自转。"""

from __future__ import annotations

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.publisher import Publisher


def publish_cmd_vel_brake(
    pub: Publisher,
    *,
    bursts: int = 12,
) -> None:
    stop = Twist()
    for _ in range(max(int(bursts), 1)):
        pub.publish(stop)
