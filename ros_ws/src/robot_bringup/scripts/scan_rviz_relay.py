#!/usr/bin/env python3
"""Subscribe /scan with sensor QoS (Best Effort), republish as /scan_rviz with Reliable QoS.

Gazebo libgazebo_ros_ray_sensor 使用 SensorDataQoS；部分 RViz2 配置下对 /scan 的订阅仍可能收不到数据（状态为 0 points from 0 messages）。
Nav2 / AMCL 等仍直接订阅 Gazebo 的 /scan；本节点仅多一路给 RViz。"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRvizRelay(Node):
    def __init__(self) -> None:
        super().__init__("scan_rviz_relay")
        self.declare_parameter("in_topic", "/scan")
        self.declare_parameter("out_topic", "/scan_rviz")
        in_topic = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic = self.get_parameter("out_topic").get_parameter_value().string_value
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self._pub = self.create_publisher(LaserScan, out_topic, pub_qos)
        self.create_subscription(LaserScan, in_topic, self._cb, qos_profile_sensor_data)
        self.get_logger().info(f"{in_topic} (sensor QoS) -> {out_topic} (reliable) for RViz")

    def _cb(self, msg: LaserScan) -> None:
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ScanRvizRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
