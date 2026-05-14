#!/usr/bin/env python3
"""从文件周期性发布 std_msgs/String 到 /robot_description。

tb3_stack 原先用 shell 拼装 ros2 topic pub 参数，整条 URDF 易触发 ARG_MAX/转义破坏，
RViz RobotModel「Description Source: Topic」会报 Model parsing failed。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class Pub(Node):
    def __init__(self, path: str, hz: float) -> None:
        super().__init__("robot_description_string_pub")
        pf = Path(path)
        if not pf.is_file():
            raise ValueError(f"URDF does not exist: {pf}")
        self._txt = pf.read_text(encoding="utf-8")
        self._pub = self.create_publisher(String, "/robot_description", _qos())
        self.get_logger().info(
            f"Publishing URDF ({len(self._txt)} chars) from {pf} at {hz:.2g} Hz on /robot_description"
        )
        dt = max(1.0 / hz, 0.05)
        self.create_timer(dt, self._tick)
        self._tick()

    def _tick(self) -> None:
        m = String()
        m.data = self._txt
        self._pub.publish(m)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf_file", help="展开的 robot URDF（绝对或相对路径）")
    ap.add_argument("--hz", type=float, default=2.0, help="发布频率")
    ns = ap.parse_args()

    hz = ns.hz if ns.hz > 0.0 else 2.0
    rclpy.init()
    try:
        node = Pub(ns.urdf_file, hz)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
