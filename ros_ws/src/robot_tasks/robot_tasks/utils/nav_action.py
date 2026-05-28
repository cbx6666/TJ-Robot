"""Nav2 NavigateToPose 辅助（等待动作服务、刹停）。"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


def wait_for_navigate_to_pose(
    node: Node,
    client: ActionClient,
    *,
    timeout_sec: float = 120.0,
    log=None,
) -> bool:
    deadline = time.monotonic() + max(timeout_sec, 1.0)
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_server(timeout_sec=1.0):
            return True
        if log is not None:
            log.info("等待 NavigateToPose 动作服务（Nav2 可能仍在启动）…")
        rclpy.spin_once(node, timeout_sec=0.1)
    return False
