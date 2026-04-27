from __future__ import annotations

import rclpy
from rclpy.node import Node


class TaskManagerNode(Node):
    """Minimal task manager placeholder for system bringup.

    Future work: subscribe to parsed intents, dispatch navigation/search/mapping
    tasks, and publish task status.
    """

    def __init__(self) -> None:
        super().__init__("task_manager")
        self.declare_parameter("event_topic", "/task/events")
        self.declare_parameter("status_topic", "/task/status")
        self.get_logger().info("task_manager is ready (placeholder dispatcher).")


def main() -> None:
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
