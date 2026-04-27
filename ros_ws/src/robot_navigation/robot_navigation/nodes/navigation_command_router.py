from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavigationCommandRouter(Node):
    """Receives high-level navigation text commands.

    This node is intentionally thin. It defines the contract boundary between
    task orchestration and concrete navigation implementations.
    """

    def __init__(self) -> None:
        super().__init__("navigation_command_router")
        self.declare_parameter("command_topic", "/navigation/command_text")
        self.declare_parameter("status_topic", "/navigation/status_text")
        self.declare_parameter("default_strategy", "coverage_patrol_nav2")

        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._strategy = str(self.get_parameter("default_strategy").value)

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.get_logger().info(
            f"navigation_command_router ready: command_topic={command_topic}, strategy={self._strategy}"
        )

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        status = f"[navigation] strategy={self._strategy} command={text}"
        self._status_pub.publish(String(data=status))
        self.get_logger().info(status)


def main() -> None:
    rclpy.init()
    node = NavigationCommandRouter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
