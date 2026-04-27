from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MockPickPlaceNode(Node):
    """Mock arm behavior for demo validation.

    No fine arm control here: we only prove that the pipeline selects
    the correct target object and calls pick/place semantics.
    """

    def __init__(self) -> None:
        super().__init__("mock_pick_place")
        self.declare_parameter("command_topic", "/manipulation/command_text")
        self.declare_parameter("status_topic", "/manipulation/status_text")
        self.declare_parameter("default_object", "unknown_object")

        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._default_object = str(self.get_parameter("default_object").value)

        self._held_object = ""
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)

        self.create_service(Trigger, "/manipulation/mock_pick", self._on_pick)
        self.create_service(Trigger, "/manipulation/mock_place", self._on_place)

        self.get_logger().info(
            f"mock_pick_place ready: command_topic={command_topic}, status_topic={status_topic}"
        )

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _on_pick(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._held_object:
            resp.success = False
            resp.message = f"already holding {self._held_object}"
            return resp
        self._held_object = self._default_object
        resp.success = True
        resp.message = f"picked {self._held_object}"
        self._publish_status(f"[manipulation] {resp.message}")
        return resp

    def _on_place(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if not self._held_object:
            resp.success = False
            resp.message = "nothing to place"
            return resp
        obj = self._held_object
        self._held_object = ""
        resp.success = True
        resp.message = f"placed {obj}"
        self._publish_status(f"[manipulation] {resp.message}")
        return resp

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if text.startswith("PICK:"):
            self._default_object = text.split(":", 1)[1].strip() or self._default_object
            self._on_pick(Trigger.Request(), Trigger.Response())
        elif text.startswith("PLACE"):
            self._on_place(Trigger.Request(), Trigger.Response())
        else:
            self._publish_status(f"[manipulation] ignored command: {text}")


def main() -> None:
    rclpy.init()
    node = MockPickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
