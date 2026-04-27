from __future__ import annotations

import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceGatewayNode(Node):
    """Voice interface placeholder.

    Current behavior:
    - Optional mock mode periodically publishes canned voice text.
    - Future behavior:
      - connect to external ASR API and publish recognized utterances.
    """

    def __init__(self) -> None:
        super().__init__("voice_gateway")
        self.declare_parameter("output_topic", "/interaction/voice_text")
        self.declare_parameter("enable_mock_input", True)
        self.declare_parameter("mock_interval_sec", 8.0)
        self.declare_parameter("mock_text", "去客厅拿杯子")
        self.declare_parameter("voice_api_url", "")
        self.declare_parameter("voice_api_key_env", "VOICE_API_KEY")

        self._output_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(String, self._output_topic, 10)

        self._api_url = str(self.get_parameter("voice_api_url").value).strip()
        api_key_env = str(self.get_parameter("voice_api_key_env").value).strip()
        self._api_key = os.environ.get(api_key_env, "")

        enable_mock = bool(self.get_parameter("enable_mock_input").value)
        self._mock_text = str(self.get_parameter("mock_text").value)
        interval = max(float(self.get_parameter("mock_interval_sec").value), 1.0)

        if self._api_url:
            self.get_logger().info(
                f"voice_gateway configured for external API: {self._api_url} "
                f"(key_present={int(bool(self._api_key))})"
            )
        if enable_mock:
            self.create_timer(interval, self._publish_mock_text)
            self.get_logger().info(
                f"voice_gateway mock mode enabled -> {self._output_topic}, every {interval}s"
            )
        else:
            self.get_logger().info(f"voice_gateway ready -> {self._output_topic} (mock disabled)")

    def _publish_mock_text(self) -> None:
        self._pub.publish(String(data=self._mock_text))
        self.get_logger().info(f"[mock voice] {self._mock_text}")


def main() -> None:
    rclpy.init()
    node = VoiceGatewayNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
