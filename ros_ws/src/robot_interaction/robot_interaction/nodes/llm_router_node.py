from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LlmRouterNode(Node):
    """LLM orchestration placeholder.

    Converts free-form voice text into a task goal text command.
    Future extension:
    - call LLM API
    - parse structured tool/action plans
    - invoke navigation/manipulation capabilities
    """

    def __init__(self) -> None:
        super().__init__("llm_router")
        self.declare_parameter("input_topic", "/interaction/voice_text")
        self.declare_parameter("task_goal_topic", "/task/goal_text")
        self.declare_parameter("plan_topic", "/interaction/llm_plan_text")
        self.declare_parameter(
            "prompt_system",
            "You are a household service robot planner. Convert user intent into concise task goals.",
        )
        self.declare_parameter("llm_api_url", "")
        self.declare_parameter("llm_model", "gpt-4o-mini")

        input_topic = str(self.get_parameter("input_topic").value)
        self._task_goal_topic = str(self.get_parameter("task_goal_topic").value)
        self._plan_topic = str(self.get_parameter("plan_topic").value)
        self._prompt_system = str(self.get_parameter("prompt_system").value)

        self._task_pub = self.create_publisher(String, self._task_goal_topic, 10)
        self._plan_pub = self.create_publisher(String, self._plan_topic, 10)
        self.create_subscription(String, input_topic, self._on_voice_text, 10)

        self.get_logger().info(
            f"llm_router ready: {input_topic} -> {self._task_goal_topic}, "
            f"plan_topic={self._plan_topic}"
        )

    def _on_voice_text(self, msg: String) -> None:
        user_text = msg.data.strip()
        if not user_text:
            return
        # Placeholder plan format: TASK|object|location style string.
        # Replace with API-driven structured planning later.
        task_goal = f"TASK_GOAL:{user_text}"
        plan_text = (
            "PLAN_STEP:parse_intent -> PLAN_STEP:navigate -> "
            "PLAN_STEP:detect_object -> PLAN_STEP:mock_pick"
        )
        self._task_pub.publish(String(data=task_goal))
        self._plan_pub.publish(String(data=plan_text))
        self.get_logger().info(f"[llm-router] goal={task_goal}")


def main() -> None:
    rclpy.init()
    node = LlmRouterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
