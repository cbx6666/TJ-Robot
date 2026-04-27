from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from robot_interfaces.msg import TaskStatus
except ImportError:  # ROBOT_INTERFACES_ENABLE_ROSIDL=OFF 或接口尚未构建
    TaskStatus = None  # type: ignore[assignment]


class TaskManagerNode(Node):
    """Minimal task manager placeholder for system bringup.

    Future work: subscribe to parsed intents, dispatch navigation/search/mapping
    tasks, and publish task status.
    """

    def __init__(self) -> None:
        super().__init__("task_manager")
        self.declare_parameter("event_topic", "/task/events")
        self.declare_parameter("status_topic", "/task/status")
        self.declare_parameter("goal_text_topic", "/task/goal_text")
        self.declare_parameter("manipulation_command_topic", "/manipulation/command_text")
        self.declare_parameter("heartbeat_sec", 2.0)
        self.declare_parameter("task_id", "bootstrap")

        self._status_topic = str(self.get_parameter("status_topic").value)
        self._task_id = str(self.get_parameter("task_id").value)
        self._goal_text_topic = str(self.get_parameter("goal_text_topic").value)
        self._mani_command_topic = str(self.get_parameter("manipulation_command_topic").value)
        heartbeat_sec = max(float(self.get_parameter("heartbeat_sec").value), 0.2)

        self._status_pub = None
        self._status_str_pub = self.create_publisher(String, f"{self._status_topic}_text", 10)
        self._mani_cmd_pub = self.create_publisher(String, self._mani_command_topic, 10)
        self.create_subscription(String, self._goal_text_topic, self._on_goal_text, 10)
        if TaskStatus is not None:
            self._status_pub = self.create_publisher(TaskStatus, self._status_topic, 10)
            self.get_logger().info(
                f"task_manager ready: publishing TaskStatus -> {self._status_topic}"
            )
        else:
            self.get_logger().warning(
                "task_manager: robot_interfaces messages unavailable, using String fallback "
                f"-> {self._status_topic}_text"
            )

        self.create_timer(heartbeat_sec, self._publish_heartbeat)
        self.get_logger().info(
            f"task_manager subscriptions: goal={self._goal_text_topic}; "
            f"outputs manipulation={self._mani_command_topic}"
        )

    def _publish_heartbeat(self) -> None:
        detail = "task_manager alive (placeholder dispatcher)"
        self._status_str_pub.publish(String(data=f"{self._task_id}:RUNNING:{detail}"))
        if self._status_pub is None:
            return
        msg = TaskStatus()
        msg.task_id = self._task_id
        msg.state = "RUNNING"
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    def _publish_status_event(self, state: str, detail: str) -> None:
        self._status_str_pub.publish(String(data=f"{self._task_id}:{state}:{detail}"))
        if self._status_pub is None:
            return
        msg = TaskStatus()
        msg.task_id = self._task_id
        msg.state = state
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    def _on_goal_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        # Minimal rule-based orchestration scaffold:
        # focus on object pick/place semantics for RGBD + YOLO stage.
        obj = self._infer_object(text)
        if obj:
            self._mani_cmd_pub.publish(String(data=f"PICK:{obj}"))
        self._publish_status_event("EXECUTING", f"goal={text} object={obj or 'NA'}")

    @staticmethod
    def _infer_object(text: str) -> str:
        objects = ["杯子", "瓶子", "遥控器", "纸巾", "书"]
        for obj in objects:
            if obj in text:
                return obj
        return ""


def main() -> None:
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
