from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PointStamped
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
        self.declare_parameter("target_point_topic", "/yolo_objects/target_point_map")
        self.declare_parameter("target_label_topic", "/yolo_objects/target_label")
        self.declare_parameter("heartbeat_sec", 2.0)
        self.declare_parameter("task_id", "bootstrap")

        self._status_topic = str(self.get_parameter("status_topic").value)
        self._task_id = str(self.get_parameter("task_id").value)
        self._goal_text_topic = str(self.get_parameter("goal_text_topic").value)
        self._mani_command_topic = str(self.get_parameter("manipulation_command_topic").value)
        self._target_point_topic = str(self.get_parameter("target_point_topic").value)
        self._target_label_topic = str(self.get_parameter("target_label_topic").value)
        heartbeat_sec = max(float(self.get_parameter("heartbeat_sec").value), 0.2)
        self._latest_target_point: PointStamped | None = None
        self._latest_target_label = ""

        self._status_pub = None
        self._status_str_pub = self.create_publisher(String, f"{self._status_topic}_text", 10)
        self._mani_cmd_pub = self.create_publisher(String, self._mani_command_topic, 10)
        event_topic = str(self.get_parameter("event_topic").value)
        self.create_subscription(String, event_topic, self._on_task_event, 10)
        self.create_subscription(String, self._goal_text_topic, self._on_goal_text, 10)
        self.create_subscription(PointStamped, self._target_point_topic, self._on_target_point, 10)
        self.create_subscription(String, self._target_label_topic, self._on_target_label, 10)
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
            f"task_manager subscriptions: events={event_topic}; goal={self._goal_text_topic}; "
            f"target={self._target_point_topic}; outputs manipulation={self._mani_command_topic}"
        )

    def _on_task_event(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        self.get_logger().info(f"[task_manager] event: {raw}")
        try:
            ev = json.loads(raw)
            if isinstance(ev, dict) and ev.get("event") == "patrol_start":
                self._publish_status_event("EXECUTING", f"patrol_scope={ev.get('scope', '')}")
            elif isinstance(ev, dict) and ev.get("event") == "navigate_done":
                self._publish_status_event("DONE" if ev.get("ok") else "FAILED", raw[:200])
        except (json.JSONDecodeError, TypeError):
            pass

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
        coord = self._latest_target_coord_text()
        if obj:
            if coord:
                self._mani_cmd_pub.publish(String(data=f"PICK:{obj};{coord}"))
            else:
                self._mani_cmd_pub.publish(String(data=f"PICK:{obj}"))
        self._publish_status_event(
            "EXECUTING", f"goal={text} object={obj or 'NA'} target={coord or 'NA'}"
        )

    def _on_target_point(self, msg: PointStamped) -> None:
        self._latest_target_point = msg

    def _on_target_label(self, msg: String) -> None:
        self._latest_target_label = msg.data.strip()

    def _latest_target_coord_text(self) -> str:
        if self._latest_target_point is None:
            return ""
        pt = self._latest_target_point.point
        frame = self._latest_target_point.header.frame_id or "map"
        return f"target_frame={frame};target_xyz={pt.x:.3f},{pt.y:.3f},{pt.z:.3f}"

    @staticmethod
    def _infer_object(text: str) -> str:
        low = text.lower()
        en_to_zh = {
            "chair": "椅子",
            "cup": "杯子",
            "bottle": "瓶子",
            "book": "书",
            "cell phone": "手机",
        }
        for en, zh in en_to_zh.items():
            if en in low:
                return zh
        objects = ["杯子", "瓶子", "遥控器", "纸巾", "书", "手机"]
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
