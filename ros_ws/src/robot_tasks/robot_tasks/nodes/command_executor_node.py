from __future__ import annotations

import json
import math
import threading
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from std_msgs.msg import String


class CommandExecutorNode(Node):
    """订阅 ``parsed_intent_topic`` 上的 JSON（由 llm_router 发布），执行 Nav2 导航与任务事件。"""

    def __init__(self) -> None:
        super().__init__("command_executor")
        self.declare_parameter("parsed_intent_topic", "/interaction/parsed_intent")
        self.declare_parameter("task_events_topic", "/task/events")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")

        self._events_topic = str(self.get_parameter("task_events_topic").value)
        action_name = str(self.get_parameter("navigate_to_pose_action").value).strip() or "navigate_to_pose"

        self._events_pub = self.create_publisher(String, self._events_topic, 10)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._goal_lock = threading.Lock()
        self._current_goal_handle: ClientGoalHandle | None = None

        topic = str(self.get_parameter("parsed_intent_topic").value)
        self.create_subscription(String, topic, self._on_parsed_intent, 10)
        self.get_logger().info(
            f"[command_executor] intent={topic} events={self._events_topic} "
            f"NavigateToPose action={action_name!r}"
        )

    def _on_parsed_intent(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        self.get_logger().info(f"[command_executor] 收到 parsed_intent: {raw}")
        # 必须在 executor 线程内调度 action client，避免 rclpy 非线程安全用法。
        self._dispatch(raw)

    def _dispatch(self, raw: str) -> None:
        try:
            obj = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[command_executor] JSON 解析失败: {e}")
            return
        if not isinstance(obj, dict):
            self.get_logger().warning("[command_executor] 顶层不是对象，忽略")
            return
        cmd = str(obj.get("command", "noop")).strip()
        args = obj.get("args") if isinstance(obj.get("args"), dict) else {}
        self.get_logger().info(f"[command_executor] 执行 command={cmd!r} args={args!r}")

        if cmd == "noop":
            return
        if cmd == "stop":
            self._cancel_navigation()
            self._publish_event({"event": "stop", "source": "command_executor"})
            return
        if cmd == "start_room_patrol":
            self._publish_event(
                {
                    "event": "patrol_start",
                    "scope": str((args or {}).get("patrol_scope", "room_default")),
                    "payload": obj,
                }
            )
            return
        if cmd == "fetch_object":
            self._publish_event({"event": "fetch_object", "args": args, "payload": obj})
            return
        if cmd == "navigate_to_pose":
            self._send_nav_goal(args or {})
            return

        self.get_logger().warning(f"[command_executor] 未实现的 command: {cmd!r}")

    def _publish_event(self, data: dict[str, Any]) -> None:
        try:
            s = json.dumps(data, ensure_ascii=False)
        except Exception as e:
            self.get_logger().error(f"[command_executor] 事件序列化失败: {e}")
            return
        self._events_pub.publish(String(data=s))
        self.get_logger().info(f"[command_executor] -> {self._events_topic}: {s}")

    def _cancel_navigation(self) -> None:
        with self._goal_lock:
            gh = self._current_goal_handle
        if gh is None:
            self.get_logger().info("[command_executor] stop：当前无进行中的导航目标")
            return
        self.get_logger().info("[command_executor] 请求取消导航目标 …")
        fut = self._nav_client.cancel_goal_async(gh)
        fut.add_done_callback(lambda f: self.get_logger().info("[command_executor] 取消请求已发出"))

    def _send_nav_goal(self, args: dict[str, Any]) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("[command_executor] NavigateToPose 动作服务不可用（Nav2 是否已启动？）")
            return

        frame = str(args.get("frame_id", "map") or "map")
        x = float(args.get("x", 0.0))
        y = float(args.get("y", 0.0))
        yaw_deg = float(args.get("yaw_deg", 0.0))
        yaw = math.radians(yaw_deg)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.get_logger().info(
            f"[command_executor] 发送 NavigateToPose frame={frame} x={x:.3f} y={y:.3f} yaw_deg={yaw_deg:.1f}"
        )

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future: Any) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("[command_executor] 导航目标被拒绝")
            return
        with self._goal_lock:
            self._current_goal_handle = goal_handle
        self.get_logger().info("[command_executor] 导航目标已接受，等待结果 …")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future: Any) -> None:
        with self._goal_lock:
            self._current_goal_handle = None
        try:
            wrapper = future.result()
            status = int(wrapper.status)
            res = wrapper.result
        except Exception as e:
            self.get_logger().error(f"[command_executor] 获取导航结果异常: {e}")
            return
        ok = status == GoalStatus.STATUS_SUCCEEDED
        self.get_logger().info(
            f"[command_executor] 导航结束 status={status} ok={ok} result={res!r}"
        )
        self._publish_event(
            {
                "event": "navigate_done",
                "ok": ok,
                "status": int(status),
            }
        )


def main() -> None:
    rclpy.init()
    node = CommandExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
