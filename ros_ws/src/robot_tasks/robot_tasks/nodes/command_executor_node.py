from __future__ import annotations

import json
import math
import threading
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from std_msgs.msg import String

from robot_tasks.utils.cmd_vel_brake import publish_cmd_vel_brake
from robot_tasks.utils.nav_action import wait_for_navigate_to_pose
from robot_tasks.utils.turn_intent import fix_turn_args, normalize_angle_rad

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    tf2_ros = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[misc, assignment]


class CommandExecutorNode(Node):
    """订阅 ``parsed_intent_topic`` 上的 JSON（由 llm_router 发布），执行 Nav2 导航与任务事件。"""

    def __init__(self) -> None:
        super().__init__("command_executor")
        self.declare_parameter("parsed_intent_topic", "/interaction/parsed_intent")
        self.declare_parameter("task_events_topic", "/task/events")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("desired_object_topic", "/yolo_objects/desired_object_label")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("nav2_ready_timeout_sec", 120.0)
        self.declare_parameter("cmd_vel_brake_bursts", 12)

        self._events_topic = str(self.get_parameter("task_events_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("robot_base_frame").value)
        self._nav_ready_timeout = float(self.get_parameter("nav2_ready_timeout_sec").value)
        self._brake_bursts = int(self.get_parameter("cmd_vel_brake_bursts").value)
        desired_topic = str(self.get_parameter("desired_object_topic").value).strip()
        action_name = str(self.get_parameter("navigate_to_pose_action").value).strip() or "navigate_to_pose"

        self._events_pub = self.create_publisher(String, self._events_topic, 10)
        self._desired_object_topic = desired_topic
        self._desired_pub = (
            self.create_publisher(String, desired_topic, 10) if desired_topic else None
        )
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._goal_lock = threading.Lock()
        self._current_goal_handle: ClientGoalHandle | None = None
        self._tf_buffer = None
        self._tf_listener = None
        if tf2_ros is not None:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        topic = str(self.get_parameter("parsed_intent_topic").value)
        self.create_subscription(String, topic, self._on_parsed_intent, 10)
        self.get_logger().info(
            f"[command_executor] intent={topic} events={self._events_topic} "
            f"desired_object={desired_topic or '(disabled)'} "
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
            self._publish_desired_object_label("")
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
            label = self._extract_object_label(args)
            self._publish_desired_object_label(label)
            self._publish_event({"event": "fetch_object", "args": args, "payload": obj})
            return
        if cmd == "navigate_to_pose":
            nav_args = fix_turn_args(dict(args or {}), str((args or {}).get("user_request_zh", "")))
            if nav_args.get("relative_yaw_deg") != (args or {}).get("relative_yaw_deg"):
                self.get_logger().info(
                    f"[command_executor] 转向符号已按原文校正: "
                    f"{(args or {}).get('relative_yaw_deg')!r} -> {nav_args.get('relative_yaw_deg')!r}"
                )
            self._send_nav_goal(nav_args)
            return

        self.get_logger().warning(f"[command_executor] 未实现的 command: {cmd!r}")

    @staticmethod
    def _extract_object_label(args: dict[str, Any]) -> str:
        if not args:
            return ""
        for key in ("object_label", "label", "object", "target"):
            v = args.get(key)
            if v is not None and str(v).strip():
                return str(v).strip()
        zh = args.get("user_request_zh")
        return str(zh).strip() if zh else ""

    def _publish_desired_object_label(self, label: str) -> None:
        if self._desired_pub is None:
            return
        self._desired_pub.publish(String(data=label))
        if label:
            self.get_logger().info(
                f"[command_executor] -> {self._desired_object_topic}: {label!r}（YOLO 按语义类别选绿点）"
            )
        else:
            self.get_logger().info(
                f"[command_executor] -> {self._desired_object_topic}: (cleared)"
            )

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

    def _current_pose_xy_yaw(self) -> tuple[float, float, float] | None:
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (float(t.x), float(t.y), yaw)

    def _resolve_nav_args(self, args: dict[str, Any]) -> tuple[str, float, float, float] | None:
        args = fix_turn_args(dict(args))
        frame = str(args.get("frame_id", self._map_frame) or self._map_frame)
        yaw_only = bool(args.get("yaw_only") or args.get("rotate_in_place"))
        rel_yaw = args.get("relative_yaw_deg", args.get("yaw_delta_deg"))
        use_current = bool(args.get("use_current_pose") or yaw_only or rel_yaw is not None)

        x = args.get("x")
        y = args.get("y")
        yaw_deg = args.get("yaw_deg", args.get("yaw_delta_deg", 0.0))

        if use_current:
            cur = self._current_pose_xy_yaw()
            if cur is None:
                self.get_logger().error("[command_executor] 无法读取当前位姿（TF），原地转向失败")
                return None
            cx, cy, cyaw = cur
            if rel_yaw is not None or yaw_only:
                try:
                    delta_deg = float(rel_yaw if rel_yaw is not None else yaw_deg)
                except (TypeError, ValueError):
                    delta_deg = 0.0
                yaw = normalize_angle_rad(cyaw + math.radians(delta_deg))
                self.get_logger().info(
                    f"[command_executor] 原地转向: 当前 yaw={math.degrees(cyaw):.1f}° "
                    f"Δ={delta_deg:+.1f}° -> 目标 yaw={math.degrees(yaw):.1f}°"
                )
                return (frame, cx, cy, yaw)
            x = cx if x is None else float(x)
            y = cy if y is None else float(y)
            yaw = (
                cyaw
                if yaw_deg in (None, "")
                else normalize_angle_rad(math.radians(float(yaw_deg)))
            )
            return (frame, float(x), float(y), yaw)

        return (
            frame,
            float(x if x is not None else 0.0),
            float(y if y is not None else 0.0),
            normalize_angle_rad(math.radians(float(yaw_deg))),
        )

    def _publish_nav_brake(self) -> None:
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self.get_logger().debug("[command_executor] 已发布 cmd_vel 刹停")

    def _send_nav_goal(self, args: dict[str, Any]) -> None:
        if not wait_for_navigate_to_pose(
            self,
            self._nav_client,
            timeout_sec=self._nav_ready_timeout,
            log=self.get_logger(),
        ):
            self.get_logger().error("[command_executor] NavigateToPose 不可用（Nav2 是否已启动？）")
            return

        resolved = self._resolve_nav_args(args)
        if resolved is None:
            return
        frame, x, y, yaw = resolved

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
            f"[command_executor] 发送 NavigateToPose frame={frame} x={x:.3f} y={y:.3f} "
            f"yaw_deg={math.degrees(yaw):.1f}"
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
        self._publish_nav_brake()
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
