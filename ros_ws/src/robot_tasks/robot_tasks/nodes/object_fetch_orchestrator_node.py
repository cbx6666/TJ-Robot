"""取物流程编排：巡检搜索目标 → Nav2 停靠对准 → mock 机械臂几何校验。

订阅 ``fetch_object`` 任务事件，发布 ``patrol_start`` 驱动覆盖式巡检；在 YOLO
``target_point_map`` 稳定可见时**优先**停止巡检并导航到物体前方；目标持续丢失则恢复巡检。
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from enum import Enum
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from robot_tasks.utils.cmd_vel_brake import publish_cmd_vel_brake
from robot_tasks.utils.nav_action import wait_for_navigate_to_pose

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    tf2_ros = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[misc, assignment]


class _State(str, Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    APPROACHING = "approaching"
    PICKING = "picking"
    DONE = "done"
    FAILED = "failed"


_LABEL_ZH = {
    "cup": "杯子",
    "bottle": "瓶子",
    "vase": "花瓶",
    "chair": "椅子",
    "book": "书",
    "cell phone": "手机",
}


def _label_zh(label: str) -> str:
    low = (label or "").strip().lower()
    if low in _LABEL_ZH:
        return _LABEL_ZH[low]
    for zh in ("杯子", "瓶子", "花瓶", "椅子", "书", "手机"):
        if zh in (label or ""):
            return zh
    return label or "物体"


class ObjectFetchOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("object_fetch_orchestrator")
        self.declare_parameter("task_events_topic", "/task/events")
        self.declare_parameter("target_point_topic", "/yolo_objects/target_point_map")
        self.declare_parameter("target_map_valid_topic", "/yolo_objects/target_map_valid")
        self.declare_parameter("target_label_topic", "/yolo_objects/target_label")
        self.declare_parameter("manipulation_command_topic", "/manipulation/command_text")
        self.declare_parameter("manipulation_status_topic", "/manipulation/status_text")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("approach_standoff_m", 0.55)
        self.declare_parameter("target_stable_samples", 5)
        self.declare_parameter("target_stable_max_jump_m", 0.25)
        self.declare_parameter("target_tentative_samples", 2)
        self.declare_parameter("target_tentative_max_jump_m", 0.45)
        self.declare_parameter("target_point_stale_sec", 0.7)
        self.declare_parameter("target_lost_resume_patrol_sec", 2.5)
        self.declare_parameter("approach_replan_min_shift_m", 0.35)
        self.declare_parameter("search_timeout_sec", 180.0)
        self.declare_parameter("approach_timeout_sec", 120.0)
        self.declare_parameter("pick_verify_timeout_sec", 45.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav2_ready_timeout_sec", 120.0)
        self.declare_parameter("cmd_vel_brake_bursts", 12)

        self._events_topic = str(self.get_parameter("task_events_topic").value)
        self._target_topic = str(self.get_parameter("target_point_topic").value)
        self._target_valid_topic = str(self.get_parameter("target_map_valid_topic").value)
        self._label_topic = str(self.get_parameter("target_label_topic").value)
        self._mani_cmd_topic = str(self.get_parameter("manipulation_command_topic").value)
        self._mani_status_topic = str(self.get_parameter("manipulation_status_topic").value)
        self._base_frame = str(self.get_parameter("robot_base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._standoff = float(self.get_parameter("approach_standoff_m").value)
        self._stable_n = max(3, int(self.get_parameter("target_stable_samples").value))
        self._stable_jump = float(self.get_parameter("target_stable_max_jump_m").value)
        self._tentative_n = max(1, int(self.get_parameter("target_tentative_samples").value))
        self._tentative_jump = float(self.get_parameter("target_tentative_max_jump_m").value)
        self._point_stale_sec = float(self.get_parameter("target_point_stale_sec").value)
        self._target_lost_sec = float(self.get_parameter("target_lost_resume_patrol_sec").value)
        self._replan_shift = float(self.get_parameter("approach_replan_min_shift_m").value)
        self._search_timeout = float(self.get_parameter("search_timeout_sec").value)
        self._approach_timeout = float(self.get_parameter("approach_timeout_sec").value)
        self._pick_timeout = float(self.get_parameter("pick_verify_timeout_sec").value)
        self._nav_ready_timeout = float(self.get_parameter("nav2_ready_timeout_sec").value)
        self._brake_bursts = int(self.get_parameter("cmd_vel_brake_bursts").value)

        action_name = str(self.get_parameter("navigate_to_pose_action").value).strip() or "navigate_to_pose"
        self._events_pub = self.create_publisher(String, self._events_topic, 10)
        self._mani_pub = self.create_publisher(String, self._mani_cmd_topic, 10)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._goal_lock = threading.Lock()
        self._nav_handle: ClientGoalHandle | None = None

        self._state = _State.IDLE
        self._object_label = ""
        self._object_zh = ""
        self._search_started = 0.0
        self._approach_started = 0.0
        self._pick_started = 0.0
        self._target_buf: deque[tuple[float, float, float]] = deque(maxlen=max(self._stable_n, self._tentative_n))
        self._locked_target: tuple[float, float, float] | None = None
        self._latest_label = ""
        self._last_target_rx = 0.0
        self._target_map_valid = False
        self._patrol_stopped_for_target = False
        self._approach_confirmed = False
        self._last_replan_mono = 0.0

        self._tf_buffer = None
        self._tf_listener = None
        if tf2_ros is not None:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(String, self._events_topic, self._on_task_event, 10)
        self.create_subscription(PointStamped, self._target_topic, self._on_target_point, 10)
        self.create_subscription(Bool, self._target_valid_topic, self._on_target_map_valid, 10)
        self.create_subscription(String, self._label_topic, self._on_target_label, 10)
        self.create_subscription(String, self._mani_status_topic, self._on_mani_status, 10)
        self.create_timer(0.25, self._tick)

        self.get_logger().info(
            f"[object_fetch] events={self._events_topic} target={self._target_topic} "
            f"nav={action_name!r} standoff={self._standoff}m "
            f"tentative={self._tentative_n}@{self._tentative_jump}m stable={self._stable_n}"
        )

    def _on_task_event(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            ev = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(ev, dict):
            return
        name = str(ev.get("event", "")).strip()
        if name == "fetch_object":
            args = ev.get("args") if isinstance(ev.get("args"), dict) else {}
            self._begin_fetch(args)
        elif name == "stop":
            source = str(ev.get("source", "")).strip()
            if source == "object_fetch":
                return
            if self._state not in (_State.IDLE, _State.DONE, _State.FAILED):
                self._fail("canceled_by_stop")

    def _begin_fetch(self, args: dict[str, Any]) -> None:
        if self._state not in (_State.IDLE, _State.DONE, _State.FAILED):
            self.get_logger().warning("[object_fetch] 已有取物任务进行中，忽略新 fetch_object")
            return
        label = ""
        for key in ("object_label", "label", "object"):
            v = args.get(key)
            if v is not None and str(v).strip():
                label = str(v).strip()
                break
        self._object_label = label
        self._object_zh = _label_zh(label)
        self._reset_search_state()
        self._state = _State.SEARCHING
        self._search_started = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"[object_fetch] 开始搜索 label={label!r} → 启动覆盖巡检"
        )
        self._publish_event(
            {
                "event": "fetch_object_started",
                "object_label": label,
                "phase": "search",
            }
        )
        self._publish_event(
            {
                "event": "patrol_start",
                "scope": "object_search",
                "object_label": label,
            }
        )

    def _reset_search_state(self) -> None:
        self._target_buf.clear()
        self._locked_target = None
        self._last_target_rx = 0.0
        self._target_map_valid = False
        self._patrol_stopped_for_target = False
        self._approach_confirmed = False
        self._last_replan_mono = 0.0

    def _on_target_map_valid(self, msg: Bool) -> None:
        self._target_map_valid = bool(msg.data)
        if not self._target_map_valid and self._state == _State.SEARCHING:
            self._target_buf.clear()

    def _on_target_point(self, msg: PointStamped) -> None:
        if self._state not in (_State.SEARCHING, _State.APPROACHING):
            return
        frame = (msg.header.frame_id or "").strip() or self._map_frame
        if frame != self._map_frame:
            return
        p = msg.point
        if not math.isfinite(p.x) or not math.isfinite(p.y):
            return
        if abs(p.x) < 1e-4 and abs(p.y) < 1e-4:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_target_rx = now
        pt = (float(p.x), float(p.y), float(p.z))
        self._target_buf.append(pt)
        if self._state == _State.APPROACHING and self._locked_target is not None:
            self._maybe_update_locked_target(pt)

    def _on_target_label(self, msg: String) -> None:
        self._latest_label = (msg.data or "").strip()

    def _cluster_from_buf(self, n: int, max_jump: float) -> tuple[float, float, float] | None:
        if len(self._target_buf) < n:
            return None
        pts = list(self._target_buf)[-n:]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        for x, y, _ in pts:
            if math.hypot(x - cx, y - cy) > max_jump:
                return None
        return (cx, cy, cz)

    def _target_is_stable(self) -> tuple[float, float, float] | None:
        return self._cluster_from_buf(self._stable_n, self._stable_jump)

    def _target_is_tentative(self) -> tuple[float, float, float] | None:
        return self._cluster_from_buf(self._tentative_n, self._tentative_jump)

    def _target_feed_stale(self, now: float) -> bool:
        if self._last_target_rx <= 0.0:
            return True
        return (now - self._last_target_rx) > self._point_stale_sec

    def _stop_patrol_for_target(self, reason: str) -> None:
        if self._patrol_stopped_for_target:
            return
        self._patrol_stopped_for_target = True
        self.get_logger().info(f"[object_fetch] 停止巡检 → 优先接近目标 ({reason})")
        self._publish_event(
            {"event": "stop", "source": "object_fetch", "reason": reason}
        )

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._state == _State.SEARCHING:
            if now - self._search_started > self._search_timeout:
                self._fail("search_timeout")
                return
            if self._target_feed_stale(now):
                if self._target_buf:
                    self._target_buf.clear()
                return
            stable = self._target_is_stable()
            tentative = self._target_is_tentative()
            if stable is not None:
                self._stop_patrol_for_target("target_stable")
                self._begin_approach(stable, confirmed=True)
            elif tentative is not None:
                self._stop_patrol_for_target("target_tentative")
                self._begin_approach(tentative, confirmed=False)
        elif self._state == _State.APPROACHING:
            if now - self._approach_started > self._approach_timeout:
                self._fail("approach_timeout")
                return
            if self._target_feed_stale(now) or not self._target_map_valid:
                if self._last_target_rx > 0.0 and (now - self._last_target_rx) > self._target_lost_sec:
                    self._resume_patrol_search("target_lost")
        elif self._state == _State.PICKING:
            if now - self._pick_started > self._pick_timeout:
                self._fail("pick_verify_timeout")

    def _maybe_update_locked_target(self, pt: tuple[float, float, float]) -> None:
        if self._locked_target is None:
            return
        lx, ly, _ = self._locked_target
        shift = math.hypot(pt[0] - lx, pt[1] - ly)
        alpha = 0.55
        self._locked_target = (
            alpha * pt[0] + (1.0 - alpha) * lx,
            alpha * pt[1] + (1.0 - alpha) * ly,
            alpha * pt[2] + (1.0 - alpha) * self._locked_target[2],
        )
        now_mono = time.monotonic()
        if shift >= self._replan_shift and (now_mono - self._last_replan_mono) > 1.5:
            self._last_replan_mono = now_mono
            self.get_logger().info(
                f"[object_fetch] 目标移动 {shift:.2f}m，重新规划接近点"
            )
            self._cancel_nav_goal()
            self._start_approach(self._locked_target)

    def _resume_patrol_search(self, reason: str) -> None:
        if self._state != _State.APPROACHING:
            return
        self.get_logger().warning(
            f"[object_fetch] 目标持续不可见 ({reason})，取消接近并恢复巡检"
        )
        self._cancel_nav_goal()
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self._reset_search_state()
        self._state = _State.SEARCHING
        self._search_started = self.get_clock().now().nanoseconds * 1e-9
        self._publish_event(
            {
                "event": "fetch_object_phase",
                "phase": "search_resume",
                "reason": reason,
            }
        )
        self._publish_event(
            {
                "event": "patrol_start",
                "scope": "object_search",
                "object_label": self._object_label,
            }
        )

    def _robot_xy_yaw(self) -> tuple[float, float, float] | None:
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as e:
            self.get_logger().debug(f"[object_fetch] TF {self._map_frame}<-{self._base_frame}: {e}")
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (float(t.x), float(t.y), yaw)

    def _begin_approach(self, target: tuple[float, float, float], *, confirmed: bool) -> None:
        if self._state not in (_State.SEARCHING, _State.APPROACHING):
            return
        if self._state == _State.APPROACHING:
            self._locked_target = target
            if confirmed:
                self._approach_confirmed = True
            return
        self._locked_target = target
        self._state = _State.APPROACHING
        self._approach_started = self.get_clock().now().nanoseconds * 1e-9
        self._approach_confirmed = confirmed
        phase = "approach" if confirmed else "approach_tentative"
        self.get_logger().info(
            f"[object_fetch] {'稳定' if confirmed else '疑似'}目标 @ map "
            f"({target[0]:.2f},{target[1]:.2f}) label={self._latest_label!r} → NavigateToPose"
        )
        self._publish_event(
            {
                "event": "fetch_object_phase",
                "phase": phase,
                "target_xyz": [target[0], target[1], target[2]],
                "confirmed": confirmed,
            }
        )
        self._start_approach(target)

    def _compute_approach(self, ox: float, oy: float) -> tuple[float, float, float] | None:
        robot = self._robot_xy_yaw()
        if robot is None:
            dx, dy = -1.0, 0.0
        else:
            rx, ry, _ = robot
            dx = ox - rx
            dy = oy - ry
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            dx, dy = 1.0, 0.0
            dist = 1.0
        ax = ox - self._standoff * dx / dist
        ay = oy - self._standoff * dy / dist
        yaw = math.atan2(oy - ay, ox - ax)
        return (ax, ay, yaw)

    def _cancel_nav_goal(self) -> None:
        with self._goal_lock:
            gh = self._nav_handle
            self._nav_handle = None
        if gh is not None:
            try:
                self._nav_client.cancel_goal_async(gh)
            except Exception:
                pass

    def _start_approach(self, target: tuple[float, float, float]) -> None:
        approach = self._compute_approach(target[0], target[1])
        if approach is None:
            self._fail("no_robot_tf")
            return
        ax, ay, yaw = approach
        if not wait_for_navigate_to_pose(
            self,
            self._nav_client,
            timeout_sec=self._nav_ready_timeout,
            log=self.get_logger(),
        ):
            self._fail("nav2_unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = ax
        goal.pose.pose.position.y = ay
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
        self.get_logger().info(
            f"[object_fetch] NavigateToPose → ({ax:.2f},{ay:.2f}) yaw={math.degrees(yaw):.1f}° "
            f"（物体 @ {target[0]:.2f},{target[1]:.2f}）"
        )
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future: Any) -> None:
        if self._state != _State.APPROACHING:
            return
        try:
            gh = future.result()
        except Exception as e:
            self._fail(f"nav_send_error:{e}")
            return
        if not gh.accepted:
            self._fail("nav_goal_rejected")
            return
        with self._goal_lock:
            self._nav_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future: Any) -> None:
        with self._goal_lock:
            self._nav_handle = None
        if self._state != _State.APPROACHING:
            return
        try:
            wrapper = future.result()
            status = int(wrapper.status)
        except Exception as e:
            self._fail(f"nav_result_error:{e}")
            return
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        if status == GoalStatus.STATUS_CANCELED:
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            if not self._target_feed_stale(self.get_clock().now().nanoseconds * 1e-9):
                self._resume_patrol_search(f"nav_status_{status}")
            else:
                self._fail(f"nav_status_{status}")
            return
        if self._locked_target is None:
            self._fail("no_locked_target")
            return
        if not self._approach_confirmed:
            stable = self._target_is_stable()
            if stable is None and self._target_feed_stale(
                self.get_clock().now().nanoseconds * 1e-9
            ):
                self._resume_patrol_search("tentative_nav_without_stable_target")
                return
            if stable is not None:
                self._locked_target = stable
                self._approach_confirmed = True
        tx, ty, tz = self._locked_target
        coord = f"target_frame={self._map_frame};target_xyz={tx:.3f},{ty:.3f},{tz:.3f}"
        cmd = f"PICK:{self._object_zh};{coord}"
        self._state = _State.PICKING
        self._pick_started = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"[object_fetch] 导航到位，请求抓取: {cmd}")
        self._mani_pub.publish(String(data=cmd))
        self._publish_event({"event": "fetch_object_phase", "phase": "pick_verify"})

    def _on_mani_status(self, msg: String) -> None:
        if self._state != _State.PICKING:
            return
        text = (msg.data or "").strip()
        if "picked" in text.lower() and "failed" not in text.lower():
            self._succeed(text)
        elif "pick_failed" in text.lower() or "not_in_range" in text.lower():
            self._fail(text[:120])

    def _succeed(self, detail: str) -> None:
        self.get_logger().info(f"[object_fetch] 完成: {detail}")
        self._state = _State.DONE
        self._publish_event(
            {
                "event": "fetch_object_done",
                "ok": True,
                "object_label": self._object_label,
                "detail": detail,
            }
        )

    def _fail(self, reason: str) -> None:
        self.get_logger().warning(f"[object_fetch] 失败: {reason}")
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self._cancel_nav_goal()
        if self._state == _State.SEARCHING and self._patrol_stopped_for_target:
            self._publish_event({"event": "stop", "source": "object_fetch", "reason": reason})
        elif self._state == _State.SEARCHING:
            self._publish_event({"event": "stop", "source": "object_fetch", "reason": reason})
        self._state = _State.FAILED
        self._publish_event(
            {
                "event": "fetch_object_done",
                "ok": False,
                "object_label": self._object_label,
                "reason": reason,
            }
        )

    def _publish_event(self, data: dict[str, Any]) -> None:
        try:
            s = json.dumps(data, ensure_ascii=False)
        except Exception as e:
            self.get_logger().error(f"[object_fetch] 事件序列化失败: {e}")
            return
        self._events_pub.publish(String(data=s))


def main() -> None:
    rclpy.init()
    node = ObjectFetchOrchestratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
