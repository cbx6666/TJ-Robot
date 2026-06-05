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
    RETURNING = "returning"
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

_LABEL_ALIASES: tuple[tuple[str, str], ...] = (
    ("水杯", "cup"),
    ("茶杯", "cup"),
    ("杯子", "cup"),
    ("杯", "cup"),
    ("可乐罐", "bottle"),
    ("可乐", "bottle"),
    ("瓶子", "bottle"),
    ("瓶", "bottle"),
    ("花瓶", "vase"),
)


def _label_zh(label: str) -> str:
    low = (label or "").strip().lower()
    if low in _LABEL_ZH:
        return _LABEL_ZH[low]
    for zh in ("杯子", "瓶子", "花瓶", "椅子", "书", "手机"):
        if zh in (label or ""):
            return zh
    return label or "物体"


def _canonical_label(label: str) -> str:
    raw = (label or "").strip()
    low = raw.lower().replace("_", " ")
    direct = {
        "cup": "cup",
        "mug": "cup",
        "water cup": "cup",
        "bottle": "bottle",
        "beer": "bottle",
        "coke": "bottle",
        "coke can": "bottle",
        "vase": "vase",
        "chair": "chair",
        "book": "book",
        "cell phone": "cell phone",
        "phone": "cell phone",
    }
    if low in direct:
        return direct[low]
    for zh, en in _LABEL_ALIASES:
        if zh in raw:
            return en
    return raw or "object"


def _fmt_xyz(pt: tuple[float, float, float] | None) -> str:
    if pt is None:
        return "(none)"
    return f"({pt[0]:.3f},{pt[1]:.3f},{pt[2]:.3f})"


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
        self.declare_parameter("target_stable_max_jump_m", 0.15)
        self.declare_parameter("target_tentative_samples", 2)
        self.declare_parameter("target_tentative_max_jump_m", 0.45)
        self.declare_parameter("target_point_stale_sec", 1.0)
        self.declare_parameter("target_lost_resume_patrol_sec", 5.0)
        self.declare_parameter("search_timeout_sec", 180.0)
        self.declare_parameter("approach_timeout_sec", 120.0)
        self.declare_parameter("pick_verify_timeout_sec", 45.0)
        self.declare_parameter("return_timeout_sec", 120.0)
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
        self._search_timeout = float(self.get_parameter("search_timeout_sec").value)
        self._approach_timeout = float(self.get_parameter("approach_timeout_sec").value)
        self._pick_timeout = float(self.get_parameter("pick_verify_timeout_sec").value)
        self._return_timeout = float(self.get_parameter("return_timeout_sec").value)
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
        self._return_started = 0.0
        self._home_pose: tuple[float, float, float] | None = None
        self._pending_task_ok = False
        self._pending_task_detail = ""
        self._target_buf: deque[tuple[float, float, float]] = deque(maxlen=max(self._stable_n, self._tentative_n))
        self._locked_target: tuple[float, float, float] | None = None
        self._latest_label = ""
        self._last_target_frame = ""
        self._last_target_rx = 0.0
        self._target_map_valid = False
        self._patrol_stopped_for_target = False
        self._approach_confirmed = False
        self._last_yolo_search_log_mono = 0.0
        self._last_yolo_lost_log_mono = 0.0
        self._approach_lost_since = 0.0

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
            f"stable_frames>={self._stable_n} stable_std<={self._stable_jump:.2f}m "
            f"target_age<={self._point_stale_sec:.1f}s"
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
                self._abort("canceled_by_stop")

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
        self._object_label = _canonical_label(label)
        self._object_zh = _label_zh(label)
        self._home_pose = self._robot_xy_yaw()
        if self._home_pose is None:
            self.get_logger().warning("[object_fetch] 无法记录接令位置，拒绝启动取物任务")
            self._state = _State.FAILED
            self._publish_event(
                {
                    "event": "fetch_object_done",
                    "ok": False,
                    "pick_ok": False,
                    "return_ok": False,
                    "object_label": self._object_label,
                    "reason": "initial_pose_unavailable",
                    "return_reason": "initial_pose_unavailable",
                }
            )
            return
        self._reset_search_state()
        self._state = _State.SEARCHING
        self._search_started = self.get_clock().now().nanoseconds * 1e-9
        hx, hy, hyaw = self._home_pose
        self.get_logger().info(
            f"[object_fetch] 开始搜索 label={self._object_label!r} → 启动覆盖巡检；"
            f"记录返航点=({hx:.3f},{hy:.3f}) yaw={math.degrees(hyaw):.1f}°"
        )
        self._publish_event(
            {
                "event": "fetch_object_started",
                "object_label": self._object_label,
                "phase": "search",
                "home_pose": {
                    "frame_id": self._map_frame,
                    "x": hx,
                    "y": hy,
                    "yaw": hyaw,
                },
            }
        )
        self._publish_event(
            {
                "event": "patrol_start",
                "scope": "object_search",
                "object_label": self._object_label,
            }
        )

    def _reset_search_state(self) -> None:
        self._target_buf.clear()
        self._locked_target = None
        self._last_target_rx = 0.0
        self._target_map_valid = False
        self._patrol_stopped_for_target = False
        self._approach_confirmed = False
        self._last_yolo_search_log_mono = 0.0
        self._last_yolo_lost_log_mono = 0.0
        self._approach_lost_since = 0.0

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
        self._last_target_frame = frame
        pt = (float(p.x), float(p.y), float(p.z))
        self._target_buf.append(pt)
        if self._state == _State.SEARCHING:
            self._log_yolo_search_seen(now, pt)

    def _on_target_label(self, msg: String) -> None:
        self._latest_label = (msg.data or "").strip()

    def _target_stats(
        self, n: int | None = None
    ) -> tuple[int, tuple[float, float, float] | None, float]:
        if not self._target_buf:
            return (0, None, math.inf)
        pts = list(self._target_buf)
        if n is not None:
            pts = pts[-n:]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        variance = sum((p[0] - cx) ** 2 + (p[1] - cy) ** 2 for p in pts) / len(pts)
        return (len(pts), (cx, cy, cz), math.sqrt(max(variance, 0.0)))

    def _log_yolo_search_seen(self, now: float, pt: tuple[float, float, float]) -> None:
        mono = time.monotonic()
        if mono - self._last_yolo_search_log_mono < 0.5:
            return
        self._last_yolo_search_log_mono = mono
        frames, _mean, std = self._target_stats(min(len(self._target_buf), self._stable_n))
        age = now - self._last_target_rx if self._last_target_rx > 0.0 else math.inf
        self.get_logger().info(
            "[object_fetch] yolo_target_seen "
            f"label={self._object_label} yolo_label={self._latest_label!r} "
            f"target_frame={self._last_target_frame or self._map_frame} "
            f"target_xyz_map={_fmt_xyz(pt)} stable_frames={frames} "
            f"target_std={std:.3f} age={age:.3f}s"
        )

    def _cluster_from_buf(self, n: int, max_std: float) -> tuple[float, float, float] | None:
        if len(self._target_buf) < n:
            return None
        _frames, mean, std = self._target_stats(n)
        if mean is None or std > max_std:
            return None
        return mean

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
            if stable is not None:
                self._stop_patrol_for_target("target_stable")
                self._begin_approach(stable, confirmed=True)
        elif self._state == _State.APPROACHING:
            if now - self._approach_started > self._approach_timeout:
                self.get_logger().warning("[object_fetch] nav_timeout reason=approach_timeout")
                self._fail("approach_timeout")
                return
            self._handle_approach_yolo_lost(now)
        elif self._state == _State.PICKING:
            if now - self._pick_started > self._pick_timeout:
                self._fail("pick_verify_timeout")
        elif self._state == _State.RETURNING:
            if (
                self._return_started > 0.0
                and now - self._return_started > self._return_timeout
            ):
                self.get_logger().warning("[object_fetch] 返航超时")
                self._cancel_nav_goal()
                publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
                self._finish_task(return_ok=False, return_reason="return_timeout")

    def _handle_approach_yolo_lost(self, now: float) -> None:
        lost = self._target_feed_stale(now) or not self._target_map_valid
        if not lost:
            self._approach_lost_since = 0.0
            return
        if self._approach_lost_since <= 0.0:
            self._approach_lost_since = now
        mono = time.monotonic()
        if mono - self._last_yolo_lost_log_mono >= 1.0:
            self._last_yolo_lost_log_mono = mono
            last_age = now - self._last_target_rx if self._last_target_rx > 0.0 else math.inf
            lost_for = now - self._approach_lost_since
            self.get_logger().info(
                "[object_fetch] yolo_lost_in_approach_continue_locked_target "
                f"label={self._object_label} locked_xyz={_fmt_xyz(self._locked_target)} "
                f"lost_for={lost_for:.2f}s last_seen_age={last_age:.2f}s "
                "action=keep_navigating"
            )

    def _resume_patrol_search(self, reason: str) -> None:
        if self._state != _State.APPROACHING:
            return
        self.get_logger().warning(
            f"[object_fetch] target_lost_exceeded_threshold reason={reason} "
            f"locked_xyz={_fmt_xyz(self._locked_target)}"
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
            return
        self._locked_target = target
        self._state = _State.APPROACHING
        self._approach_started = self.get_clock().now().nanoseconds * 1e-9
        self._approach_confirmed = confirmed
        phase = "approach" if confirmed else "approach_tentative"
        frames, _mean, std = self._target_stats(self._stable_n)
        self.get_logger().info(
            f"[object_fetch] {'稳定' if confirmed else '疑似'}目标 @ map "
            f"({target[0]:.2f},{target[1]:.2f}) label={self._latest_label!r} → NavigateToPose"
        )
        self.get_logger().info(
            f"[object_fetch] locked_target label={self._object_label} "
            f"xyz={_fmt_xyz(self._locked_target)} stable_frames={frames} "
            f"target_std={std:.3f}"
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
        self.get_logger().info(
            f"[object_fetch] nav_goal_sent label={self._object_label} "
            f"goal_xyz_map=({ax:.3f},{ay:.3f},0.000) "
            f"goal_yaw_deg={math.degrees(yaw):.1f} "
            f"locked_xyz={_fmt_xyz(self._locked_target)}"
        )
        fut = self._nav_client.send_goal_async(goal)
        fut.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future: Any) -> None:
        try:
            gh = future.result()
        except Exception as e:
            if self._state == _State.APPROACHING:
                self.get_logger().warning(
                    f"[object_fetch] nav_failed reason=nav_send_error detail={e}"
                )
                self._fail(f"nav_send_error:{e}")
            return
        if self._state != _State.APPROACHING:
            if gh.accepted:
                self._nav_client.cancel_goal_async(gh)
            return
        if not gh.accepted:
            self.get_logger().warning("[object_fetch] nav_failed reason=nav_goal_rejected")
            self._fail("nav_goal_rejected")
            return
        with self._goal_lock:
            self._nav_handle = gh
        gh.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future: Any) -> None:
        if self._state != _State.APPROACHING:
            return
        with self._goal_lock:
            self._nav_handle = None
        try:
            wrapper = future.result()
            status = int(wrapper.status)
        except Exception as e:
            self.get_logger().warning(f"[object_fetch] nav_failed reason=nav_result_error detail={e}")
            self._fail(f"nav_result_error:{e}")
            return
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        if status == GoalStatus.STATUS_CANCELED:
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"[object_fetch] nav_failed status={status}")
            if not self._target_feed_stale(self.get_clock().now().nanoseconds * 1e-9):
                self._resume_patrol_search(f"nav_status_{status}")
            else:
                self._fail(f"nav_status_{status}")
            return
        self.get_logger().info(
            f"[object_fetch] nav_goal_reached label={self._object_label} "
            f"locked_xyz={_fmt_xyz(self._locked_target)}"
        )
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
        coord = f"locked_target_frame={self._map_frame};locked_target_xyz={tx:.3f},{ty:.3f},{tz:.3f}"
        cmd = f"PICK:{self._object_label};{coord}"
        self._state = _State.PICKING
        self._pick_started = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"[object_fetch] 导航到位，请求抓取: {cmd}")
        self.get_logger().info(
            f"[object_fetch] pick_command_sent label={self._object_label} "
            f"locked_xyz={_fmt_xyz(self._locked_target)}"
        )
        self._mani_pub.publish(String(data=cmd))
        self._publish_event({"event": "fetch_object_phase", "phase": "pick_verify"})

    def _on_mani_status(self, msg: String) -> None:
        if self._state != _State.PICKING:
            return
        text = (msg.data or "").strip()
        low = text.lower()
        if ("pick_success" in low or "picked" in low) and "failed" not in low:
            self._succeed(text)
        elif "pick_failed" in low or "not_in_range" in low:
            self._fail(text[:120])

    def _succeed(self, detail: str) -> None:
        self.get_logger().info(f"[object_fetch] mock 抓取成功: {detail}")
        self._begin_return(task_ok=True, detail=detail)

    def _fail(self, reason: str) -> None:
        self.get_logger().warning(f"[object_fetch] 失败: {reason}")
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self._cancel_nav_goal()
        if self._state == _State.SEARCHING:
            self._publish_event({"event": "stop", "source": "object_fetch", "reason": reason})
        self._begin_return(task_ok=False, detail=reason)

    def _abort(self, reason: str) -> None:
        self.get_logger().warning(f"[object_fetch] 中止且不返航: {reason}")
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self._cancel_nav_goal()
        self._state = _State.FAILED
        self._publish_event(
            {
                "event": "fetch_object_done",
                "ok": False,
                "pick_ok": False,
                "return_ok": False,
                "object_label": self._object_label,
                "reason": reason,
                "return_reason": "return_skipped_by_stop",
            }
        )

    def _begin_return(self, *, task_ok: bool, detail: str) -> None:
        if self._state == _State.RETURNING:
            return
        self._pending_task_ok = task_ok
        self._pending_task_detail = detail
        home = self._home_pose
        if home is None:
            self._finish_task(return_ok=False, return_reason="initial_pose_unavailable")
            return

        self._cancel_nav_goal()
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        self._state = _State.RETURNING
        self._return_started = 0.0
        hx, hy, hyaw = home
        self.get_logger().info(
            f"[object_fetch] pre_return_ok={task_ok}，开始返航 "
            f"goal=({hx:.3f},{hy:.3f}) yaw={math.degrees(hyaw):.1f}°"
        )
        self._publish_event(
            {
                "event": "fetch_object_phase",
                "phase": "returning",
                "pick_ok": task_ok,
                "home_pose": {
                    "frame_id": self._map_frame,
                    "x": hx,
                    "y": hy,
                    "yaw": hyaw,
                },
            }
        )
        nav_ready = wait_for_navigate_to_pose(
            self,
            self._nav_client,
            timeout_sec=self._nav_ready_timeout,
            log=self.get_logger(),
        )
        if self._state != _State.RETURNING:
            return
        if not nav_ready:
            self._finish_task(return_ok=False, return_reason="return_nav2_unavailable")
            return
        self._return_started = self.get_clock().now().nanoseconds * 1e-9

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = hx
        goal.pose.pose.position.y = hy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(hyaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(hyaw * 0.5)
        try:
            future = self._nav_client.send_goal_async(goal)
        except Exception as e:
            self._finish_task(
                return_ok=False,
                return_reason=f"return_nav_send_error:{e}",
            )
            return
        future.add_done_callback(self._return_goal_response_cb)

    def _return_goal_response_cb(self, future: Any) -> None:
        try:
            gh = future.result()
        except Exception as e:
            if self._state == _State.RETURNING:
                self._finish_task(
                    return_ok=False,
                    return_reason=f"return_nav_send_error:{e}",
                )
            return
        if self._state != _State.RETURNING:
            if gh.accepted:
                self._nav_client.cancel_goal_async(gh)
            return
        if not gh.accepted:
            self._finish_task(return_ok=False, return_reason="return_goal_rejected")
            return
        with self._goal_lock:
            self._nav_handle = gh
        gh.get_result_async().add_done_callback(self._return_result_cb)

    def _return_result_cb(self, future: Any) -> None:
        if self._state != _State.RETURNING:
            return
        with self._goal_lock:
            self._nav_handle = None
        try:
            wrapper = future.result()
            status = int(wrapper.status)
        except Exception as e:
            self._finish_task(
                return_ok=False,
                return_reason=f"return_nav_result_error:{e}",
            )
            return
        publish_cmd_vel_brake(self._cmd_vel_pub, bursts=self._brake_bursts)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[object_fetch] 已返回接令位置")
            self._finish_task(return_ok=True)
            return
        self.get_logger().warning(f"[object_fetch] 返航失败 status={status}")
        self._finish_task(
            return_ok=False,
            return_reason=f"return_nav_status_{status}",
        )

    def _finish_task(self, *, return_ok: bool, return_reason: str = "") -> None:
        task_ok = self._pending_task_ok
        detail = self._pending_task_detail
        overall_ok = task_ok and return_ok
        self._state = _State.DONE if overall_ok else _State.FAILED
        event: dict[str, Any] = {
            "event": "fetch_object_done",
            "ok": overall_ok,
            "pick_ok": task_ok,
            "return_ok": return_ok,
            "object_label": self._object_label,
        }
        if task_ok:
            event["detail"] = detail
        else:
            event["reason"] = detail
        if return_reason:
            event["return_reason"] = return_reason
        self._publish_event(event)

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
