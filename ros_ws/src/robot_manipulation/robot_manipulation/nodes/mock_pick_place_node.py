from __future__ import annotations

import math
import random
import re
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from gazebo_msgs.srv import GetEntityState
except ImportError:
    GetEntityState = None  # type: ignore[assignment]

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    tf2_ros = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[misc, assignment]


_NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_pose(pose: Pose) -> float:
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _fmt_pose_xy_yaw(pose: tuple[float, float, float] | None, frame: str) -> str:
    if pose is None:
        return f"{frame}:(none)"
    x, y, yaw = pose
    return f"{frame}:(x={x:.3f},y={y:.3f},yaw_deg={math.degrees(yaw):.1f})"


def _fmt_object_pose(pose: Pose | None, frame: str) -> str:
    if pose is None:
        return f"{frame}:(none)"
    yaw = _yaw_from_pose(pose)
    p = pose.position
    return f"{frame}:(x={p.x:.3f},y={p.y:.3f},z={p.z:.3f},yaw_deg={math.degrees(yaw):.1f})"


def _fmt_locked(locked: tuple[str, tuple[float, float, float]] | None) -> str:
    if locked is None:
        return "(none)"
    frame, (x, y, z) = locked
    return f"{frame}:({x:.3f},{y:.3f},{z:.3f})"


def _canonical_label(label: str) -> str:
    raw = (label or "").strip()
    low = raw.lower().replace("_", " ")
    direct = {
        "cup": "cup",
        "mug": "cup",
        "water cup": "cup",
        "杯": "cup",
        "杯子": "cup",
        "水杯": "cup",
        "茶杯": "cup",
        "bottle": "bottle",
        "beer": "bottle",
        "coke": "bottle",
        "coke can": "bottle",
        "瓶": "bottle",
        "瓶子": "bottle",
        "可乐": "bottle",
        "可乐罐": "bottle",
        "vase": "vase",
        "花瓶": "vase",
    }
    if raw in direct:
        return direct[raw]
    return direct.get(low, low or "object")


class MockPickPlaceNode(Node):
    """Mock pick/place node whose PICK verdict is based on Gazebo ground truth."""

    def __init__(self) -> None:
        super().__init__("mock_pick_place")
        self.declare_parameter("command_topic", "/manipulation/command_text")
        self.declare_parameter("status_topic", "/manipulation/status_text")
        self.declare_parameter("default_object", "unknown_object")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("gazebo_get_entity_state_service", "/get_entity_state")
        self.declare_parameter("gazebo_reference_frame", "world")
        self.declare_parameter("cup_entities", ["coke_can", "cup", "beer"])
        self.declare_parameter("bottle_entities", ["bottle", "beer", "coke_can"])
        self.declare_parameter("vase_entities", ["vase", "Vase_01_001"])
        self.declare_parameter("robot_entities", [
            "waffle",
            "waffle::base_footprint",
            "waffle::base_link",
            "base_footprint",
            "base_link",
            "turtlebot3_waffle",
            "turtlebot3_burger",
        ])
        self.declare_parameter("pick_min_distance_m", 0.0)
        self.declare_parameter("pick_max_distance_m", 1.5)
        self.declare_parameter("pick_max_yaw_error_deg", 90.0)
        self.declare_parameter("pick_check_period_sec", 0.15)
        self.declare_parameter("pick_delay_min_sec", 2.0)
        self.declare_parameter("pick_delay_max_sec", 5.0)
        self.declare_parameter("gazebo_query_timeout_sec", 8.0)

        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._default_object = str(self.get_parameter("default_object").value)
        self._base_frame = str(self.get_parameter("robot_base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._gazebo_service = str(self.get_parameter("gazebo_get_entity_state_service").value)
        self._gazebo_frame = str(self.get_parameter("gazebo_reference_frame").value)
        self._min_dist = float(self.get_parameter("pick_min_distance_m").value)
        self._max_dist = float(self.get_parameter("pick_max_distance_m").value)
        self._max_yaw_err = math.radians(float(self.get_parameter("pick_max_yaw_error_deg").value))
        delay_a = max(float(self.get_parameter("pick_delay_min_sec").value), 0.0)
        delay_b = max(float(self.get_parameter("pick_delay_max_sec").value), 0.0)
        self._pick_delay_min = min(delay_a, delay_b)
        self._pick_delay_max = max(delay_a, delay_b)
        self._query_timeout = max(float(self.get_parameter("gazebo_query_timeout_sec").value), 1.0)
        check_period = max(float(self.get_parameter("pick_check_period_sec").value), 0.05)

        self._entity_map = {
            "cup": self._string_list_param("cup_entities"),
            "bottle": self._string_list_param("bottle_entities"),
            "vase": self._string_list_param("vase_entities"),
        }
        self._robot_entities = self._string_list_param("robot_entities")

        self._held_object = ""
        self._pending: dict[str, Any] | None = None
        self._query_future: Any | None = None
        self._query_kind = ""
        self._query_entity = ""
        self._last_service_wait_log = 0.0

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_service(Trigger, "/manipulation/mock_pick", self._on_pick)
        self.create_service(Trigger, "/manipulation/mock_place", self._on_place)
        self._check_timer = self.create_timer(check_period, self._check_pending_pick)

        self._tf_buffer = None
        self._tf_listener = None
        if tf2_ros is not None:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._gazebo_cli = None
        if GetEntityState is not None:
            self._gazebo_cli = self.create_client(GetEntityState, self._gazebo_service)

        self.get_logger().info(
            "[manipulation] mock_pick_place ready "
            f"gazebo_service={self._gazebo_service} gazebo_frame={self._gazebo_frame} "
            f"threshold_dist=[{self._min_dist:.2f},{self._max_dist:.2f}] "
            f"threshold_yaw={math.degrees(self._max_yaw_err):.1f}deg "
            f"pick_delay=[{self._pick_delay_min:.1f},{self._pick_delay_max:.1f}]s "
            f"entity_map={self._entity_map}"
        )

    def _string_list_param(self, name: str) -> list[str]:
        value = self.get_parameter(name).value
        if isinstance(value, (list, tuple)):
            return [str(v).strip() for v in value if str(v).strip()]
        text = str(value or "").strip()
        return [part.strip() for part in text.split(",") if part.strip()]

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _robot_pose_from_tf(self) -> tuple[tuple[float, float, float], str] | None:
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return ((float(t.x), float(t.y), yaw), self._map_frame)

    def _parse_locked_target(
        self, payload: str
    ) -> tuple[str, tuple[float, float, float]] | None:
        patterns = (
            rf"locked_target_frame=([^;]+);locked_target_xyz=({_NUM}),({_NUM}),({_NUM})",
            rf"target_frame=([^;]+);target_xyz=({_NUM}),({_NUM}),({_NUM})",
        )
        for pattern in patterns:
            m = re.search(pattern, payload)
            if m:
                return (
                    m.group(1).strip(),
                    (float(m.group(2)), float(m.group(3)), float(m.group(4))),
                )
        return None

    def _entities_for_label(self, label: str) -> list[str]:
        mapped = self._entity_map.get(label)
        if mapped:
            return list(mapped)
        return [label] if label and label != "object" else [self._default_object]

    def _start_pick_command(self, text: str) -> None:
        payload = text.split(":", 1)[1].strip()
        raw_label = payload.split(";", 1)[0].strip()
        label = _canonical_label(raw_label or self._default_object)
        locked = self._parse_locked_target(payload)
        candidates = self._entities_for_label(label)
        if self._held_object:
            self._publish_status(
                f"[manipulation] pick_failed reason=already_holding label={label} "
                f"held_object={self._held_object}"
            )
            return
        self._default_object = label or self._default_object
        started_mono = time.monotonic()
        delay_sec = random.uniform(self._pick_delay_min, self._pick_delay_max)
        self._pending = {
            "label": label,
            "raw_label": raw_label,
            "locked": locked,
            "object_candidates": candidates,
            "object_index": 0,
            "robot_candidates": list(self._robot_entities),
            "robot_index": 0,
            "object_entity": "",
            "object_pose": None,
            "phase": "object",
            "started_mono": started_mono,
            "ready_mono": started_mono + delay_sec,
            "delay_sec": delay_sec,
        }
        self._clear_query()
        self._publish_status(
            "[manipulation] pick_command_received "
            f"label={label} raw_label={raw_label!r} "
            f"mock_delay_sec={delay_sec:.2f} "
            f"gazebo_candidates={candidates} locked_xyz={_fmt_locked(locked)}"
        )
        self.get_logger().info(
            "[manipulation] YOLO locked target is log-only; "
            "Gazebo entity pose decides pick_success/pick_failed"
        )

    def _on_pick(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._held_object:
            resp.success = False
            resp.message = f"already holding {self._held_object}"
            return resp
        if self._pending is None:
            resp.success = False
            resp.message = "no pending PICK:<label> command"
            return resp
        self._check_pending_pick()
        resp.success = False
        resp.message = "pending asynchronous Gazebo ground-truth pick check"
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
        self._publish_status(f"[manipulation] placed {obj}")
        return resp

    def _check_pending_pick(self) -> None:
        if self._pending is None or self._held_object or self._query_future is not None:
            return
        now_mono = time.monotonic()
        ready_mono = float(self._pending.get("ready_mono", now_mono))
        if now_mono < ready_mono:
            return
        elapsed = now_mono - ready_mono
        if elapsed > self._query_timeout:
            self._fail_pending("gazebo_query_timeout")
            return
        if GetEntityState is None or self._gazebo_cli is None:
            self._fail_pending("gazebo_msgs_unavailable")
            return
        if not self._gazebo_cli.service_is_ready():
            if self._gazebo_cli.wait_for_service(timeout_sec=0.0):
                return
            mono = time.monotonic()
            if mono - self._last_service_wait_log > 1.0:
                self._last_service_wait_log = mono
                self.get_logger().info(
                    f"[manipulation] waiting_for_gazebo_service service={self._gazebo_service}"
                )
            return

        phase = str(self._pending.get("phase", "object"))
        if phase == "object":
            self._query_next_object_entity()
            return
        if phase == "robot":
            robot_tf = self._robot_pose_from_tf()
            if robot_tf is not None:
                pose, frame = robot_tf
                self._evaluate_pick(pose, frame)
                return
            self._query_next_robot_entity()

    def _query_next_object_entity(self) -> None:
        assert self._pending is not None
        candidates = list(self._pending.get("object_candidates", []))
        idx = int(self._pending.get("object_index", 0))
        if idx >= len(candidates):
            self._fail_pending("entity_not_found")
            return
        entity = str(candidates[idx])
        self._pending["object_index"] = idx + 1
        self._send_entity_query("object", entity)

    def _query_next_robot_entity(self) -> None:
        assert self._pending is not None
        candidates = list(self._pending.get("robot_candidates", []))
        idx = int(self._pending.get("robot_index", 0))
        if idx >= len(candidates):
            self._fail_pending("robot_pose_unavailable")
            return
        entity = str(candidates[idx])
        self._pending["robot_index"] = idx + 1
        self._send_entity_query("robot", entity)

    def _send_entity_query(self, kind: str, entity: str) -> None:
        if self._gazebo_cli is None or GetEntityState is None:
            self._fail_pending("gazebo_msgs_unavailable")
            return
        req = GetEntityState.Request()
        req.name = entity
        req.reference_frame = self._gazebo_frame
        self._query_kind = kind
        self._query_entity = entity
        self._query_future = self._gazebo_cli.call_async(req)
        self.get_logger().info(
            f"[manipulation] gazebo_query kind={kind} entity={entity} "
            f"reference_frame={self._gazebo_frame}"
        )
        self._query_future.add_done_callback(self._on_entity_query_done)

    def _on_entity_query_done(self, future: Any) -> None:
        if self._pending is None:
            self._clear_query()
            return
        kind = self._query_kind
        entity = self._query_entity
        self._clear_query()
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warning(
                f"[manipulation] gazebo_query_failed kind={kind} entity={entity} error={e}"
            )
            return
        if not bool(getattr(res, "success", False)):
            self.get_logger().info(
                f"[manipulation] gazebo_entity_not_found kind={kind} entity={entity}"
            )
            return
        pose = res.state.pose
        if kind == "object":
            self._pending["object_entity"] = entity
            self._pending["object_pose"] = pose
            self._pending["phase"] = "robot"
            self.get_logger().info(
                f"[manipulation] gazebo_entity={entity} "
                f"object_pose={_fmt_object_pose(pose, self._gazebo_frame)}"
            )
            return
        if kind == "robot":
            robot_pose = (
                float(pose.position.x),
                float(pose.position.y),
                _yaw_from_pose(pose),
            )
            self._evaluate_pick(robot_pose, self._gazebo_frame)

    def _clear_query(self) -> None:
        self._query_future = None
        self._query_kind = ""
        self._query_entity = ""

    def _evaluate_pick(self, robot_pose: tuple[float, float, float], robot_frame: str) -> None:
        if self._pending is None:
            return
        object_pose = self._pending.get("object_pose")
        if not isinstance(object_pose, Pose):
            self._fail_pending("entity_not_found")
            return
        label = str(self._pending.get("label", self._default_object))
        entity = str(self._pending.get("object_entity", ""))
        locked = self._pending.get("locked")
        rx, ry, yaw = robot_pose
        ox = float(object_pose.position.x)
        oy = float(object_pose.position.y)
        dist = math.hypot(ox - rx, oy - ry)
        face_yaw = math.atan2(oy - ry, ox - rx)
        yaw_err = abs(_normalize_angle(face_yaw - yaw))
        robot_s = _fmt_pose_xy_yaw(robot_pose, robot_frame)
        object_s = _fmt_object_pose(object_pose, self._gazebo_frame)
        threshold_s = (
            f"threshold_dist=[{self._min_dist:.2f},{self._max_dist:.2f}] "
            f"threshold_yaw={math.degrees(self._max_yaw_err):.1f}"
        )
        self.get_logger().info(
            f"[manipulation] gazebo_entity={entity} robot_pose={robot_s} "
            f"object_pose={object_s} dist={dist:.3f} yaw_err={math.degrees(yaw_err):.1f} "
            f"{threshold_s} locked_xyz={_fmt_locked(locked)}"
        )

        reason = ""
        if not all(math.isfinite(v) for v in (ox, oy, dist, yaw_err)):
            reason = "invalid_object_pose"
        elif dist < self._min_dist:
            reason = "too_close"
        elif dist > self._max_dist:
            reason = "too_far"
        elif yaw_err > self._max_yaw_err:
            reason = "bad_yaw"

        if reason:
            self._publish_status(
                f"[manipulation] pick_failed reason={reason} label={label} "
                f"gazebo_entity={entity} robot_pose={robot_s} object_pose={object_s} "
                f"dist={dist:.3f} yaw_err={math.degrees(yaw_err):.1f} {threshold_s} "
                f"locked_xyz={_fmt_locked(locked)}"
            )
            self._pending = None
            return

        self._held_object = label
        self._pending = None
        self._publish_status(
            f"[manipulation] pick_success label={label} gazebo_entity={entity} "
            f"robot_pose={robot_s} object_pose={object_s} dist={dist:.3f} "
            f"yaw_err={math.degrees(yaw_err):.1f} {threshold_s} "
            f"locked_xyz={_fmt_locked(locked)}"
        )

    def _fail_pending(self, reason: str) -> None:
        if self._pending is None:
            return
        label = str(self._pending.get("label", self._default_object))
        candidates = self._pending.get("object_candidates", [])
        entity = str(self._pending.get("object_entity", ""))
        locked = self._pending.get("locked")
        self._publish_status(
            f"[manipulation] pick_failed reason={reason} label={label} "
            f"gazebo_entity={entity or '(none)'} candidates={candidates} "
            f"robot_pose=(none) object_pose=(none) dist=nan yaw_err=nan "
            f"threshold_dist=[{self._min_dist:.2f},{self._max_dist:.2f}] "
            f"threshold_yaw={math.degrees(self._max_yaw_err):.1f} "
            f"locked_xyz={_fmt_locked(locked)}"
        )
        self._pending = None
        self._clear_query()

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if text.startswith("PICK:"):
            self._start_pick_command(text)
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
        if rclpy.ok():
            rclpy.shutdown()
