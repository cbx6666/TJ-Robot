from __future__ import annotations

import math
import re

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import tf2_ros
    from tf2_ros import TransformException
except ImportError:
    tf2_ros = None  # type: ignore[assignment]
    TransformException = Exception  # type: ignore[misc, assignment]


def _normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class MockPickPlaceNode(Node):
    """Mock 抓放：仅在机器人距目标合适且朝向目标时判定抓取成功。"""

    def __init__(self) -> None:
        super().__init__("mock_pick_place")
        self.declare_parameter("command_topic", "/manipulation/command_text")
        self.declare_parameter("status_topic", "/manipulation/status_text")
        self.declare_parameter("default_object", "unknown_object")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pick_min_distance_m", 0.35)
        self.declare_parameter("pick_max_distance_m", 0.85)
        self.declare_parameter("pick_max_yaw_error_deg", 28.0)
        self.declare_parameter("pick_check_period_sec", 0.15)

        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._default_object = str(self.get_parameter("default_object").value)
        self._base_frame = str(self.get_parameter("robot_base_frame").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._min_dist = float(self.get_parameter("pick_min_distance_m").value)
        self._max_dist = float(self.get_parameter("pick_max_distance_m").value)
        self._max_yaw_err = math.radians(float(self.get_parameter("pick_max_yaw_error_deg").value))
        check_period = max(float(self.get_parameter("pick_check_period_sec").value), 0.05)

        self._held_object = ""
        self._pending: dict[str, float | str] | None = None
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

        self.get_logger().info(
            f"mock_pick_place: dist=[{self._min_dist},{self._max_dist}]m "
            f"yaw_err<={math.degrees(self._max_yaw_err):.0f}°"
        )

    def _publish_status(self, text: str) -> None:
        self._status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _robot_xy_yaw(self) -> tuple[float, float, float] | None:
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.4),
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

    def _parse_target(self, payload: str) -> tuple[str, float, float, float] | None:
        m = re.search(
            r"target_frame=([^;]+);target_xyz=([-\d.]+),([-\d.]+),([-\d.]+)",
            payload,
        )
        if not m:
            return None
        frame = m.group(1).strip()
        return (
            frame,
            float(m.group(2)),
            float(m.group(3)),
            float(m.group(4)),
        )

    def _geometry_ok(self, tx: float, ty: float) -> tuple[bool, str]:
        robot = self._robot_xy_yaw()
        if robot is None:
            return False, "no_tf"
        rx, ry, yaw = robot
        dist = math.hypot(tx - rx, ty - ry)
        face_yaw = math.atan2(ty - ry, tx - rx)
        yaw_err = abs(_normalize_angle(face_yaw - yaw))
        if dist < self._min_dist:
            return False, f"too_close dist={dist:.2f}m"
        if dist > self._max_dist:
            return False, f"too_far dist={dist:.2f}m"
        if yaw_err > self._max_yaw_err:
            return False, f"bad_yaw err={math.degrees(yaw_err):.1f}° dist={dist:.2f}m"
        return True, f"ok dist={dist:.2f}m yaw_err={math.degrees(yaw_err):.1f}°"

    def _on_pick(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        if self._held_object:
            resp.success = False
            resp.message = f"already holding {self._held_object}"
            return resp
        if self._pending is None:
            resp.success = False
            resp.message = "no pending pick with target"
            return resp
        tx = float(self._pending["tx"])
        ty = float(self._pending["ty"])
        ok, detail = self._geometry_ok(tx, ty)
        if not ok:
            resp.success = False
            resp.message = f"pick_failed:{detail}"
            self._publish_status(f"[manipulation] pick_failed {detail}")
            self._pending = None
            return resp
        obj = str(self._pending.get("obj", self._default_object))
        self._held_object = obj
        self._pending = None
        resp.success = True
        resp.message = f"picked {obj} ({detail})"
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

    def _check_pending_pick(self) -> None:
        if self._pending is None or self._held_object:
            return
        tx = float(self._pending["tx"])
        ty = float(self._pending["ty"])
        ok, detail = self._geometry_ok(tx, ty)
        if ok:
            self._on_pick(Trigger.Request(), Trigger.Response())

    def _on_command(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        if text.startswith("PICK:"):
            payload = text.split(":", 1)[1].strip()
            obj = payload.split(";", 1)[0].strip()
            self._default_object = obj or self._default_object
            parsed = self._parse_target(payload)
            if parsed is None:
                self._publish_status(f"[manipulation] pick_failed no_target_in_command")
                return
            frame, tx, ty, tz = parsed
            if frame != self._map_frame:
                self._publish_status(f"[manipulation] pick_failed bad_frame={frame}")
                return
            self._pending = {"obj": obj, "tx": tx, "ty": ty, "tz": tz}
            self._publish_status(
                f"[manipulation] target_hint: {payload} (等待到位: "
                f"dist∈[{self._min_dist},{self._max_dist}]m, 正对目标)"
            )
            ok, detail = self._geometry_ok(tx, ty)
            if ok:
                self._on_pick(Trigger.Request(), Trigger.Response())
            else:
                self.get_logger().info(f"[manipulation] 尚未到位: {detail}")
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
