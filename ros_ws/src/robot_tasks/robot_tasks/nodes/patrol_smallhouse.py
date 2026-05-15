#!/usr/bin/env python3
"""Event-driven Nav2 waypoint patrol executor for the indoor map."""

import json
import math
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


# 仿真脚本默认把 TurtleBot3 生成在地图坐标 (0, 0, 0) 附近。
# 如果真实底盘或自定义 world 的初始位姿不同，请先在 RViz 使用 2D Pose Estimate，
# 或把这里改成实测初始位姿，确保 AMCL 能建立 map -> odom。
INITIAL_POSE = Waypoint("初始位姿", 0.0, 0.0, 0.0)


# 这些点按 robot_bringup/maps/map.yaml 的坐标系人工预置，目标是让机器人绕主要室内区域走一遍。
# 后续如果地图更新，优先在 RViz 里读取可通行区域坐标并同步调整这里。
WAYPOINTS = [
    Waypoint("门口区域", -7.5, 0.4, 0.0),
    Waypoint("客厅区域", -4.5, -3.3, -1.57),
    Waypoint("中部走廊区域", -0.5, 0.5, 0.0),
    Waypoint("北侧房间区域", 0.5, 3.4, 1.57),
    Waypoint("右侧公共区域", 5.5, 2.2, 0.0),
    Waypoint("右侧房间区域", 6.5, -2.4, -1.57),
    Waypoint("末端房间区域", 4.5, -4.4, 3.14),
    Waypoint("回到走廊区域", 0.0, 0.4, 3.14),
]


def make_pose(navigator: BasicNavigator, waypoint: Waypoint) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0

    # 仅绕 Z 轴旋转：yaw -> quaternion，避免额外依赖 tf_transformations。
    half_yaw = waypoint.yaw * 0.5
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


def describe_result(result: TaskResult) -> str:
    if result == TaskResult.SUCCEEDED:
        return "SUCCEEDED"
    if result == TaskResult.CANCELED:
        return "CANCELED"
    if result == TaskResult.FAILED:
        return "FAILED"
    return f"UNKNOWN({result})"


class PatrolExecutor:
    def __init__(self, navigator: BasicNavigator) -> None:
        self._navigator = navigator
        self._start_requested = False
        self._cancel_requested = False
        self._is_patrolling = False

        self._navigator.declare_parameter("task_events_topic", "/task/events")
        self._events_topic = str(
            self._navigator.get_parameter("task_events_topic").value
        )
        self._events_pub = self._navigator.create_publisher(
            String, self._events_topic, 10
        )
        self._navigator.create_subscription(
            String, self._events_topic, self._on_task_event, 10
        )

        self._navigator.get_logger().info(
            f"[patrol_smallhouse] 已订阅任务事件: {self._events_topic}"
        )

    def _on_task_event(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return

        try:
            event = json.loads(raw)
        except json.JSONDecodeError:
            return

        if not isinstance(event, dict):
            return

        event_name = str(event.get("event", "")).strip()
        if event_name == "patrol_start":
            if self._is_patrolling:
                self._navigator.get_logger().info(
                    "[patrol_smallhouse] 巡航已在进行中，忽略重复启动"
                )
                return
            self._start_requested = True
            scope = str(event.get("scope", "room_default"))
            self._navigator.get_logger().info(
                f"[patrol_smallhouse] 收到 patrol_start，准备开始巡航，scope={scope}"
            )
            return

        if event_name == "stop" and self._is_patrolling:
            self._cancel_requested = True
            self._navigator.get_logger().info(
                "[patrol_smallhouse] 收到 stop，准备取消当前巡航"
            )

    def run_forever(self) -> None:
        self._navigator.get_logger().info(
            "发布 AMCL 初始位姿，等待 Nav2 lifecycle 节点进入 active..."
        )
        self._navigator.setInitialPose(make_pose(self._navigator, INITIAL_POSE))
        self._navigator.waitUntilNav2Active(localizer="amcl")
        self._navigator.get_logger().info("Nav2 已就绪，等待 patrol_start 任务事件。")

        while rclpy.ok():
            rclpy.spin_once(self._navigator, timeout_sec=0.2)
            if self._start_requested and not self._is_patrolling:
                self._start_requested = False
                self._run_patrol_once()

    def _run_patrol_once(self) -> None:
        self._is_patrolling = True
        self._cancel_requested = False
        completed_waypoints = 0
        has_failure = False

        self._navigator.get_logger().info("[patrol_smallhouse] 开始执行全屋巡航")
        try:
            for index, waypoint in enumerate(WAYPOINTS, start=1):
                if self._cancel_requested:
                    break

                self._navigator.get_logger().info(
                    f"前往第 {index}/{len(WAYPOINTS)} 个 waypoint：{waypoint.name} "
                    f"(x={waypoint.x:.2f}, y={waypoint.y:.2f}, yaw={waypoint.yaw:.2f})"
                )

                self._navigator.goToPose(make_pose(self._navigator, waypoint))
                last_feedback_log_time = 0.0

                while not self._navigator.isTaskComplete():
                    rclpy.spin_once(self._navigator, timeout_sec=0.2)
                    if self._cancel_requested:
                        self._navigator.cancelTask()

                    feedback = self._navigator.getFeedback()
                    now = time.monotonic()
                    if feedback is not None and now - last_feedback_log_time > 5.0:
                        last_feedback_log_time = now
                        self._navigator.get_logger().info(
                            f"{waypoint.name} 剩余距离约 {feedback.distance_remaining:.2f} m"
                        )

                result = self._navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    completed_waypoints += 1
                    self._navigator.get_logger().info(f"已到达：{waypoint.name}")
                else:
                    has_failure = True
                    self._navigator.get_logger().warn(
                        f"{waypoint.name} 导航结果：{describe_result(result)}，继续尝试下一个点。"
                    )

                if self._cancel_requested:
                    self._navigator.get_logger().info(
                        "[patrol_smallhouse] 巡航被 stop 事件打断"
                    )
                    break

            patrol_ok = (not self._cancel_requested) and (not has_failure)
            done_event = {
                "event": "patrol_done",
                "ok": patrol_ok,
                "canceled": self._cancel_requested,
                "completed_waypoints": completed_waypoints,
                "total_waypoints": len(WAYPOINTS),
            }
            payload = json.dumps(done_event, ensure_ascii=False)
            self._events_pub.publish(String(data=payload))
            self._navigator.get_logger().info(
                f"[patrol_smallhouse] -> {self._events_topic}: {payload}"
            )
            self._navigator.get_logger().info("[patrol_smallhouse] 本次巡航结束")
        finally:
            self._is_patrolling = False
            self._cancel_requested = False


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()
    executor = PatrolExecutor(navigator)

    try:
        executor.run_forever()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
