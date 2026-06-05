"""Nav2 巡航调度器。

这个文件只负责“巡航编排”，不直接实现地图解析或 marker 展示。核心目标是把三件容易混在
一起的事情拆开：

1. goal 切换：同一个巡视点可以尝试不同 approach pose。
2. path 切换：每个候选 goal 都先 getPath，命中失败路径段就拒绝。
3. pose 脱困：真正卡住后先 cancel Nav2，再自己发布 cmd_vel 离开局部困境。

代码整体按状态机推进，避免恢复逻辑散落在多个 if 分支里。日志字段尽量保持机器可 grep，
方便从 nav2.launch.log 里复盘“为什么卡住、为什么取消、为什么拒绝某条路径、最终发了哪个 goal”。

当前真实入口链路：

`patrol_waypoints.py` -> `NavigationManager.run()` -> `_run_waypoint()` 状态机。

旧的独立检测/恢复模块已移除；卡住判断、failed corridor 和主动 cmd_vel escape 都集中在
本文件，避免出现两套恢复逻辑同时存在却只有一套生效的情况。
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from coverage_planner import CoveragePlanner, CoveragePlannerConfig
from patrol_markers import PatrolMarkerPublisher
from waypoint_utils import (
    Pose2D,
    Waypoint,
    make_pose_stamped,
    normalize_angle,
    yaw_from_quaternion,
)


# =============================================================================
# 常量与状态机
# =============================================================================
# fallback waypoint 只在 coverage planner 关闭或地图读取失败时使用。正常情况下，
# `_build_waypoints()` 会从静态地图生成覆盖式巡航点。
FALLBACK_WAYPOINTS = [
    Waypoint("entry_area", -7.2, 0.6, 0.0),
    Waypoint("left_room_lower", -7.0, -3.2, -1.57),
    Waypoint("living_room", -4.0, -2.6, -1.57),
    Waypoint("left_room_upper", -6.8, 2.4, 1.57),
    Waypoint("middle_corridor", -0.7, 0.7, 0.0),
    Waypoint("north_room", 0.5, 3.1, 1.57),
    Waypoint("right_common_area", 5.0, 2.0, 0.0),
    Waypoint("right_room", 5.7, -2.0, -1.57),
    Waypoint("end_room", 4.0, -3.8, 3.14),
    Waypoint("return_corridor", 0.0, 0.7, 3.14),
]


class NavState(str, Enum):
    """巡航恢复状态机。

    状态名直接进入日志 `NAV_STATE_CHANGE`，所以保持英文、稳定、可 grep。
    """

    IDLE = "IDLE"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    STUCK_DETECTED = "STUCK_DETECTED"
    CANCELING_GOAL = "CANCELING_GOAL"
    ESCAPING = "ESCAPING"
    REPLAN_AFTER_ESCAPE = "REPLAN_AFTER_ESCAPE"
    SELECT_ALTERNATIVE = "SELECT_ALTERNATIVE"
    SKIP_WAYPOINT = "SKIP_WAYPOINT"
    DONE = "DONE"


# =============================================================================
# 运行期数据结构
# =============================================================================
# 这些 dataclass 都是状态机内部流转的数据容器：odom/cmd_vel 采样用于判断是否真动，
# RoutePlan 表示已经 getPath 的候选路线，FailedCorridor 表示失败路径段黑名单。
@dataclass(frozen=True)
class OdomSample:
    """一条 odom 采样。

    stamp 使用 monotonic 时间，避免仿真 `/clock` 暂停或跳变影响本地超时判断。
    """

    stamp: float
    pose: Pose2D


@dataclass(frozen=True)
class CmdSample:
    """一条 cmd_vel 活跃度采样。

    卡死检测可以要求“最近确实有控制输出”，从而区分 Nav2 没发速度和发了速度但车不动。
    """

    stamp: float
    active: bool


@dataclass(frozen=True)
class FailedCorridor:
    """失败路径段黑名单。

    这里存的是 global path 上的一段 polyline，而不是单个失败点。检查候选路径时，只要路径
    点落入 polyline 任意采样点的 radius 内，就认为这条候选路线复用了失败走廊。
    """

    corridor_id: int
    points: list[tuple[float, float]]
    radius_m: float
    created_at: float
    reason: str
    path_signature: str


@dataclass(frozen=True)
class RoutePlan:
    """一次 getPath 的结果。

    RoutePlan 是唯一允许被 `_send_goal()` 发送给 Nav2 的对象；它已经经过路径长度、签名、
    failed corridor 等检查。
    """

    goal: Waypoint
    goal_id: str
    path_points: list[tuple[float, float]]
    path_len_m: float
    signature: str
    endpoint_error_m: float
    failed_overlap_score: float


@dataclass(frozen=True)
class NavOutcome:
    """导航监控循环的退出结果。

    `status == "reached"` 表示目标成功；其他状态都会进入恢复状态机。
    """

    status: str
    reason: str
    pose: Pose2D | None
    moved_m: float = 0.0
    yaw_delta_rad: float = 0.0
    window_sec: float = 0.0
    cmd_ratio: float = 0.0


@dataclass(frozen=True)
class EscapeStep:
    """一个直接 cmd_vel 脱困动作。"""

    mode: str
    linear_x: float
    angular_z: float
    duration_sec: float


# =============================================================================
# 巡航主控
# =============================================================================
class NavigationManager:
    """巡航主控类。

    BasicNavigator 负责和 Nav2 action/service 通信；本类只做策略编排：
    - 建立候选 goal；
    - 调用 getPath 做发车前检查；
    - 监控 odom/cmd_vel 判断是否卡住；
    - 卡住后执行 cancel -> escape -> replan。
    """

    def __init__(self, navigator: BasicNavigator) -> None:
        self._navigator = navigator
        self._declare_parameters()

        self._markers = PatrolMarkerPublisher(navigator)

        # ---------- 通用巡航参数 ----------
        # waypoint_timeout_* 是“到目标距离没有持续下降”的兜底超时；它和 odom stuck 检测互补。
        self._goal_timeout_sec = self._float_param("waypoint_timeout_sec", 6.0)
        self._timeout_progress_m = self._float_param("waypoint_timeout_progress_m", 0.08)
        self._odom_topic = self._str_param("odom_topic", "/odom")

        # ---------- 主动卡住检测 ----------
        # 只在 NAVIGATING 状态下生效，并且窗口从 goal accepted 之后开始算，避免把发 goal 前
        # 的静止历史误判成“刚发 goal 就卡住”。
        self._stuck_enabled = self._bool_param("patrol_stuck_enabled", True)
        self._stuck_min_progress_m = self._float_param("patrol_stuck_min_progress_m", 0.08)
        self._stuck_window_sec = self._float_param("patrol_stuck_window_sec", 4.0)
        self._stuck_cmd_required = self._bool_param("patrol_stuck_cmd_required", True)
        self._near_goal_tolerance_m = self._float_param(
            "patrol_stuck_near_goal_tolerance_m", 0.35
        )

        # ---------- 失败路径段黑名单 ----------
        # failed corridor 记录的是 active global path 在 stuck pose 前后的一段 polyline。
        # 这比只记一个失败点更稳，因为墙边/窄通道常常是一整段路线都有问题。
        self._failed_corridor_enabled = self._bool_param(
            "patrol_failed_corridor_enabled", True
        )
        self._failed_corridor_radius_m = self._float_param(
            "patrol_failed_corridor_radius_m", 0.60
        )
        self._failed_corridor_behind_m = self._float_param(
            "patrol_failed_corridor_behind_m", 0.80
        )
        self._failed_corridor_ahead_m = self._float_param(
            "patrol_failed_corridor_ahead_m", 2.00
        )
        self._failed_corridor_ttl_sec = self._float_param(
            "patrol_failed_corridor_ttl_sec", 240.0
        )
        self._failed_corridor_max = self._int_param("patrol_failed_corridor_max", 12)

        # ---------- approach goal 生成 ----------
        # 原始 waypoint 不一定是最好的到达姿态；这里在同一巡视点附近生成可替代 approach pose。
        self._approach_offsets_enabled = self._bool_param(
            "patrol_approach_offsets_enabled", True
        )
        self._approach_offset_m = self._float_param("patrol_approach_offset_m", 0.60)
        self._approach_yaw_variants_enabled = self._bool_param(
            "patrol_approach_yaw_variants_enabled", True
        )

        # ---------- 直接 cmd_vel 脱困 ----------
        # 必须先 cancel 当前 Nav2 goal，再发布这些速度，否则 Nav2 controller 可能抢占 cmd_vel。
        self._escape_enabled = self._bool_param("patrol_escape_enabled", True)
        self._escape_cmd_topic = self._str_param("patrol_escape_cmd_topic", "/cmd_vel")
        self._escape_min_move_m = self._float_param("patrol_escape_min_move_m", 0.10)
        self._escape_max_attempts = self._int_param("patrol_escape_max_attempts", 2)
        self._escape_sequence_attempts = self._int_param("patrol_escape_sequence_attempts", 1)
        self._escape_backward_speed = self._float_param(
            "patrol_escape_backward_speed", 0.12
        )
        self._escape_forward_speed = self._float_param(
            "patrol_escape_forward_speed", 0.08
        )
        self._escape_turn_speed = self._float_param("patrol_escape_turn_speed", 0.50)
        self._escape_arc_turn_speed = max(self._escape_turn_speed, 0.45)
        self._escape_backward_duration_sec = self._float_param(
            "patrol_escape_backward_duration_sec", 1.0
        )
        self._escape_rotate_duration_sec = self._float_param(
            "patrol_escape_rotate_duration_sec", 0.7
        )
        self._escape_arc_duration_sec = self._float_param(
            "patrol_escape_arc_duration_sec", 1.0
        )

        # ---------- “action accepted 但车体不动”的保护 ----------
        # 这类问题通常来自速度 mux、安全层、controller/lifecycle 状态或仿真暂停。
        self._no_motion_after_goal_sec = self._float_param(
            "patrol_no_motion_after_goal_sec", 3.0
        )
        self._no_motion_max_count = self._int_param("patrol_no_motion_max_count", 2)

        # ---------- 运行期状态 ----------
        self._odom_history: deque[OdomSample] = deque()
        self._cmd_history: deque[CmdSample] = deque()
        self._latest_pose: Pose2D | None = None
        self._failed_corridors: list[FailedCorridor] = []
        self._failed_path_signatures: dict[str, float] = {}
        self._last_failed_signature = ""
        self._corridor_seq = 0
        self._active_route: RoutePlan | None = None
        self._active_goal: Waypoint | None = None
        self._no_motion_count = 0

        # cmd_pub 用于主动 escape；cmd_vel subscription 只用于判断 Nav2/controller 是否在输出速度。
        cmd_monitor_topic = self._str_param("cmd_vel_topic", self._escape_cmd_topic)
        self._cmd_pub = navigator.create_publisher(Twist, self._escape_cmd_topic, 10)
        navigator.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        navigator.create_subscription(Twist, cmd_monitor_topic, self._on_cmd_vel, 20)

        self._patrol_publish_initial_pose = self._bool_param(
            "patrol_publish_initial_pose_on_start", False
        )
        self._patrol_seed_pose_from_tf = self._bool_param(
            "patrol_seed_initial_pose_from_tf", True
        )
        self._map_frame = self._str_param("map_frame", "map")
        self._base_frame = self._str_param("robot_base_frame", "base_footprint")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, navigator)

        self._patrol_trigger_mode = self._str_param("patrol_trigger_mode", "auto").strip().lower()
        self._task_events_topic = self._str_param("task_events_topic", "/task/events")
        self._events_pub = navigator.create_publisher(String, self._task_events_topic, 10)
        self._patrol_start_requested = False
        self._patrol_cancel_requested = False
        self._is_patrolling = False
        if self._patrol_trigger_mode == "event":
            navigator.create_subscription(
                String, self._task_events_topic, self._on_task_event, 10
            )
            navigator.get_logger().info(
                f"Patrol trigger=event on {self._task_events_topic} "
                "(publish patrol_start to begin; same as run_full_system voice)"
            )

    def run(self) -> None:
        """启动巡航：auto=launch 后立即巡；event=等待 /task/events 的 patrol_start。"""

        if self._patrol_trigger_mode == "event":
            self._run_event_driven()
        else:
            self._run_auto_once()

    def _run_auto_once(self) -> None:
        if not self._ensure_nav2_ready():
            return
        self._execute_patrol_loop()

    def _run_event_driven(self) -> None:
        # 全链路 run_full_system 使用 defer_navigation_autostart：导航栈晚于本节点启动。
        # 若在启动时立刻 wait Nav2，会在 map_server/导航 lifecycle 完成前超时（非正常现象）。
        self._navigator.get_logger().info(
            f"Event mode: waiting for patrol_start on {self._task_events_topic} "
            "(Nav2 readiness is checked when patrol begins, not at node start)"
        )
        while rclpy.ok():
            rclpy.spin_once(self._navigator, timeout_sec=0.2)
            if self._patrol_start_requested and not self._is_patrolling:
                self._patrol_start_requested = False
                if not self._ensure_nav2_ready():
                    self._publish_patrol_done(
                        ok=False,
                        canceled=False,
                        completed=0,
                        total=0,
                    )
                    continue
                if self._patrol_cancel_requested:
                    self._navigator.get_logger().info(
                        "PATROL_ABORT before loop (stop during Nav2 wait, e.g. object found)"
                    )
                    self._publish_patrol_done(
                        ok=False,
                        canceled=True,
                        completed=0,
                        total=0,
                    )
                    self._patrol_cancel_requested = False
                    continue
                self._execute_patrol_loop()

    def _current_map_pose_waypoint(self) -> Waypoint | None:
        """从 TF 读取当前 map 位姿，用于可选的 AMCL 播种（不移动 Gazebo 模型）。"""

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = yaw_from_quaternion(q.z, q.w)
        return Waypoint("current_tf", t.x, t.y, yaw)

    def _seed_initial_pose_before_patrol(self) -> None:
        """巡检开始前是否向 AMCL 发布 initial pose。

        默认关闭：避免在 RViz/Nav2 已把机器人移到别处后，取物/巡检仍把地图坐标拉回 (0,0)，
        而 Gazebo 模型仍在原地的“地图与仿真脱节”现象。
        """

        if not self._patrol_publish_initial_pose:
            self._navigator.get_logger().info(
                "PATROL_SKIP_INITIAL_POSE: 保持当前 AMCL 位姿（不重置到 map 原点）"
            )
            return
        waypoint = Waypoint("initial_pose", 0.0, 0.0, 0.0)
        if self._patrol_seed_pose_from_tf:
            current = self._current_map_pose_waypoint()
            if current is not None:
                waypoint = current
                self._navigator.get_logger().info(
                    "PATROL_INITIAL_POSE from TF "
                    f"({waypoint.x:.2f}, {waypoint.y:.2f}, yaw={waypoint.yaw:.2f})"
                )
            else:
                self._navigator.get_logger().warning(
                    "PATROL_INITIAL_POSE: TF 不可用，回退 map 原点 (0,0,0)"
                )
        else:
            self._navigator.get_logger().info("PATROL_INITIAL_POSE fixed (0,0,0)")
        self._navigator.setInitialPose(make_pose_stamped(self._navigator, waypoint))

    def _ensure_nav2_ready(self) -> bool:
        default_timeout = 120.0 if self._patrol_trigger_mode == "event" else 45.0
        timeout = self._float_param("nav2_ready_timeout_sec", default_timeout)
        self._navigator.get_logger().info(
            "Waiting for Nav2 lifecycle nodes to become active "
            f"(timeout={timeout:g}s)"
        )
        self._seed_initial_pose_before_patrol()
        if not self._wait_for_nav2_ready(timeout_sec=timeout):
            self._navigator.get_logger().error("PATROL_ABORT_NAV2_NOT_READY")
            return False
        self._wait_for_initial_odom(timeout_sec=12.0)
        return True

    def _execute_patrol_loop(self) -> None:
        if self._patrol_cancel_requested:
            self._navigator.get_logger().info("PATROL_SKIP loop entry (already canceled)")
            if self._patrol_trigger_mode == "event":
                self._publish_patrol_done(
                    ok=False,
                    canceled=True,
                    completed=0,
                    total=0,
                )
            self._patrol_cancel_requested = False
            return
        self._is_patrolling = True
        self._patrol_cancel_requested = False
        completed = 0
        has_failure = False
        total = 0
        try:
            self._navigator.get_logger().info("PATROL_START coverage navigation")
            waypoints = self._build_waypoints()
            total = len(waypoints)
            self._navigator.get_logger().info(f"PATROL_WAYPOINT_COUNT count={total}")
            for index, waypoint in enumerate(waypoints, start=1):
                if self._patrol_cancel_requested:
                    self._navigator.get_logger().info("PATROL_CANCELED by stop event")
                    break
                self._navigator.get_logger().info(
                    f"WAYPOINT_START {index}/{total} name={waypoint.name} "
                    f"x={waypoint.x:.2f} y={waypoint.y:.2f} yaw={waypoint.yaw:.2f}"
                )
                reached = self._run_waypoint(waypoint, index, total)
                if reached:
                    completed += 1
                    self._markers.publish_done(waypoint)
                else:
                    has_failure = True
                    self._markers.publish_skipped(waypoint)
                if self._patrol_cancel_requested:
                    break
            patrol_ok = (not self._patrol_cancel_requested) and (not has_failure)
            self._navigator.get_logger().info(
                f"PATROL_FINISHED ok={patrol_ok} completed={completed}/{total}"
            )
        finally:
            if self._patrol_trigger_mode == "event":
                patrol_ok = (not self._patrol_cancel_requested) and (not has_failure)
                self._publish_patrol_done(
                    ok=patrol_ok,
                    canceled=self._patrol_cancel_requested,
                    completed=completed,
                    total=total,
                )
            self._is_patrolling = False
            self._patrol_cancel_requested = False

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
                if self._patrol_cancel_requested:
                    self._patrol_start_requested = True
                    self._navigator.get_logger().info(
                        "PATROL_EVENT_START queued until current patrol stops"
                    )
                    return
                self._navigator.get_logger().info(
                    "Patrol already running; ignore duplicate patrol_start"
                )
                return
            scope = str(event.get("scope", "room_default"))
            self._navigator.get_logger().info(
                f"PATROL_EVENT_START scope={scope}"
            )
            self._patrol_start_requested = True
            return
        if event_name == "stop":
            source = str(event.get("source", "")).strip()
            self._patrol_cancel_requested = True
            self._patrol_start_requested = False
            if self._is_patrolling:
                self._navigator.get_logger().info(
                    f"PATROL_EVENT_STOP cancel requested source={source or 'unknown'}"
                )
            else:
                self._navigator.get_logger().info(
                    f"PATROL_EVENT_STOP (pre-loop) source={source or 'unknown'}"
                )
            return

    def _publish_patrol_done(
        self,
        *,
        ok: bool,
        canceled: bool,
        completed: int,
        total: int,
    ) -> None:
        payload = json.dumps(
            {
                "event": "patrol_done",
                "ok": ok,
                "canceled": canceled,
                "completed_waypoints": completed,
                "total_waypoints": total,
            },
            ensure_ascii=False,
        )
        self._events_pub.publish(String(data=payload))
        self._navigator.get_logger().info(f"PATROL_EVENT_DONE -> {self._task_events_topic}: {payload}")

    def _run_waypoint(self, waypoint: Waypoint, index: int, total: int) -> bool:
        """执行单个 waypoint 的状态机。

        状态流转的关键约束：
        - `PLANNING/REPLAN_AFTER_ESCAPE/SELECT_ALTERNATIVE` 只负责选路线，不直接发车；
        - `NAVIGATING` 只监控 Nav2 和 odom，不做恢复动作；
        - `STUCK_DETECTED` 固化失败现场：记录 failed corridor 和 path signature；
        - `CANCELING_GOAL` 确保 Nav2 不再控制 cmd_vel；
        - `ESCAPING` 发布自己的 Twist，让机器人先离开局部困境；
        - escape 成功后回到 `REPLAN_AFTER_ESCAPE`，从新位姿重新 getPath。
        """

        state = NavState.IDLE
        selected_route: RoutePlan | None = None
        stuck_outcome: NavOutcome | None = None
        skip_reason = ""
        escape_events = 0
        goal_active = False
        self._active_route = None
        self._active_goal = None
        self._no_motion_count = 0

        state = self._transition(state, NavState.PLANNING, "waypoint_start")
        while rclpy.ok() and state != NavState.DONE:
            if self._patrol_cancel_requested:
                if goal_active:
                    self._cancel_active_goal(selected_route, "patrol_stop_event")
                    goal_active = False
                self._navigator.get_logger().info(
                    f"WAYPOINT_CANCELED name={waypoint.name} reason=patrol_stop_event "
                    f"state={state.value}"
                )
                self._transition(state, NavState.DONE, "patrol_canceled")
                return False
            # 规划类状态共用一套路线选择逻辑。这里不会直接沿用旧路径，必须重新 getPath。
            if state in (
                NavState.PLANNING,
                NavState.REPLAN_AFTER_ESCAPE,
                NavState.SELECT_ALTERNATIVE,
            ):
                if state == NavState.REPLAN_AFTER_ESCAPE:
                    self._navigator.get_logger().info(
                        f"REPLAN_AFTER_ESCAPE waypoint={waypoint.name}"
                    )
                if state == NavState.SELECT_ALTERNATIVE:
                    self._navigator.get_logger().info(
                        f"SELECT_ALTERNATIVE waypoint={waypoint.name}"
                    )

                selected_route = self._select_route(waypoint)
                if selected_route is None:
                    skip_reason = "no_viable_route"
                    state = self._transition(
                        state, NavState.SKIP_WAYPOINT, "all_candidate_routes_rejected"
                    )
                    continue

                # 路径检查期间会 spin ROS 回调；stop 可能就在这时到达。
                # 必须在发送 goal 前复核，避免巡检抢占取物导航。
                if self._patrol_cancel_requested:
                    continue

                if not self._send_goal(selected_route, index, total):
                    self._mark_failed_signature(selected_route.signature)
                    self._last_failed_signature = selected_route.signature
                    state = self._transition(
                        state, NavState.SELECT_ALTERNATIVE, "goal_send_failed"
                    )
                    continue

                goal_active = True
                state = self._transition(state, NavState.NAVIGATING, "goal_accepted")
                continue

            # 监控类状态只观察：Nav2 result、feedback 距离、odom 窗口、cmd_vel 活跃度。
            if state == NavState.NAVIGATING:
                if selected_route is None:
                    skip_reason = "internal_missing_route"
                    state = self._transition(state, NavState.SKIP_WAYPOINT, skip_reason)
                    continue
                outcome = self._monitor_navigation(selected_route)
                goal_active = False
                if outcome.status == "canceled":
                    self._navigator.get_logger().info(
                        f"WAYPOINT_CANCELED name={waypoint.name} reason={outcome.reason}"
                    )
                    state = self._transition(state, NavState.DONE, "patrol_canceled")
                    return False
                if outcome.status == "reached":
                    self._navigator.get_logger().info(
                        f"WAYPOINT_REACHED name={selected_route.goal.name} "
                        f"signature={selected_route.signature}"
                    )
                    state = self._transition(state, NavState.DONE, "waypoint_reached")
                    return True

                stuck_outcome = outcome
                state = self._transition(state, NavState.STUCK_DETECTED, outcome.reason)
                continue

            # 卡住被确认后，先记录失败路径段，再进入 cancel。记录必须发生在 cancel 前，
            # 因为 active route 是“这次真正发给 Nav2 的全局路径”。
            if state == NavState.STUCK_DETECTED:
                if selected_route is None:
                    skip_reason = "stuck_without_active_route"
                    state = self._transition(state, NavState.SKIP_WAYPOINT, skip_reason)
                    continue
                pose = stuck_outcome.pose if stuck_outcome else self._latest_pose
                reason = stuck_outcome.reason if stuck_outcome else "unknown_stuck"
                self._markers.publish_stuck(pose, reason)
                self._add_failed_corridor(pose, selected_route.path_points, reason)
                self._mark_failed_signature(selected_route.signature)
                self._last_failed_signature = selected_route.signature
                state = self._transition(state, NavState.CANCELING_GOAL, reason)
                continue

            # cancel 后才能 escape，避免 Nav2 controller 和本节点同时抢 /cmd_vel。
            if state == NavState.CANCELING_GOAL:
                reason = stuck_outcome.reason if stuck_outcome else "stuck"
                self._cancel_active_goal(selected_route, reason)
                if escape_events >= max(self._escape_max_attempts, 1):
                    skip_reason = "escape_limit_reached"
                    state = self._transition(state, NavState.SKIP_WAYPOINT, skip_reason)
                else:
                    state = self._transition(state, NavState.ESCAPING, reason)
                continue

            # escape 成功才允许重新规划；如果 odom 没有有效变化，就跳过当前 waypoint。
            if state == NavState.ESCAPING:
                escape_events += 1
                stuck_pose = stuck_outcome.pose if stuck_outcome else self._latest_pose
                if not self._escape_enabled:
                    self._navigator.get_logger().warn("ESCAPE_FAILED reason=disabled")
                    skip_reason = "escape_disabled"
                    state = self._transition(state, NavState.SKIP_WAYPOINT, skip_reason)
                    continue

                if self._run_escape(stuck_pose, escape_events):
                    state = self._transition(
                        state, NavState.REPLAN_AFTER_ESCAPE, "escape_success"
                    )
                else:
                    skip_reason = "escape_failed_no_pose_progress"
                    state = self._transition(state, NavState.SKIP_WAYPOINT, skip_reason)
                continue

            # SKIP_WAYPOINT 是明确终态：不再对当前 waypoint 无限换点或无限规划。
            if state == NavState.SKIP_WAYPOINT:
                self._navigator.get_logger().warn(
                    f"WAYPOINT_SKIPPED_STUCK name={waypoint.name} reason={skip_reason}"
                )
                state = self._transition(state, NavState.DONE, skip_reason)
                return False

        return False

    def _send_goal(self, route: RoutePlan, index: int, total: int) -> bool:
        """把已经通过检查的 RoutePlan 发给 Nav2。

        注意：本函数不做路径过滤。调用方必须先通过 `_select_route()` 得到 RoutePlan，
        这样可以保证所有 `GOAL_SENT` 日志之前一定有对应的 `ROUTE_SELECTED`。
        """

        self._active_route = route
        self._active_goal = route.goal
        self._markers.publish_goal(route.goal)
        self._navigator.get_logger().info(
            f"GOAL_SENT index={index}/{total} goal_id={route.goal_id} "
            f"x={route.goal.x:.2f} y={route.goal.y:.2f} yaw={route.goal.yaw:.2f} "
            f"signature={route.signature}"
        )
        try:
            self._navigator.goToPose(make_pose_stamped(self._navigator, route.goal))
        except Exception as exc:
            self._navigator.get_logger().error(
                f"GOAL_SEND_FAILED goal_id={route.goal_id} signature={route.signature} error={exc}"
            )
            return False

        self._navigator.get_logger().info(
            f"NAV_ACCEPTED goal_id={route.goal_id} signature={route.signature}"
        )
        return True

    def _monitor_navigation(self, route: RoutePlan) -> NavOutcome:
        """监控一个 Nav2 goal 的执行过程。

        返回值只表达“为什么退出监控”：
        - reached：Nav2 成功到达；
        - accepted_but_no_motion：action 接收后车体没动；
        - no_progress：odom 窗口内位移不足；
        - goal_progress_timeout：feedback 距离长期没有改善；
        - nav2_result_*：Nav2 action 自己返回失败/取消。

        这个函数不 cancel、不发布 cmd_vel，只把事实报告给状态机。
        """

        start_time = time.monotonic()
        start_pose = self._latest_pose
        best_distance: float | None = None
        last_progress_time = start_time
        last_log_time = 0.0
        started_moving = False
        no_motion_logged = False

        while rclpy.ok() and not self._navigator.isTaskComplete():
            rclpy.spin_once(self._navigator, timeout_sec=0.05)
            if self._patrol_cancel_requested:
                self._navigator.cancelTask()
                return NavOutcome("canceled", "patrol_stop_event", self._latest_pose)
            now = time.monotonic()
            feedback = self._navigator.getFeedback()
            distance = self._feedback_distance(feedback)
            distance_to_goal = self._distance_to_goal(route.goal, distance)

            if distance is not None and (
                best_distance is None
                or distance < best_distance - self._timeout_progress_m
            ):
                best_distance = distance
                last_progress_time = now

            moved_from_start = self._pose_distance(start_pose, self._latest_pose)
            if not started_moving and moved_from_start >= max(
                self._stuck_min_progress_m * 0.5, 0.03
            ):
                started_moving = True
                self._no_motion_count = 0
                self._navigator.get_logger().info(
                    f"NAV_STARTED_MOVING goal_id={route.goal_id} "
                    f"moved={moved_from_start:.3f} signature={route.signature}"
                )

            if (
                self._no_motion_after_goal_sec > 0.0
                and not started_moving
                and not no_motion_logged
                and now - start_time >= self._no_motion_after_goal_sec
                and distance_to_goal > self._near_goal_tolerance_m
            ):
                no_motion_logged = True
                self._no_motion_count += 1
                self._navigator.get_logger().warn(
                    f"NAV_ACCEPTED_BUT_NO_MOTION goal_id={route.goal_id} "
                    f"count={self._no_motion_count}/{self._no_motion_max_count} "
                    f"moved={moved_from_start:.3f} "
                    f"waited={now - start_time:.1f}s signature={route.signature}"
                )
                reason = "accepted_but_no_motion"
                if self._no_motion_count >= max(self._no_motion_max_count, 1):
                    reason = "accepted_but_no_motion_limit"
                return NavOutcome(reason, reason, self._latest_pose, moved_from_start)

            stuck = self._evaluate_patrol_stuck(route, distance_to_goal, start_time)
            if stuck is not None:
                return stuck

            if (
                self._goal_timeout_sec > 0.0
                and distance_to_goal > self._near_goal_tolerance_m
                and now - last_progress_time > self._goal_timeout_sec
            ):
                reason = "goal_progress_timeout"
                self._navigator.get_logger().warn(
                    f"STUCK_DETECTED goal_id={route.goal_id} reason={reason} "
                    f"timeout={self._goal_timeout_sec:.1f}s "
                    f"distance={distance_to_goal:.2f} signature={route.signature}"
                )
                return NavOutcome(reason, reason, self._latest_pose)

            if now - last_log_time >= 2.0:
                last_log_time = now
                moved, observed, cmd_ratio = self._window_motion(
                    self._stuck_window_sec, since_time=start_time
                )
                self._navigator.get_logger().info(
                    f"NAV_DIAG goal_id={route.goal_id} distance={distance_to_goal:.2f} "
                    f"window={observed:.1f}s moved={moved:.3f} "
                    f"cmd_ratio={cmd_ratio:.2f} signature={route.signature}"
                )

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            return NavOutcome("reached", "succeeded", self._latest_pose)

        reason = f"nav2_result_{self._describe_result(result)}"
        self._navigator.get_logger().warn(
            f"STUCK_DETECTED goal_id={route.goal_id} reason={reason} "
            f"signature={route.signature}"
        )
        return NavOutcome(reason, reason, self._latest_pose)

    def _evaluate_patrol_stuck(
        self, route: RoutePlan, distance_to_goal: float, since_time: float
    ) -> NavOutcome | None:
        """基于 odom 窗口判断是否卡住。

        `since_time` 必须是 goal 发出后的时间点，防止把机器人等待 Nav2 激活或等待上一个
        waypoint 时的静止历史算进当前 goal。
        """

        if not self._stuck_enabled:
            return None
        if distance_to_goal <= self._near_goal_tolerance_m:
            return None

        moved, observed, cmd_ratio = self._window_motion(
            self._stuck_window_sec, since_time=since_time
        )
        if observed < self._stuck_window_sec * 0.8:
            return None
        if self._stuck_cmd_required and cmd_ratio <= 0.05:
            return None
        # 留一点小容差，避免 0.079m vs 0.080m 这种毫米级边界把慢速移动误判成卡死。
        progress_epsilon_m = min(0.02, self._stuck_min_progress_m * 0.25)
        if moved + progress_epsilon_m >= self._stuck_min_progress_m:
            return None

        pose = self._latest_pose
        pose_text = "unknown"
        if pose is not None:
            pose_text = f"x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}"
        self._navigator.get_logger().warn(
            f"STUCK_DETECTED goal_id={route.goal_id} reason=no_progress "
            f"moved={moved:.3f} min={self._stuck_min_progress_m:.3f} "
            f"window={observed:.1f}s cmd_ratio={cmd_ratio:.2f} "
            f"distance={distance_to_goal:.2f} pose={pose_text} "
            f"signature={route.signature}"
        )
        return NavOutcome(
            "no_progress",
            "no_progress",
            pose,
            moved_m=moved,
            window_sec=observed,
            cmd_ratio=cmd_ratio,
        )

    def _select_route(self, waypoint: Waypoint) -> RoutePlan | None:
        """为一个巡视点选择真正要发给 Nav2 的路线。

        这里是路径黑名单生效的唯一入口：每个候选 approach goal 都必须先 getPath，
        再经过 `_route_rejection_reason()` 检查。只有加入 `viable` 的 RoutePlan 才能
        被后续 `_send_goal()` 发送给 Nav2。
        """

        now = time.monotonic()
        self._prune_failed_corridors(now)
        candidates = self._approach_goals(waypoint)
        viable: list[RoutePlan] = []

        for goal in candidates:
            # 这里是“只规划、不发车”的阶段：每个候选点先拿到 global path，
            # 让 failed corridor 和 path_signature 有机会把坏路线挡在 GOAL_SENT 前。
            route = self._plan_route(goal, waypoint)
            if route is None:
                continue

            reject_reason = self._route_rejection_reason(route)
            if reject_reason is not None:
                self._log_route_rejected(route, reject_reason)
                continue

            viable.append(route)

        if not viable:
            return None

        # 多条路线都可用时，优先选离失败路径更远、终点偏移更小、路径更短的路线。
        # 这一步只在“没有硬拒绝”的候选中排序，不会把命中黑名单的路线重新放回来。
        selected = min(
            viable,
            key=lambda r: (
                r.failed_overlap_score,
                r.endpoint_error_m,
                r.path_len_m,
            ),
        )
        self._navigator.get_logger().info(
            f"ROUTE_SELECTED goal_id={selected.goal_id} "
            f"path_len={selected.path_len_m:.2f} signature={selected.signature} "
            f"endpoint_error={selected.endpoint_error_m:.2f} "
            f"failed_overlap={selected.failed_overlap_score:.2f}"
        )
        return selected

    def _route_rejection_reason(self, route: RoutePlan) -> str | None:
        """返回候选路线被拒绝的原因；不拒绝则返回 None。

        检查顺序有意保持从便宜到昂贵：
        1. path_signature 是否和刚失败/历史失败路线一致；
        2. 路径几何是否命中 failed corridor。
        """

        if route.signature == self._last_failed_signature:
            return "same_as_failed"
        if route.signature in self._failed_path_signatures:
            return "failed_signature"
        if self._route_hits_failed_corridor(route):
            return "failed_corridor"
        return None

    def _log_route_rejected(self, route: RoutePlan, reason: str) -> None:
        """统一打印路线拒绝日志，避免多个分支里重复拼同样的字段。"""

        if reason in ("same_as_failed", "failed_signature"):
            self._navigator.get_logger().warn(
                f"ROUTE_SAME_AS_FAILED goal_id={route.goal_id} signature={route.signature}"
            )
        if reason == "failed_corridor":
            self._navigator.get_logger().warn(
                f"ROUTE_REJECTED_FAILED_CORRIDOR goal_id={route.goal_id} "
                f"signature={route.signature}"
            )
        self._navigator.get_logger().warn(
            f"ROUTE_REJECTED goal_id={route.goal_id} "
            f"signature={route.signature} reason={reason}"
        )

    def _plan_route(self, target: Waypoint, original_waypoint: Waypoint) -> RoutePlan | None:
        """调用 Nav2 planner 计算当前位姿到候选 goal 的 global path。

        这里不会发车，只把 path 转换成 RoutePlan。候选 goal 如果没有 path，就直接在
        route selection 阶段淘汰。
        """

        current_pose = self._latest_pose
        if current_pose is None:
            self._navigator.get_logger().warn(
                f"ROUTE_REJECTED goal_id={target.name} reason=no_odom_pose"
            )
            return None

        start = make_pose_stamped(
            self._navigator,
            Waypoint("route_check_start", current_pose.x, current_pose.y, current_pose.yaw),
        )
        goal = make_pose_stamped(self._navigator, target)
        try:
            try:
                path = self._navigator.getPath(start, goal, use_start=True)
            except TypeError:
                path = self._navigator.getPath(start, goal)
        except Exception as exc:
            self._navigator.get_logger().warn(
                f"ROUTE_REJECTED goal_id={target.name} reason=getPath_exception error={exc}"
            )
            return None

        poses = getattr(path, "poses", None)
        if not poses:
            self._navigator.get_logger().warn(
                f"ROUTE_CHECK goal_id={target.name} path_len=0.00 signature=none"
            )
            self._navigator.get_logger().warn(
                f"ROUTE_REJECTED goal_id={target.name} reason=no_path"
            )
            return None

        points = [
            (float(pose_stamped.pose.position.x), float(pose_stamped.pose.position.y))
            for pose_stamped in poses
        ]
        path_len = self._path_length(points)
        signature = self._path_signature(points)
        endpoint_error = math.hypot(target.x - original_waypoint.x, target.y - original_waypoint.y)
        overlap_score = self._failed_overlap_score(points)
        self._navigator.get_logger().info(
            f"ROUTE_CHECK goal_id={target.name} path_len={path_len:.2f} "
            f"signature={signature}"
        )
        return RoutePlan(
            target,
            target.name,
            points,
            path_len,
            signature,
            endpoint_error,
            overlap_score,
        )

    def _run_escape(self, stuck_pose: Pose2D | None, event_index: int) -> bool:
        """执行直接 cmd_vel 脱困动作。

        escape 不依赖 Nav2 action。每个 step 后都会检查 odom：
        - 累计位移达到 `patrol_escape_min_move_m`：立即成功并回到重新规划；
        - 单步没有平移/转角变化：打印 ESCAPE_NO_ODOM_PROGRESS，提示速度可能被拦截。
        """

        escape_start_pose = self._latest_pose
        if escape_start_pose is None:
            self._navigator.get_logger().warn("ESCAPE_FAILED reason=no_odom_before_escape")
            return False

        attempts = max(self._escape_sequence_attempts, 1)
        for attempt in range(1, attempts + 1):
            attempt_start = self._latest_pose or escape_start_pose
            for step in self._escape_steps():
                before = self._latest_pose or attempt_start
                self._navigator.get_logger().warn(
                    f"ESCAPE_START event={event_index} attempt={attempt}/{attempts} "
                    f"mode={step.mode} linear_x={step.linear_x:.2f} "
                    f"angular_z={step.angular_z:.2f} duration={step.duration_sec:.1f}"
                )
                self._publish_twist_for(step.linear_x, step.angular_z, step.duration_sec)
                self._publish_stop()
                self._spin_for(0.20)
                after = self._latest_pose or before
                step_moved = self._pose_distance(before, after)
                step_yaw = self._yaw_delta(before, after)
                attempt_moved = self._pose_distance(attempt_start, after)
                attempt_yaw = self._yaw_delta(attempt_start, after)
                total_moved = self._pose_distance(escape_start_pose, after)
                total_yaw = self._yaw_delta(escape_start_pose, after)
                self._navigator.get_logger().info(
                    f"ESCAPE_PROGRESS mode={step.mode} moved={total_moved:.3f} "
                    f"yaw_delta={total_yaw:.3f} attempt_moved={attempt_moved:.3f} "
                    f"attempt_yaw_delta={attempt_yaw:.3f} step_moved={step_moved:.3f} "
                    f"step_yaw_delta={step_yaw:.3f}"
                )
                if step_moved < 0.01 and step_yaw < 0.03:
                    self._navigator.get_logger().warn(
                        f"ESCAPE_NO_ODOM_PROGRESS mode={step.mode} "
                        f"step_moved={step_moved:.3f} step_yaw_delta={step_yaw:.3f}"
                    )
                if total_moved >= self._escape_min_move_m:
                    left_stuck_pose = self._pose_distance(stuck_pose, after)
                    self._navigator.get_logger().warn(
                        f"ESCAPE_SUCCESS event={event_index} attempt={attempt} "
                        f"moved={total_moved:.3f} yaw_delta={total_yaw:.3f} "
                        f"from_stuck={left_stuck_pose:.3f}"
                    )
                    return True

        final_pose = self._latest_pose or escape_start_pose
        moved = self._pose_distance(escape_start_pose, final_pose)
        yaw_delta = self._yaw_delta(escape_start_pose, final_pose)
        if moved < 0.01 and yaw_delta < 0.03:
            self._navigator.get_logger().warn(
                f"ESCAPE_NO_ODOM_PROGRESS moved={moved:.3f} yaw_delta={yaw_delta:.3f}"
            )
        self._navigator.get_logger().warn(
            f"ESCAPE_FAILED event={event_index} moved={moved:.3f} "
            f"yaw_delta={yaw_delta:.3f} min_move={self._escape_min_move_m:.3f}"
        )
        return False

    def _escape_steps(self) -> list[EscapeStep]:
        """生成一轮 escape 动作序列。

        顺序保守：先短距离后退，再旋转观察，再用左右弧线尝试离开局部代价地图困境。
        每一步的速度和时长都通过 launch/run_nav2.sh 参数传入。
        """

        return [
            EscapeStep(
                "backward",
                -abs(self._escape_backward_speed),
                0.0,
                self._escape_backward_duration_sec,
            ),
            EscapeStep(
                "rotate_left",
                0.0,
                abs(self._escape_turn_speed),
                self._escape_rotate_duration_sec,
            ),
            EscapeStep(
                "arc_left",
                abs(self._escape_forward_speed),
                abs(self._escape_arc_turn_speed),
                self._escape_arc_duration_sec,
            ),
            EscapeStep(
                "rotate_right",
                0.0,
                -abs(self._escape_turn_speed),
                self._escape_rotate_duration_sec,
            ),
            EscapeStep(
                "arc_right",
                abs(self._escape_forward_speed),
                -abs(self._escape_arc_turn_speed),
                self._escape_arc_duration_sec,
            ),
        ]

    def _cancel_active_goal(self, route: RoutePlan | None, reason: str) -> None:
        """取消当前 Nav2 goal，并发布零速度。

        这是进入 ESCAPING 前的安全闸门。即使 cancelTask 抛异常，也会继续 publish stop，
        避免旧速度残留。
        """

        goal_id = route.goal_id if route is not None else "unknown"
        signature = route.signature if route is not None else "unknown"
        self._navigator.get_logger().warn(
            f"NAV_CANCEL_GOAL goal_id={goal_id} signature={signature} reason={reason}"
        )
        try:
            self._navigator.cancelTask()
        except Exception as exc:
            self._navigator.get_logger().warn(f"NAV_CANCEL_FAILED error={exc}")
        self._publish_stop()
        self._spin_for(0.25)

    def _add_failed_corridor(
        self, stuck_pose: Pose2D | None, active_path: list[tuple[float, float]], reason: str
    ) -> None:
        """把当前失败路线附近的一段 global path 写入 failed corridor。

        记录内容包括：
        - stuck_pose 在 path 上最近点；
        - 最近点后方 `behind_m` 的路径；
        - 最近点前方 `ahead_m` 的路径；
        - stuck_pose 自身。

        这样后续 route check 拒绝的是“失败路径段”，而不是一个孤立圆点。
        """

        if not self._failed_corridor_enabled or stuck_pose is None or not active_path:
            return
        now = time.monotonic()
        self._prune_failed_corridors(now)
        points = self._path_segment_around_pose(
            stuck_pose,
            active_path,
            self._failed_corridor_behind_m,
            self._failed_corridor_ahead_m,
        )
        if not points:
            points = [(stuck_pose.x, stuck_pose.y)]

        signature = self._active_route.signature if self._active_route else "unknown"
        self._corridor_seq += 1
        corridor = FailedCorridor(
            self._corridor_seq,
            points,
            self._failed_corridor_radius_m,
            now,
            reason,
            signature,
        )
        self._failed_corridors.append(corridor)
        if len(self._failed_corridors) > max(self._failed_corridor_max, 1):
            self._failed_corridors = self._failed_corridors[-self._failed_corridor_max :]
        self._navigator.get_logger().warn(
            f"FAILED_CORRIDOR_ADDED id={corridor.corridor_id} "
            f"points={len(points)} radius={corridor.radius_m:.2f} "
            f"behind={self._failed_corridor_behind_m:.2f} "
            f"ahead={self._failed_corridor_ahead_m:.2f} "
            f"signature={signature} reason={reason}"
        )

    def _path_segment_around_pose(
        self,
        pose: Pose2D,
        path_points: list[tuple[float, float]],
        behind_m: float,
        ahead_m: float,
    ) -> list[tuple[float, float]]:
        """从一条 path 中截取 stuck pose 周围的 polyline 采样点。"""

        nearest_index = min(
            range(len(path_points)),
            key=lambda i: math.hypot(path_points[i][0] - pose.x, path_points[i][1] - pose.y),
        )
        selected: list[tuple[float, float]] = [path_points[nearest_index]]
        min_spacing = max(self._failed_corridor_radius_m * 0.30, 0.12)

        walked = 0.0
        previous = path_points[nearest_index]
        for point in reversed(path_points[:nearest_index]):
            walked += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
            if walked > behind_m:
                break
            if self._far_enough_from_segment_points(point, selected, min_spacing):
                selected.insert(0, point)

        walked = 0.0
        previous = path_points[nearest_index]
        for point in path_points[nearest_index + 1 :]:
            walked += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
            if walked > ahead_m:
                break
            if self._far_enough_from_segment_points(point, selected, min_spacing):
                selected.append(point)

        if self._far_enough_from_segment_points((pose.x, pose.y), selected, min_spacing):
            selected.append((pose.x, pose.y))
        return selected

    def _route_hits_failed_corridor(self, route: RoutePlan) -> bool:
        """检查候选路线是否重新穿过失败路径段。

        如果机器人当前仍在 corridor 半径内，会先允许路径“离开 corridor”。只有路径已经离开
        之后再次进入 corridor，才判定为命中。这样刚 escape 后不会因为起点仍贴着失败
        路径段，就把所有候选路线都拒绝掉。
        """

        if not self._failed_corridor_enabled or not self._failed_corridors:
            return False
        self._prune_failed_corridors(time.monotonic())
        current_pose = self._latest_pose
        left_corridor: dict[int, bool] = {}
        for corridor in self._failed_corridors:
            # 如果当前位姿已经在 corridor 外面，整条候选路径从第一个点开始就要避开它。
            # 如果当前位姿仍在 corridor 里面，允许起始一小段路径先“走出黑名单区域”。
            if current_pose is None:
                left_corridor[corridor.corridor_id] = True
                continue
            nearest_current = min(
                math.hypot(current_pose.x - cx, current_pose.y - cy)
                for cx, cy in corridor.points
            )
            left_corridor[corridor.corridor_id] = nearest_current > corridor.radius_m * 1.2

        distance_from_start = 0.0
        previous_point: tuple[float, float] | None = None
        for px, py in route.path_points:
            if previous_point is not None:
                distance_from_start += math.hypot(
                    px - previous_point[0],
                    py - previous_point[1],
                )
            previous_point = (px, py)

            for corridor in self._failed_corridors:
                # failed corridor 是一段 polyline，因此每个 path 点要和 corridor 里所有采样点
                # 取最近距离，而不是只比较 stuck_pose 一个圆。
                nearest_point = min(
                    corridor.points,
                    key=lambda point: math.hypot(px - point[0], py - point[1]),
                )
                cx, cy = nearest_point
                distance = math.hypot(px - cx, py - cy)
                if not left_corridor[corridor.corridor_id]:
                    # 起点还在 corridor 内时，不用固定距离强行截断。路径必须先穿过
                    # 当前所在的黑名单区域才能离开，所以这段连续重合不算“复用失败路”。
                    # 只有第一次离开 corridor 后，再次进入才算真正命中。
                    if distance > corridor.radius_m * 1.4:
                        left_corridor[corridor.corridor_id] = True
                    else:
                        continue
                if distance <= corridor.radius_m:
                    self._navigator.get_logger().warn(
                        f"FAILED_CORRIDOR_HIT corridor_id={corridor.corridor_id} "
                        f"goal_id={route.goal_id} signature={route.signature} "
                        f"path_point=({px:.2f},{py:.2f}) "
                        f"corridor_point=({cx:.2f},{cy:.2f}) "
                        f"distance={distance:.2f} radius={corridor.radius_m:.2f} "
                        f"path_s={distance_from_start:.2f}"
                    )
                    return True
        return False

    def _failed_overlap_score(self, points: list[tuple[float, float]]) -> float:
        """计算候选 path 与失败 corridor 的接近程度。

        这个分数只用于排序，不用于硬拒绝。硬拒绝由 `_route_hits_failed_corridor()` 完成。
        """

        if not self._failed_corridors:
            return 0.0
        score = 0.0
        for px, py in points[:: max(len(points) // 40, 1)]:
            nearest = None
            for corridor in self._failed_corridors:
                for cx, cy in corridor.points:
                    distance = math.hypot(px - cx, py - cy)
                    if nearest is None or distance < nearest:
                        nearest = distance
            if nearest is not None:
                score += 1.0 / max(nearest, 0.05)
        return score

    def _prune_failed_corridors(self, now: float) -> None:
        """清理过期 failed corridor 和对应的 path signature。"""

        if self._failed_corridor_ttl_sec <= 0.0:
            return
        self._failed_corridors = [
            corridor
            for corridor in self._failed_corridors
            if now - corridor.created_at <= self._failed_corridor_ttl_sec
        ]
        self._failed_path_signatures = {
            signature: created_at
            for signature, created_at in self._failed_path_signatures.items()
            if now - created_at <= self._failed_corridor_ttl_sec
        }

    def _mark_failed_signature(self, signature: str) -> None:
        """记录失败 path_signature，用于快速拒绝完全相同的全局路径。"""

        if self._failed_corridor_enabled and signature:
            self._failed_path_signatures[signature] = time.monotonic()

    def _approach_goals(self, waypoint: Waypoint) -> list[Waypoint]:
        """为一个巡视点生成 approach goal 列表。

        生成顺序是确定的：原始点优先，然后按左/右/前/后偏移，并按 yaw 变体展开。
        去重使用厘米级坐标和 yaw 近似，避免同一姿态重复 getPath。
        """

        base_offsets: list[tuple[str, float, float]] = [("orig", 0.0, 0.0)]
        if self._approach_offsets_enabled and self._approach_offset_m > 0.0:
            yaw = waypoint.yaw
            forward = (math.cos(yaw), math.sin(yaw))
            left = (-math.sin(yaw), math.cos(yaw))
            offset = self._approach_offset_m
            base_offsets.extend(
                [
                    ("left", left[0] * offset, left[1] * offset),
                    ("right", -left[0] * offset, -left[1] * offset),
                    ("front", forward[0] * offset, forward[1] * offset),
                    ("back", -forward[0] * offset, -forward[1] * offset),
                ]
            )

        yaw_offsets = [("yaw0", 0.0)]
        if self._approach_yaw_variants_enabled:
            yaw_offsets.extend(
                [
                    ("yaw_left", math.pi * 0.5),
                    ("yaw_right", -math.pi * 0.5),
                    ("yaw_back", math.pi),
                ]
            )

        goals: list[Waypoint] = []
        seen: set[tuple[int, int, int]] = set()
        for offset_name, dx, dy in base_offsets:
            for yaw_name, yaw_delta in yaw_offsets:
                x = waypoint.x + dx
                y = waypoint.y + dy
                yaw = normalize_angle(waypoint.yaw + yaw_delta)
                key = (round(x * 100), round(y * 100), round(yaw * 100))
                if key in seen:
                    continue
                seen.add(key)
                suffix = offset_name if yaw_name == "yaw0" else f"{offset_name}_{yaw_name}"
                goals.append(Waypoint(f"{waypoint.name}_{suffix}", x, y, yaw))
        return goals

    def _build_waypoints(self) -> list[Waypoint]:
        """生成巡航 waypoint。

        优先从静态地图采样覆盖点；地图不可读或关闭 coverage planner 时使用内置 fallback。
        """

        config = CoveragePlannerConfig(
            enabled=self._bool_param("coverage_planner_enabled", True),
            sample_spacing_m=self._float_param("coverage_sample_spacing_m", 1.10),
            min_obstacle_distance_m=self._float_param("waypoint_min_obstacle_distance_m", 0.35),
            max_waypoints=self._int_param("coverage_max_waypoints", 40),
            start_x=0.0,
            start_y=0.0,
        )
        map_yaml = self._str_param("map_yaml", "")
        planner = CoveragePlanner(config)
        waypoints = planner.plan(map_yaml, FALLBACK_WAYPOINTS)
        if waypoints == FALLBACK_WAYPOINTS:
            self._navigator.get_logger().warn(
                "Using fallback waypoints; check map_yaml and map readability"
            )
        else:
            self._navigator.get_logger().info("Generated coverage waypoints from static map")
            self._log_waypoint_bounds(waypoints)
        return waypoints

    def _log_waypoint_bounds(self, waypoints: list[Waypoint]) -> None:
        """打印生成 waypoint 的范围和顺序，便于确认覆盖点没有集中在地图一角。"""

        min_x = min(point.x for point in waypoints)
        max_x = max(point.x for point in waypoints)
        min_y = min(point.y for point in waypoints)
        max_y = max(point.y for point in waypoints)
        preview = ", ".join(f"{point.name}=({point.x:.1f},{point.y:.1f})" for point in waypoints)
        self._navigator.get_logger().info(
            f"COVERAGE_WAYPOINT_BOUNDS count={len(waypoints)} "
            f"x=[{min_x:.2f},{max_x:.2f}] y=[{min_y:.2f},{max_y:.2f}]"
        )
        self._navigator.get_logger().info(f"COVERAGE_WAYPOINT_ORDER {preview}")

    def _on_odom(self, msg: Odometry) -> None:
        """记录最新 odom，并维护有限长度历史窗口。"""

        pose = msg.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation.z, pose.orientation.w)
        current = Pose2D(float(pose.position.x), float(pose.position.y), yaw)
        self._latest_pose = current
        now = time.monotonic()
        self._odom_history.append(OdomSample(now, current))
        max_window = self._history_max_window_sec()
        while self._odom_history and now - self._odom_history[0].stamp > max_window:
            self._odom_history.popleft()

    def _on_cmd_vel(self, msg: Twist) -> None:
        """记录 cmd_vel 是否活跃。

        这里订阅的是监控 topic；escape 发布 topic 可以相同，也可以由 mux/remap 区分。
        """

        active = abs(float(msg.linear.x)) >= 0.01 or abs(float(msg.angular.z)) >= 0.03
        now = time.monotonic()
        self._cmd_history.append(CmdSample(now, active))
        max_window = self._history_max_window_sec()
        while self._cmd_history and now - self._cmd_history[0].stamp > max_window:
            self._cmd_history.popleft()

    def _history_max_window_sec(self) -> float:
        """odom/cmd_vel 历史缓存长度。

        缓存必须覆盖 stuck 窗口、accepted-but-no-motion 窗口，并额外留一点余量，避免回调
        频率抖动时刚好把边界样本删掉。
        """

        return max(self._stuck_window_sec, self._no_motion_after_goal_sec, 10.0) + 5.0

    def _wait_for_initial_odom(self, timeout_sec: float) -> None:
        """等待第一条 odom。

        没有 odom 时仍允许程序继续，但 route check 会因为没有当前位置而拒绝候选路线。
        """

        end = time.monotonic() + timeout_sec
        while rclpy.ok() and self._latest_pose is None and time.monotonic() < end:
            rclpy.spin_once(self._navigator, timeout_sec=0.10)
        if self._latest_pose is None:
            self._navigator.get_logger().warn(
                f"ODOM_WAIT_TIMEOUT timeout={timeout_sec:.1f}s topic={self._odom_topic}"
            )
        else:
            self._navigator.get_logger().info(
                f"ODOM_READY x={self._latest_pose.x:.2f} "
                f"y={self._latest_pose.y:.2f} yaw={self._latest_pose.yaw:.2f}"
            )

    def _wait_for_nav2_ready(self, timeout_sec: float) -> bool:
        """等待 Nav2 lifecycle 节点进入 active。

        这里不用 BasicNavigator.waitUntilNav2Active()，因为当前 launch 中巡航节点可能早于
        navigation lifecycle 完全 active 启动；旧等待没有清晰超时日志，实际会表现成
        “启动后机器人不动”。本函数只轮询必要节点，并且每次状态变化都打印出来。
        """

        required_nodes = ("amcl", "bt_navigator", "planner_server", "controller_server")
        clients = {
            node_name: self._navigator.create_client(GetState, f"/{node_name}/get_state")
            for node_name in required_nodes
        }
        active_nodes: set[str] = set()
        last_labels: dict[str, str] = {}
        deadline = time.monotonic() + max(timeout_sec, 1.0)
        self._navigator.get_logger().info(
            f"NAV2_WAIT_START timeout={timeout_sec:.1f}s nodes={','.join(required_nodes)}"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            for node_name, client in clients.items():
                if node_name in active_nodes:
                    continue
                if not client.wait_for_service(timeout_sec=0.20):
                    label = "service_unavailable"
                else:
                    label = self._query_lifecycle_state(node_name, client)

                if last_labels.get(node_name) != label:
                    last_labels[node_name] = label
                    self._navigator.get_logger().info(
                        f"NAV2_WAIT_NODE node={node_name} state={label}"
                    )
                if label == "active":
                    active_nodes.add(node_name)
                    self._navigator.get_logger().info(f"NAV2_NODE_ACTIVE node={node_name}")

            if len(active_nodes) == len(required_nodes):
                self._navigator.get_logger().info("NAV2_WAIT_READY")
                return True
            rclpy.spin_once(self._navigator, timeout_sec=0.10)

        missing = ",".join(node for node in required_nodes if node not in active_nodes)
        self._navigator.get_logger().error(
            f"NAV2_WAIT_TIMEOUT timeout={timeout_sec:.1f}s missing={missing}"
        )
        return False

    def _query_lifecycle_state(self, node_name: str, client) -> str:
        """查询单个 lifecycle 节点状态；失败时返回可写入日志的原因字符串。"""

        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self._navigator, future, timeout_sec=0.80)
        if not future.done():
            return "state_timeout"
        try:
            state = future.result().current_state
        except Exception as exc:
            self._navigator.get_logger().warn(
                f"NAV2_WAIT_QUERY_FAILED node={node_name} error={exc}"
            )
            return "state_error"
        if state.id == State.PRIMARY_STATE_ACTIVE:
            return "active"
        return state.label or f"id_{state.id}"

    def _window_motion(
        self, window_sec: float, since_time: float | None = None
    ) -> tuple[float, float, float]:
        """统计一个时间窗口内的平移位移、实际观测时长和 cmd_vel 活跃比例。"""

        now = time.monotonic()
        start_time = now - window_sec
        if since_time is not None:
            start_time = max(start_time, since_time)
        samples = [sample for sample in self._odom_history if sample.stamp >= start_time]
        if len(samples) < 2:
            return 0.0, 0.0, 0.0
        first = samples[0]
        last = samples[-1]
        moved = math.hypot(last.pose.x - first.pose.x, last.pose.y - first.pose.y)
        observed = last.stamp - first.stamp
        cmd_samples = [sample for sample in self._cmd_history if sample.stamp >= start_time]
        cmd_ratio = 0.0
        if cmd_samples:
            cmd_ratio = sum(1 for sample in cmd_samples if sample.active) / float(len(cmd_samples))
        return moved, observed, cmd_ratio

    def _publish_twist_for(self, linear_x: float, angular_z: float, duration_sec: float) -> None:
        """按 20Hz 左右发布一段 Twist，并在发布期间 spin 一下以接收 odom。"""

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        end = time.monotonic() + max(duration_sec, 0.0)
        while rclpy.ok() and time.monotonic() < end:
            self._cmd_pub.publish(twist)
            rclpy.spin_once(self._navigator, timeout_sec=0.0)
            time.sleep(0.05)

    def _publish_stop(self) -> None:
        """发布零速度。"""

        self._cmd_pub.publish(Twist())

    def _spin_for(self, duration_sec: float) -> None:
        """短暂 spin，用于等待 cancel/odom/cmd_vel 回调落地。"""

        end = time.monotonic() + max(duration_sec, 0.0)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self._navigator, timeout_sec=0.02)

    def _transition(self, old: NavState, new: NavState, reason: str) -> NavState:
        """状态机统一流转日志。"""

        self._navigator.get_logger().info(
            f"NAV_STATE_CHANGE from={old.value} to={new.value} reason={reason}"
        )
        return new

    def _distance_to_goal(self, goal: Waypoint, feedback_distance: float | None) -> float:
        """优先使用 Nav2 feedback 距离；没有 feedback 时退回直线距离。"""

        if feedback_distance is not None:
            return feedback_distance
        if self._latest_pose is None:
            return float("inf")
        return math.hypot(goal.x - self._latest_pose.x, goal.y - self._latest_pose.y)

    @staticmethod
    def _feedback_distance(feedback: Any) -> float | None:
        """从 BasicNavigator feedback 中安全读取剩余距离。"""

        if feedback is None:
            return None
        try:
            value = float(feedback.distance_remaining)
        except (AttributeError, TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        return value

    @staticmethod
    def _pose_distance(a: Pose2D | None, b: Pose2D | None) -> float:
        """计算两个二维位姿的平面距离；任一为空时返回 0。"""

        if a is None or b is None:
            return 0.0
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def _yaw_delta(a: Pose2D | None, b: Pose2D | None) -> float:
        """计算两个 yaw 的最小角度差绝对值。"""

        if a is None or b is None:
            return 0.0
        return abs(normalize_angle(b.yaw - a.yaw))

    @staticmethod
    def _path_length(points: list[tuple[float, float]]) -> float:
        """计算 polyline 总长度。"""

        if len(points) < 2:
            return 0.0
        return sum(
            math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
            for i in range(1, len(points))
        )

    @staticmethod
    def _path_signature(points: list[tuple[float, float]]) -> str:
        """为 path 生成短签名。

        签名只用于运行时诊断和“明显相同路线”的快速拒绝，不作为持久化数据。
        """

        if not points:
            return "empty"
        sample_count = min(12, len(points))
        if sample_count == 1:
            sample_indices = [0]
        else:
            sample_indices = [
                round(i * (len(points) - 1) / float(sample_count - 1))
                for i in range(sample_count)
            ]
        payload = ";".join(
            f"{points[index][0]:.1f},{points[index][1]:.1f}" for index in sample_indices
        )
        return hashlib.sha1(payload.encode("utf-8")).hexdigest()[:10]

    @staticmethod
    def _far_enough_from_segment_points(
        point: tuple[float, float], points: list[tuple[float, float]], min_spacing: float
    ) -> bool:
        """判断 point 是否和已有采样点保持最小间距。"""

        return all(math.hypot(point[0] - x, point[1] - y) >= min_spacing for x, y in points)

    @staticmethod
    def _yaw_from_quaternion(z: float, w: float) -> float:
        """从只含 z/w 的 2D quaternion 中恢复 yaw。"""

        return normalize_angle(2.0 * math.atan2(z, w))

    @staticmethod
    def _describe_result(result: TaskResult) -> str:
        """把 Nav2 TaskResult 转成人类可读字符串。"""

        if result == TaskResult.SUCCEEDED:
            return "SUCCEEDED"
        if result == TaskResult.CANCELED:
            return "CANCELED"
        if result == TaskResult.FAILED:
            return "FAILED"
        return f"UNKNOWN({result})"

    def _declare_parameters(self) -> None:
        """声明本节点使用的 ROS 参数。

        run_nav2.sh -> nav2_patrol.launch.py -> patrol_waypoints.py 的参数链路最终都会落到这里。
        """

        parameter_groups = [
            (
                "地图与 waypoint 生成",
                {
                    "map_yaml": "",
                    "coverage_planner_enabled": True,
                    "coverage_sample_spacing_m": 1.10,
                    "coverage_max_waypoints": 40,
                    "waypoint_min_obstacle_distance_m": 0.35,
                },
            ),
            (
                "导航监控与卡住判断",
                {
                    "odom_topic": "/odom",
                    "cmd_vel_topic": "/cmd_vel",
                    "waypoint_timeout_sec": 6.0,
                    "waypoint_timeout_progress_m": 0.08,
                    "patrol_stuck_enabled": True,
                    "patrol_stuck_min_progress_m": 0.08,
                    "patrol_stuck_window_sec": 4.0,
                    "patrol_stuck_cmd_required": True,
                    "patrol_stuck_near_goal_tolerance_m": 0.35,
                    "patrol_no_motion_after_goal_sec": 3.0,
                    "patrol_no_motion_max_count": 2,
                },
            ),
            (
                "失败路径段黑名单",
                {
                    "patrol_failed_corridor_enabled": True,
                    "patrol_failed_corridor_radius_m": 0.60,
                    "patrol_failed_corridor_behind_m": 0.80,
                    "patrol_failed_corridor_ahead_m": 2.00,
                    "patrol_failed_corridor_ttl_sec": 240.0,
                    "patrol_failed_corridor_max": 12,
                },
            ),
            (
                "候选 approach goal",
                {
                    "patrol_approach_offsets_enabled": True,
                    "patrol_approach_offset_m": 0.60,
                    "patrol_approach_yaw_variants_enabled": True,
                },
            ),
            (
                "主动 cmd_vel 脱困",
                {
                    "patrol_escape_enabled": True,
                    "patrol_escape_cmd_topic": "/cmd_vel",
                    "patrol_escape_min_move_m": 0.10,
                    "patrol_escape_max_attempts": 2,
                    "patrol_escape_sequence_attempts": 1,
                    "patrol_escape_backward_speed": 0.12,
                    "patrol_escape_forward_speed": 0.08,
                    "patrol_escape_turn_speed": 0.50,
                    "patrol_escape_backward_duration_sec": 1.0,
                    "patrol_escape_rotate_duration_sec": 0.7,
                    "patrol_escape_arc_duration_sec": 1.0,
                },
            ),
            (
                "任务事件触发（run_full_system 语音巡检）",
                {
                    "patrol_trigger_mode": "auto",
                    "task_events_topic": "/task/events",
                    "nav2_ready_timeout_sec": 45.0,
                    "patrol_publish_initial_pose_on_start": False,
                    "patrol_seed_initial_pose_from_tf": True,
                    "map_frame": "map",
                    "robot_base_frame": "base_footprint",
                },
            ),
        ]
        for _, defaults in parameter_groups:
            for name, value in defaults.items():
                self._navigator.declare_parameter(name, value)

    def _str_param(self, name: str, default: str) -> str:
        """读取字符串参数；None 时使用默认值。"""

        value = self._navigator.get_parameter(name).value
        return str(value if value is not None else default)

    def _float_param(self, name: str, default: float) -> float:
        """读取 float 参数；非法值回退默认值。"""

        try:
            return float(self._navigator.get_parameter(name).value)
        except (TypeError, ValueError):
            return default

    def _int_param(self, name: str, default: int) -> int:
        """读取 int 参数；非法值回退默认值。"""

        try:
            return int(self._navigator.get_parameter(name).value)
        except (TypeError, ValueError):
            return default

    def _bool_param(self, name: str, default: bool) -> bool:
        """读取 bool 参数。

        launch 里可能传 true/false，shell 里常传 1/0，所以这里同时兼容字符串和数字。
        """

        value = self._navigator.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        if isinstance(value, (int, float)):
            return bool(value)
        return default
