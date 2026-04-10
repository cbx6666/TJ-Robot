"""Coverage patrol：外圈激光沿墙并记录轨迹 → 大转角处向质心方向按激光量程多层偏移生成航点 → 跟点。

遇障：路径模式下慢速前进并沿「内侧」偏转；距航点小于一个激光量程时可跳过。
可选在任务完成后调用 nav2 `map_saver_cli` 保存 `/map`（不订阅 /map，仅子进程存图）。"""

from __future__ import annotations

import math
import os
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any, List, Optional, Tuple

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from robot_navigation.nav_math import Pose2D, clamp, normalize_angle, yaw_from_quaternion


Waypoint = Tuple[float, float, str]


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class CoveragePatrol(Node):
    def __init__(self) -> None:
        super().__init__('coverage_patrol')
        # 存图延迟用墙上时钟：use_sim_time 下默认 ROS 定时器跟 /clock，仿真暂停时永不触发
        # Humble rclpy 为 STEADY_TIME（无 STEADY）；用单调时钟避免 use_sim_time 下跟 /clock 卡住定时器
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)

        # launch 的 params-file 可能已注入 use_sim_time，重复 declare 会报错
        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('room_min_x', -7.0)
        self.declare_parameter('room_max_x', 7.0)
        self.declare_parameter('room_min_y', -7.0)
        self.declare_parameter('room_max_y', 7.0)
        self.declare_parameter('robot_radius_m', 0.18)
        self.declare_parameter('wall_clearance_robot_sizes', 4.0)
        self.declare_parameter('override_margins_with_size_rules', True)
        self.declare_parameter('route_margin', 0.7)
        self.declare_parameter('progress_epsilon', 0.12)
        self.declare_parameter('goal_stuck_sec', 4.0)
        self.declare_parameter('position_tolerance', 0.22)
        self.declare_parameter('heading_tolerance', 0.20)
        self.declare_parameter('linear_speed_limit', 0.60)
        self.declare_parameter('angular_speed_limit', 2.8)
        self.declare_parameter('linear_kp', 1.2)
        self.declare_parameter('angular_kp', 3.0)
        self.declare_parameter('obstacle_distance', 0.50)
        self.declare_parameter('avoid_turn_speed', 1.8)
        self.declare_parameter('slowdown_distance', 2.20)
        self.declare_parameter('min_linear_speed', 0.12)
        self.declare_parameter('wall_follow_side', 'right')
        self.declare_parameter('perimeter_seek_wall_distance', 0.90)
        self.declare_parameter('perimeter_seek_linear_speed', 0.48)
        self.declare_parameter('perimeter_front_block_distance', 0.55)
        self.declare_parameter('perimeter_wall_lost_range_m', 0.0)
        self.declare_parameter('perimeter_corner_extra_turn_deg', 8.0)
        self.declare_parameter('perimeter_lap_radius_m', 1.05)
        self.declare_parameter('perimeter_min_traveled_m', 22.0)
        self.declare_parameter('perimeter_lap_min_separation_m', 2.8)
        self.declare_parameter('perimeter_force_finish_traveled_m', 0.0)
        self.declare_parameter('wall_dist_near_robot_sizes', 2.0)
        self.declare_parameter('wall_dist_far_robot_sizes', 4.0)
        self.declare_parameter('wall_band_hysteresis_m', 0.12)
        self.declare_parameter('wall_bias_away_deg', 7.0)
        self.declare_parameter('wall_bias_toward_deg', 5.5)
        self.declare_parameter('wall_parallel_kp', 1.35)
        self.declare_parameter('wall_follow_linear_speed', 0.40)
        self.declare_parameter('wall_follow_angular_limit', 2.7)
        self.declare_parameter('wall_follow_side_ema_alpha', 0.22)
        self.declare_parameter('wall_follow_tangent_ema_alpha', 0.18)
        self.declare_parameter('wall_follow_heading_deadband_deg', 2.2)
        self.declare_parameter('wall_follow_angular_slew_rad_per_sec', 4.8)
        self.declare_parameter('wall_follow_linear_ema_alpha', 0.35)
        # 外圈沿墙：为减轻 v+ω 同时积分带来的建图/轨迹错位，可强制先对准再直行（与第二阶段思路一致）
        self.declare_parameter('perimeter_pure_turn_drive', True)
        self.declare_parameter('perimeter_align_heading_deg', 6.0)
        self.declare_parameter('laser_range_from_scan', True)
        self.declare_parameter('laser_range_fallback_m', 8.0)
        self.declare_parameter('laser_range_cap_m', 25.0)
        self.declare_parameter('route_obstacle_giveup_sec', 4.5)
        # 必须 > heading_tolerance，否则会在两阈值之间只发零速、车停住直到 stuck 超时
        self.declare_parameter('route_drift_realign_rad', 0.28)
        self.declare_parameter('route_goal_stuck_sec', 7.5)
        self.declare_parameter('p1_path_sample_min_step_m', 0.12)
        # 沿轨迹累计弧长后再算夹角；密采样+平滑沿墙时相邻边夹角很小，必须用弦长尺度
        self.declare_parameter('p1_corner_arc_chord_m', 0.30)
        self.declare_parameter('p1_corner_turn_threshold_deg', 70.0)
        self.declare_parameter('route_corner_merge_m', 0.28)
        self.declare_parameter('route_contour_body_widths', 3.0)
        self.declare_parameter('route_skip_goal_if_within_laser_range', True)
        # 用字符串默认，便于 launch 里只传 LaunchConfiguration（避免部分环境下 bool 覆盖失效）
        self.declare_parameter('save_map_on_complete', 'false')
        self.declare_parameter('save_map_directory', '')
        self.declare_parameter('save_map_basename', '')
        self.declare_parameter('save_map_delay_sec', 2.0)
        self.declare_parameter('exit_after_map_save', 'true')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('post_save_person_overlay', 'false')
        self.declare_parameter(
            'person_regions_yaml',
            str(Path.home() / '.ros' / 'tj_person_strip_regions.yaml'),
        )
        self.declare_parameter('snapshot_overlay_from_cloud_if_no_regions', 'true')
        self.declare_parameter('person_laser_map_cloud_topic', '/human_yolo/person_laser_map_cloud')

        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._scan_topic = str(self.get_parameter('scan_topic').value)
        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._room_min_x = float(self.get_parameter('room_min_x').value)
        self._room_max_x = float(self.get_parameter('room_max_x').value)
        self._room_min_y = float(self.get_parameter('room_min_y').value)
        self._room_max_y = float(self.get_parameter('room_max_y').value)
        self._robot_radius_m = float(self.get_parameter('robot_radius_m').value)
        self._override_margins = _as_bool(self.get_parameter('override_margins_with_size_rules').value)
        self._route_margin = float(self.get_parameter('route_margin').value)
        self._progress_epsilon = float(self.get_parameter('progress_epsilon').value)
        self._goal_stuck_sec = float(self.get_parameter('goal_stuck_sec').value)
        self._position_tolerance = float(self.get_parameter('position_tolerance').value)
        self._heading_tolerance = float(self.get_parameter('heading_tolerance').value)
        self._linear_speed_limit = float(self.get_parameter('linear_speed_limit').value)
        self._angular_speed_limit = float(self.get_parameter('angular_speed_limit').value)
        self._linear_kp = float(self.get_parameter('linear_kp').value)
        self._angular_kp = float(self.get_parameter('angular_kp').value)
        self._obstacle_distance = float(self.get_parameter('obstacle_distance').value)
        self._avoid_turn_speed = float(self.get_parameter('avoid_turn_speed').value)
        self._slowdown_distance = float(self.get_parameter('slowdown_distance').value)
        self._min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        side = str(self.get_parameter('wall_follow_side').value).strip().lower()
        self._wall_follow_right = side != 'left'
        self._perimeter_seek_wall_d = float(self.get_parameter('perimeter_seek_wall_distance').value)
        self._perimeter_seek_v = float(self.get_parameter('perimeter_seek_linear_speed').value)
        self._perimeter_front_block = float(self.get_parameter('perimeter_front_block_distance').value)
        self._perimeter_wall_lost_m = float(self.get_parameter('perimeter_wall_lost_range_m').value)
        self._perimeter_corner_extra = math.radians(
            float(self.get_parameter('perimeter_corner_extra_turn_deg').value)
        )
        self._perimeter_lap_r = float(self.get_parameter('perimeter_lap_radius_m').value)
        self._perimeter_min_travel = float(self.get_parameter('perimeter_min_traveled_m').value)
        self._perimeter_lap_min_sep = float(self.get_parameter('perimeter_lap_min_separation_m').value)
        self._perimeter_force_finish_m = float(self.get_parameter('perimeter_force_finish_traveled_m').value)
        self._wall_dist_near_mult = float(self.get_parameter('wall_dist_near_robot_sizes').value)
        self._wall_dist_far_mult = float(self.get_parameter('wall_dist_far_robot_sizes').value)
        self._wall_band_hyst_m = float(self.get_parameter('wall_band_hysteresis_m').value)
        self._wall_bias_away_rad = math.radians(float(self.get_parameter('wall_bias_away_deg').value))
        self._wall_bias_toward_rad = math.radians(float(self.get_parameter('wall_bias_toward_deg').value))
        self._wall_parallel_kp = float(self.get_parameter('wall_parallel_kp').value)
        self._wall_follow_v = float(self.get_parameter('wall_follow_linear_speed').value)
        self._wall_follow_w_lim = float(self.get_parameter('wall_follow_angular_limit').value)
        self._wf_side_ema = clamp(float(self.get_parameter('wall_follow_side_ema_alpha').value), 0.05, 1.0)
        self._wf_tan_ema = clamp(float(self.get_parameter('wall_follow_tangent_ema_alpha').value), 0.05, 1.0)
        self._wf_heading_deadband = math.radians(
            float(self.get_parameter('wall_follow_heading_deadband_deg').value)
        )
        self._wf_w_slew = max(float(self.get_parameter('wall_follow_angular_slew_rad_per_sec').value), 0.1)
        self._wf_v_ema = clamp(float(self.get_parameter('wall_follow_linear_ema_alpha').value), 0.05, 1.0)
        self._perimeter_pure_turn_drive = _as_bool(self.get_parameter('perimeter_pure_turn_drive').value)
        self._perimeter_align_heading_rad = math.radians(
            max(float(self.get_parameter('perimeter_align_heading_deg').value), 0.5)
        )
        self._laser_range_from_scan = _as_bool(self.get_parameter('laser_range_from_scan').value)
        self._laser_range_fallback_m = float(self.get_parameter('laser_range_fallback_m').value)
        self._laser_range_cap_m = max(float(self.get_parameter('laser_range_cap_m').value), 0.5)
        self._route_obstacle_giveup_sec = max(float(self.get_parameter('route_obstacle_giveup_sec').value), 0.5)
        self._route_drift_realign_rad = max(float(self.get_parameter('route_drift_realign_rad').value), 0.05)
        self._route_goal_stuck_sec = float(self.get_parameter('route_goal_stuck_sec').value)
        self._route_drift_realign_rad = max(
            self._route_drift_realign_rad,
            self._heading_tolerance + 0.05,
        )
        self._p1_sample_min_step_m = max(float(self.get_parameter('p1_path_sample_min_step_m').value), 0.05)
        self._p1_corner_arc_m = max(float(self.get_parameter('p1_corner_arc_chord_m').value), 0.08)
        self._p1_corner_deg = max(float(self.get_parameter('p1_corner_turn_threshold_deg').value), 15.0)
        self._route_corner_merge_m = max(float(self.get_parameter('route_corner_merge_m').value), 0.08)
        _rcbw = float(self.get_parameter('route_contour_body_widths').value)
        self._route_contour_body_widths = clamp(_rcbw, 2.0, 4.0)
        self._route_skip_within_laser_r = _as_bool(self.get_parameter('route_skip_goal_if_within_laser_range').value)
        self._save_map_on_complete = _as_bool(self.get_parameter('save_map_on_complete').value)
        self._save_map_directory_raw = str(self.get_parameter('save_map_directory').value or '').strip()
        self._save_map_basename_raw = str(self.get_parameter('save_map_basename').value or '').strip()
        self._save_map_delay_sec = max(float(self.get_parameter('save_map_delay_sec').value), 0.0)
        self._exit_after_map_save = _as_bool(self.get_parameter('exit_after_map_save').value)
        self._map_topic = str(self.get_parameter('map_topic').value or '/map').strip() or '/map'
        self._post_save_person_overlay = _as_bool(self.get_parameter('post_save_person_overlay').value)
        self._person_regions_yaml = os.path.expanduser(
            str(self.get_parameter('person_regions_yaml').value or '').strip()
            or str(Path.home() / '.ros' / 'tj_person_strip_regions.yaml')
        )
        self._snapshot_overlay_from_cloud = _as_bool(
            self.get_parameter('snapshot_overlay_from_cloud_if_no_regions').value
        )
        self._person_laser_cloud_topic = (
            str(self.get_parameter('person_laser_map_cloud_topic').value or '').strip()
            or '/human_yolo/person_laser_map_cloud'
        )

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self._ctrl_period_s = 1.0 / max(control_rate_hz, 1.0)

        robot_size = max(self._robot_radius_m * 2.0, 0.05)
        self._robot_size_m = robot_size
        wall_clearance = max(robot_size * max(float(self.get_parameter('wall_clearance_robot_sizes').value), 0.0), robot_size)
        if self._override_margins:
            self._route_margin = wall_clearance
        self._obstacle_distance = max(self._obstacle_distance, wall_clearance * 0.6)
        d_far_nom = robot_size * max(self._wall_dist_far_mult, 1.0)
        if self._perimeter_wall_lost_m <= 0.0:
            self._perimeter_wall_lost_m = max(d_far_nom * 1.35, 1.2)

        self._current_pose: Optional[Pose2D] = None
        self._last_scan: Optional[LaserScan] = None
        self._perimeter_done = False
        self._waypoints: List[Waypoint] = []
        self._current_index = 0
        self._route_ready = False
        self._perimeter_subphase_seek = True
        self._lap_xy: Optional[Tuple[float, float]] = None
        self._lap_max_separation_m = 0.0
        self._last_odom_xy: Optional[Tuple[float, float]] = None
        self._perimeter_traveled_m = 0.0
        self._wf_smooth_side: Optional[float] = None
        self._wf_smooth_tangent: Optional[float] = None
        self._wf_last_w_cmd = 0.0
        self._wf_last_v_cmd = 0.0
        self._completed = False
        self._last_status = ''
        self._progress_goal_index = -1
        self._best_goal_distance = float('inf')
        self._last_progress_ns: Optional[int] = None
        self._scan_range_max_m = 0.0
        self._p2_exec = 'rotate'
        self._p2_exec_for_idx = -1
        self._p2_blocked_since_ns: Optional[int] = None
        self._p1_samples: List[Tuple[float, float]] = []
        self._p1_ccw = True
        self._map_save_scheduled = False
        self._map_save_timer: Any = None

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self._timer = self.create_timer(1.0 / control_rate_hz, self._on_timer)

        self.get_logger().info(
            f'coverage_patrol: odom={self._odom_topic} scan={self._scan_topic} cmd_vel={self._cmd_vel_topic}'
        )
        self.get_logger().info(
            f'coverage_patrol: map_save={int(self._save_map_on_complete)} delay_s={self._save_map_delay_sec} '
            f'exit_after_save={int(self._exit_after_map_save)} map_topic={self._map_topic} '
            f'post_person_overlay={int(self._post_save_person_overlay)}'
        )
        if self._save_map_on_complete:
            try:
                sd = self._resolve_map_save_directory()
                self.get_logger().info(
                    'coverage_patrol: 存图绝对路径（未设 save_map_directory / '
                    'TJ_ROBOT_SAVED_MAPS_DIR 时 = 当前 shell 的 cwd + saved_maps/；'
                    f'从 ros_ws 启动常见为 …/ros_ws/saved_maps/）: {sd}'
                )
            except OSError as e:
                self.get_logger().warning(f'coverage_patrol: 无法创建存图目录: {e}')
        if self._post_save_person_overlay:
            self.get_logger().info(
                f'coverage_patrol: person_regions_yaml={self._person_regions_yaml} '
                f'snapshot_from_cloud_if_missing={int(self._snapshot_overlay_from_cloud)} '
                f'cloud_topic={self._person_laser_cloud_topic}'
            )

    def _resolve_map_save_directory(self) -> Path:
        raw = self._save_map_directory_raw
        if raw:
            p = Path(raw).expanduser()
        else:
            env = (os.environ.get('TJ_ROBOT_SAVED_MAPS_DIR') or '').strip()
            if env:
                p = Path(env).expanduser()
            else:
                p = Path.cwd() / 'saved_maps'
        p = p.resolve()
        p.mkdir(parents=True, exist_ok=True)
        return p

    def _ros_use_sim_time(self) -> bool:
        v = self.get_parameter('use_sim_time').value
        if isinstance(v, bool):
            return v
        return str(v).lower() in ('1', 'true', 'yes', 'on')

    def _schedule_map_save_if_needed(self) -> None:
        if self._map_save_scheduled:
            return
        if not self._save_map_on_complete:
            self.get_logger().info(
                'coverage_patrol: save_map_on_complete=false → 不自动存图、任务结束后节点继续运行 '
                '(自动存图并退出请加 -p save_map_on_complete:=true；手动存图见 README 或 map_saver_cli -h)'
            )
            return
        self._map_save_scheduled = True
        d = self._save_map_delay_sec
        if d <= 0.0:
            self._run_map_saver()
            return
        self.get_logger().info(
            f'coverage_patrol: scheduling map save in {d}s (wall clock; avoids sim-time stall)'
        )
        self._map_save_timer = self.create_timer(
            d, self._map_save_timer_cb, clock=self._wall_clock
        )

    def _map_save_timer_cb(self) -> None:
        if self._map_save_timer is not None:
            self._map_save_timer.cancel()
            self._map_save_timer = None
        self._run_map_saver()

    def _shutdown_if_exit_after_save(self) -> None:
        if self._exit_after_map_save:
            rclpy.shutdown()

    def _run_cloud_snapshot_for_overlay(self, output_yaml: Path) -> None:
        cmd = [
            'ros2',
            'run',
            'human_yolo_seg',
            'snapshot_person_regions_from_cloud',
            '--ros-args',
        ]
        if self._ros_use_sim_time():
            cmd += ['-p', 'use_sim_time:=true']
        cmd += [
            '-p',
            f'output_path:={output_yaml}',
            '-p',
            f'cloud_topic:={self._person_laser_cloud_topic}',
            '-p',
            'timeout_sec:=12.0',
        ]
        self.get_logger().info(f'snapshot_person_regions_from_cloud: {" ".join(cmd)}')
        try:
            r = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=45.0,
                env=os.environ.copy(),
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            self.get_logger().warning(f'snapshot_person_regions_from_cloud failed: {e}')
            return
        if r.returncode != 0:
            err = (r.stderr or r.stdout or '').strip()
            self.get_logger().warning(f'snapshot_person_regions_from_cloud exit {r.returncode}: {err}')
            return
        out = (r.stdout or '').strip()
        if out:
            self.get_logger().info(out)

    def _maybe_post_save_person_overlay(self, map_yaml: Path) -> None:
        if not self._post_save_person_overlay:
            return
        regions = Path(self._person_regions_yaml)
        snap_yaml = map_yaml.parent / f'{map_yaml.stem}_person_regions_snapshot.yaml'
        if not regions.is_file():
            if self._snapshot_overlay_from_cloud:
                self.get_logger().info(
                    f'post_save_person_overlay: 无 recorder 文件 {regions}，'
                    f'从 {self._person_laser_cloud_topic} 快照 -> {snap_yaml}'
                )
                self._run_cloud_snapshot_for_overlay(snap_yaml)
            if snap_yaml.is_file():
                regions = snap_yaml
            else:
                self.get_logger().info(
                    'post_save_person_overlay: skip（无 tj_person_strip_regions.yaml，'
                    '且云快照未生成；请起 person_strip_recorder，或确保仿真中曾发布 person_laser_map_cloud）'
                )
                return
        out_png = map_yaml.parent / f'{map_yaml.stem}_person_overlay.png'
        cmd = [
            'ros2',
            'run',
            'human_yolo_seg',
            'annotate_saved_map_person_overlay',
            str(map_yaml.resolve()),
            str(regions.resolve()),
            '-o',
            str(out_png.resolve()),
            '--also-composite',
        ]
        self.get_logger().info(f'person overlay: {" ".join(cmd)}')
        try:
            r = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=60.0,
                env=os.environ.copy(),
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            self.get_logger().warning(f'person overlay failed to run: {e}')
            return
        if r.returncode != 0:
            err = (r.stderr or r.stdout or '').strip()
            self.get_logger().warning(f'person overlay exit {r.returncode}: {err}')
            return
        tail = (r.stdout or '').strip()
        if tail:
            self.get_logger().info(tail)

    def _run_map_saver(self) -> None:
        try:
            out_dir = self._resolve_map_save_directory()
        except OSError as e:
            self.get_logger().error(f'map save: cannot create directory: {e}')
            self._shutdown_if_exit_after_save()
            return

        base = self._save_map_basename_raw.strip()
        if not base:
            base = datetime.now().strftime('coverage_map_%Y%m%d_%H%M%S')
        base = Path(base).name
        if not base:
            base = datetime.now().strftime('coverage_map_%Y%m%d_%H%M%S')
        full_stub = str((out_dir / base).resolve())

        ros_args: List[str] = ['--ros-args', '-p', f'map_topic:={self._map_topic}']
        if self._ros_use_sim_time():
            ros_args += ['-p', 'use_sim_time:=true']

        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', full_stub, *ros_args]
        self.get_logger().info(f'map_saver_cli: {" ".join(cmd)}')
        try:
            r = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=120.0,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired:
            self.get_logger().error('map_saver_cli timed out (120s)')
            self._shutdown_if_exit_after_save()
            return
        except FileNotFoundError:
            self.get_logger().error('ros2 / map_saver_cli not found in PATH (need nav2_map_server)')
            self._shutdown_if_exit_after_save()
            return

        if r.returncode != 0:
            err = (r.stderr or r.stdout or '').strip()
            self.get_logger().error(f'map_saver_cli failed ({r.returncode}): {err}')
            self._shutdown_if_exit_after_save()
            return

        self.get_logger().info(f'map saved: {full_stub}.pgm + .yaml')
        self._maybe_post_save_person_overlay(Path(full_stub).with_suffix('.yaml'))
        self._shutdown_if_exit_after_save()

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._current_pose = Pose2D(
            float(p.x),
            float(p.y),
            normalize_angle(yaw_from_quaternion(o.z, o.w)),
        )
        if not self._route_ready:
            self._route_ready = True
            self.get_logger().info('coverage_patrol: ready (phase1 wall-follow + path record).')

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg
        if self._laser_range_from_scan:
            rm = float(msg.range_max)
            if math.isfinite(rm) and rm > 0.3:
                self._scan_range_max_m = rm

    def _effective_laser_m(self) -> float:
        r = self._laser_range_fallback_m
        if self._laser_range_from_scan and self._scan_range_max_m > 0.35:
            r = min(self._scan_range_max_m, self._laser_range_cap_m)
        else:
            r = min(r, self._laser_range_cap_m)
        return max(r, 0.35)

    def _obs_threshold_m(self) -> float:
        return max(
            self._obstacle_distance,
            self._route_contour_body_widths * 0.82 * self._robot_size_m,
        )

    def _append_perimeter_sample(self) -> None:
        if self._current_pose is None or self._perimeter_subphase_seek:
            return
        x, y = self._current_pose.x, self._current_pose.y
        if not self._p1_samples:
            self._p1_samples.append((x, y))
            return
        lx, ly = self._p1_samples[-1]
        if math.hypot(x - lx, y - ly) >= self._p1_sample_min_step_m:
            self._p1_samples.append((x, y))

    def _path_centroid(self, pts: List[Tuple[float, float]]) -> Tuple[float, float]:
        n = len(pts)
        return sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n

    def _estimate_path_ccw(self, pts: List[Tuple[float, float]], cx: float, cy: float) -> bool:
        s = 0.0
        for i in range(len(pts) - 1):
            ax = pts[i + 1][0] - pts[i][0]
            ay = pts[i + 1][1] - pts[i][1]
            mx = pts[i][0] - cx
            my = pts[i][1] - cy
            s += ax * my - ay * mx
        return s > 0.0

    @staticmethod
    def _polyline_arc_index_back(pts: List[Tuple[float, float]], i: int, arc_m: float) -> int:
        dsum = 0.0
        j = i
        while j > 0:
            j -= 1
            dsum += math.hypot(
                pts[j + 1][0] - pts[j][0],
                pts[j + 1][1] - pts[j][1],
            )
            if dsum >= arc_m:
                return j
        return 0

    @staticmethod
    def _polyline_arc_index_fwd(pts: List[Tuple[float, float]], i: int, arc_m: float) -> int:
        n = len(pts)
        dsum = 0.0
        k = i
        while k < n - 1:
            dsum += math.hypot(
                pts[k + 1][0] - pts[k][0],
                pts[k + 1][1] - pts[k][1],
            )
            k += 1
            if dsum >= arc_m:
                return k
        return n - 1

    def _detect_corner_indices(self, pts: List[Tuple[float, float]]) -> List[int]:
        """在顶点 i 处，用沿折线各走 arc_m 后的入射/出射弦向量算转角（适配密采样+平滑沿墙）。"""
        thr = math.radians(self._p1_corner_deg)
        arc = self._p1_corner_arc_m
        min_eucl = 0.04
        n = len(pts)
        out: List[int] = []
        for i in range(1, n - 1):
            ja = self._polyline_arc_index_back(pts, i, arc)
            ka = self._polyline_arc_index_fwd(pts, i, arc)
            ax = pts[i][0] - pts[ja][0]
            ay = pts[i][1] - pts[ja][1]
            bx = pts[ka][0] - pts[i][0]
            by = pts[ka][1] - pts[i][1]
            la, lb = math.hypot(ax, ay), math.hypot(bx, by)
            if la < min_eucl or lb < min_eucl:
                continue
            ax, ay = ax / la, ay / la
            bx, by = bx / lb, by / lb
            c = ax * bx + ay * by
            s = ax * by - ay * bx
            if abs(math.atan2(s, c)) > thr:
                out.append(i)
        return out

    def _merge_close_corner_indices(
        self, pts: List[Tuple[float, float]], idxs: List[int]
    ) -> List[int]:
        if not idxs:
            return []
        merged = [idxs[0]]
        for j in idxs[1:]:
            pj, plast = pts[j], pts[merged[-1]]
            if math.hypot(pj[0] - plast[0], pj[1] - plast[1]) > self._route_corner_merge_m:
                merged.append(j)
        return merged

    def _room_center_xy(self) -> Tuple[float, float]:
        return (
            0.5 * (self._room_min_x + self._room_max_x),
            0.5 * (self._room_min_y + self._room_max_y),
        )

    def _build_post_perimeter_waypoints(self) -> None:
        self._waypoints = []
        self._current_index = 0
        pts = self._p1_samples
        margin = self._route_margin + 0.12
        mx, my = self._room_center_xy()

        def finish_center_only(msg: str) -> None:
            self.get_logger().warning(msg)
            self._waypoints = [(mx, my, 'room center finish')]
            cx, cy = (self._path_centroid(pts) if len(pts) >= 4 else (mx, my))
            self._p1_ccw = self._estimate_path_ccw(pts, cx, cy) if len(pts) >= 4 else True

        if len(pts) < 8:
            finish_center_only(f'Recorded path short ({len(pts)} pts); only room center.')
            return

        r_laser = self._effective_laser_m()
        cx, cy = self._path_centroid(pts)
        corners = self._merge_close_corner_indices(pts, self._detect_corner_indices(pts))
        self._p1_ccw = self._estimate_path_ccw(pts, cx, cy)

        if not corners:
            finish_center_only('No sharp corners on path; only room center.')
            return

        k = 1
        max_k = 40
        layers = 0
        while k <= max_k:
            layer: List[Waypoint] = []
            for ci in corners:
                px, py = pts[ci]
                ux, uy = cx - px, cy - py
                nm = math.hypot(ux, uy)
                if nm < 1e-3:
                    ux, uy = 0.0, 1.0
                else:
                    ux, uy = ux / nm, uy / nm
                gx = px + float(k) * r_laser * ux
                gy = py + float(k) * r_laser * uy
                gx = clamp(gx, self._room_min_x + margin, self._room_max_x - margin)
                gy = clamp(gy, self._room_min_y + margin, self._room_max_y - margin)
                layer.append((gx, gy, f'route L{k} c{ci}'))
            min_d = float('inf')
            if len(layer) >= 2:
                for i in range(1, len(layer)):
                    d = math.hypot(layer[i][0] - layer[i - 1][0], layer[i][1] - layer[i - 1][1])
                    min_d = min(min_d, d)
            if k > 1 and len(layer) >= 2 and min_d < 2.0 * r_laser:
                break
            self._waypoints.extend(layer)
            layers += 1
            k += 1

        self._waypoints.append((mx, my, 'room center finish'))
        self.get_logger().info(
            f'Post-perimeter: {len(corners)} corners, {layers} layer(s), R={r_laser:.2f}m, '
            f'{len(self._waypoints)} wps, ccw={self._p1_ccw}'
        )

    def _range_at_bearing_deg(self, scan: LaserScan, deg: float) -> Optional[float]:
        target = math.radians(deg)
        ang = float(scan.angle_min)
        best_da = 1e9
        best_v: Optional[float] = None
        for sample in scan.ranges:
            a = normalize_angle(ang)
            da = abs(normalize_angle(a - target))
            v = float(sample)
            ok = math.isfinite(v) and float(scan.range_min) <= v <= float(scan.range_max)
            if ok and da < best_da:
                best_da = da
                best_v = v
            ang += float(scan.angle_increment)
        return best_v

    def _wall_tangent_bearing_rad(self, scan: LaserScan, right_hand: bool) -> Optional[float]:
        if right_hand:
            b1, b2 = -88.0, -48.0
            lo, hi = -102.0, -35.0
        else:
            b1, b2 = 48.0, 88.0
            lo, hi = 35.0, 102.0
        r1 = self._range_at_bearing_deg(scan, b1)
        r2 = self._range_at_bearing_deg(scan, b2)
        if r1 is None or r2 is None:
            ang = float(scan.angle_min)
            best_r = float('inf')
            best_a = 0.0
            found = False
            lo_r, hi_r = math.radians(lo), math.radians(hi)
            for sample in scan.ranges:
                a = normalize_angle(ang)
                if lo_r <= a <= hi_r:
                    v = float(sample)
                    if math.isfinite(v) and scan.range_min <= v <= scan.range_max and v < best_r:
                        best_r = v
                        best_a = a
                        found = True
                ang += float(scan.angle_increment)
            if not found:
                return None
            return normalize_angle(best_a + (math.pi / 2.0 if right_hand else -math.pi / 2.0))
        a1, a2 = math.radians(b1), math.radians(b2)
        p1x, p1y = r1 * math.cos(a1), r1 * math.sin(a1)
        p2x, p2y = r2 * math.cos(a2), r2 * math.sin(a2)
        dx, dy = p2x - p1x, p2y - p1y
        if dx * dx + dy * dy < 0.01:
            return None
        return math.atan2(dy, dx)

    def _wf_ema_angle(self, prev: Optional[float], raw: float, alpha: float) -> float:
        if prev is None:
            return raw
        return normalize_angle(prev + alpha * normalize_angle(raw - prev))

    def _wf_apply_angular_slew(self, w_desired: float) -> float:
        step = self._wf_w_slew * self._ctrl_period_s
        w = clamp(w_desired, self._wf_last_w_cmd - step, self._wf_last_w_cmd + step)
        return clamp(w, -self._wall_follow_w_lim, self._wall_follow_w_lim)

    def _sector_min(self, min_deg: float, max_deg: float) -> Optional[float]:
        if self._last_scan is None:
            return None
        msg = self._last_scan
        min_rad, max_rad = math.radians(min_deg), math.radians(max_deg)
        angle = float(msg.angle_min)
        best = float('inf')
        found = False
        for sample in msg.ranges:
            a = normalize_angle(angle)
            v = float(sample)
            if min_rad <= a <= max_rad and math.isfinite(v):
                if msg.range_min <= v <= msg.range_max:
                    best = min(best, v)
                    found = True
            angle += float(msg.angle_increment)
        return best if found else None

    def _wall_follow_perimeter_twist(self) -> Twist:
        scan = self._last_scan
        assert scan is not None
        right = self._wall_follow_right
        if right:
            side_lo, side_hi = -105.0, -38.0
            lost_turn, corner_turn = 1.0, 1.0
        else:
            side_lo, side_hi = 38.0, 105.0
            lost_turn, corner_turn = -1.0, -1.0

        front = self._sector_min(-32.0, 32.0)
        side_raw = self._sector_min(side_lo, side_hi)
        tw = Twist()

        if front is not None and front < self._perimeter_front_block:
            self._wf_smooth_tangent = None
            tw.linear.x = 0.0
            w0 = 0.58 * self._wall_follow_w_lim * corner_turn
            w_corner = clamp(
                w0 + self._perimeter_corner_extra * corner_turn,
                -self._wall_follow_w_lim,
                self._wall_follow_w_lim,
            )
            tw.angular.z = self._wf_apply_angular_slew(w_corner)
            self._wf_last_w_cmd = tw.angular.z
            self._wf_last_v_cmd = tw.linear.x
            return tw

        if side_raw is None or side_raw > self._perimeter_wall_lost_m:
            self._wf_smooth_tangent = None
            self._wf_smooth_side = None
            w_lost = clamp(
                0.42 * self._wall_follow_w_lim * lost_turn,
                -self._wall_follow_w_lim,
                self._wall_follow_w_lim,
            )
            if self._perimeter_pure_turn_drive:
                tw.linear.x = 0.0
                self._wf_last_v_cmd = 0.0
                tw.angular.z = self._wf_apply_angular_slew(w_lost)
            else:
                tw.linear.x = 0.32
                tw.angular.z = self._wf_apply_angular_slew(w_lost)
            self._wf_last_w_cmd = tw.angular.z
            self._wf_last_v_cmd = tw.linear.x
            return tw

        side = float(side_raw)
        if self._wf_smooth_side is None:
            self._wf_smooth_side = side
        else:
            self._wf_smooth_side = self._wf_side_ema * side + (1.0 - self._wf_side_ema) * self._wf_smooth_side

        tangent_raw = self._wall_tangent_bearing_rad(scan, right) or 0.0
        # EMA 只平滑「激光估计的墙向/侧距」，减小噪声；不替代下面的 v/ω 互斥决策
        self._wf_smooth_tangent = self._wf_ema_angle(
            self._wf_smooth_tangent, float(tangent_raw), self._wf_tan_ema
        )
        tangent_s = self._wf_smooth_tangent

        rs = self._robot_size_m
        d_near = rs * max(self._wall_dist_near_mult, 0.25)
        d_far = rs * max(self._wall_dist_far_mult, d_near / rs + 0.15)
        h = max(self._wall_band_hyst_m, 0.02)
        wall_dir = 1.0 if right else -1.0
        side_s = self._wf_smooth_side
        if side_s < d_near + h:
            t = clamp(((d_near + h) - side_s) / max(h, 0.02), 0.0, 1.0)
            bias = wall_dir * t * self._wall_bias_away_rad
        elif side_s > d_far - h:
            t = clamp((side_s - (d_far - h)) / max(h, 0.02), 0.0, 1.0)
            bias = -wall_dir * t * self._wall_bias_toward_rad
        else:
            bias = 0.0

        err = normalize_angle(tangent_s + bias)
        if abs(err) < self._wf_heading_deadband:
            err = 0.0

        v_scale = clamp(side_s / max(d_far, 0.15), 0.45, 1.0)
        v_des = self._wall_follow_v * v_scale
        v_floor = max(self._min_linear_speed * 0.5, 0.0)

        if self._perimeter_pure_turn_drive:
            if abs(err) > self._perimeter_align_heading_rad:
                tw.linear.x = 0.0
                self._wf_last_v_cmd = 0.0
                w_des = clamp(
                    self._wall_parallel_kp * err,
                    -self._wall_follow_w_lim,
                    self._wall_follow_w_lim,
                )
                tw.angular.z = self._wf_apply_angular_slew(w_des)
            else:
                tw.angular.z = 0.0
                self._wf_last_w_cmd = 0.0
                tw.linear.x = self._wf_v_ema * v_des + (1.0 - self._wf_v_ema) * self._wf_last_v_cmd
                tw.linear.x = max(tw.linear.x, v_floor)
        else:
            tw.linear.x = self._wf_v_ema * v_des + (1.0 - self._wf_v_ema) * self._wf_last_v_cmd
            tw.linear.x = max(tw.linear.x, v_floor)
            w_des = clamp(
                self._wall_parallel_kp * err,
                -self._wall_follow_w_lim,
                self._wall_follow_w_lim,
            )
            tw.angular.z = self._wf_apply_angular_slew(w_des)
        self._wf_last_w_cmd = tw.angular.z
        self._wf_last_v_cmd = tw.linear.x
        return tw

    def _lap_closed(self) -> bool:
        if self._perimeter_subphase_seek or self._lap_xy is None or self._current_pose is None:
            return False
        if self._perimeter_traveled_m < max(self._perimeter_min_travel, 0.5):
            return False
        if self._lap_max_separation_m < max(self._perimeter_lap_min_sep, 0.8):
            return False
        d = math.hypot(self._current_pose.x - self._lap_xy[0], self._current_pose.y - self._lap_xy[1])
        return d < max(self._perimeter_lap_r, 0.35)

    def _finish_perimeter_phase(self) -> None:
        self._perimeter_done = True
        self._build_post_perimeter_waypoints()
        self.get_logger().info(f'Perimeter done; {len(self._waypoints)} waypoint(s).')

    def _tick_perimeter_wall_follow(self) -> None:
        if self._perimeter_subphase_seek:
            front = self._sector_min(-28.0, 28.0)
            if self._wall_follow_right:
                side_lo, side_hi = -105.0, -40.0
            else:
                side_lo, side_hi = 40.0, 105.0
            side = self._sector_min(side_lo, side_hi)
            got_wall = (front is not None and front < self._perimeter_seek_wall_d) or (
                side is not None and side < self._perimeter_seek_wall_d * 1.25
            )
            if got_wall:
                self._perimeter_subphase_seek = False
                if self._current_pose is not None:
                    self._lap_xy = (self._current_pose.x, self._current_pose.y)
                    self._last_odom_xy = self._lap_xy
                    self._perimeter_traveled_m = 0.0
                    self._lap_max_separation_m = 0.0
                self._wf_smooth_side = None
                self._wf_smooth_tangent = None
                self._wf_last_w_cmd = 0.0
                self._wf_last_v_cmd = 0.0
                self.get_logger().info('Perimeter: wall acquired, following.')
            else:
                tw = Twist()
                if self._perimeter_pure_turn_drive:
                    if front is not None and front < self._perimeter_seek_wall_d * 1.85:
                        tw.angular.z = 0.48 if self._wall_follow_right else -0.48
                    else:
                        tw.linear.x = self._perimeter_seek_v
                else:
                    tw.linear.x = self._perimeter_seek_v
                    if front is not None and front < self._perimeter_seek_wall_d * 1.85:
                        tw.angular.z = 0.48 if self._wall_follow_right else -0.48
                self._set_status('Perimeter: seeking wall')
                self._cmd_pub.publish(tw)
                return

        self._append_perimeter_sample()

        if self._current_pose is not None and self._last_odom_xy is not None:
            dx = self._current_pose.x - self._last_odom_xy[0]
            dy = self._current_pose.y - self._last_odom_xy[1]
            self._perimeter_traveled_m += math.hypot(dx, dy)
            self._last_odom_xy = (self._current_pose.x, self._current_pose.y)

        if self._lap_xy is not None and self._current_pose is not None:
            d0 = math.hypot(
                self._current_pose.x - self._lap_xy[0],
                self._current_pose.y - self._lap_xy[1],
            )
            self._lap_max_separation_m = max(self._lap_max_separation_m, d0)

        if self._perimeter_force_finish_m > 0.0 and self._perimeter_traveled_m >= self._perimeter_force_finish_m:
            self._publish_stop()
            self._finish_perimeter_phase()
            return

        if self._lap_closed():
            self._publish_stop()
            self._finish_perimeter_phase()
            return

        if self._last_scan is None:
            self._set_status('Perimeter: waiting for scan')
            return

        self._set_status('Perimeter: wall-follow')
        self._cmd_pub.publish(self._wall_follow_perimeter_twist())

    def _unblock_twist_route(self) -> Twist:
        tw = Twist()
        left = self._sector_min(35.0, 120.0)
        right = self._sector_min(-120.0, -35.0)
        lf = left if left is not None else 99.0
        rt = right if right is not None else 99.0
        if self._p1_ccw:
            tw.angular.z = 0.72 * self._avoid_turn_speed if lf >= rt else -0.52 * self._avoid_turn_speed
        else:
            tw.angular.z = -0.72 * self._avoid_turn_speed if rt >= lf else 0.52 * self._avoid_turn_speed
        tw.linear.x = max(self._min_linear_speed * 0.7, 0.08)
        return tw

    def _tick_route_follow(self, now_ns: int) -> None:
        assert self._current_pose is not None
        goal_x, goal_y, goal_name = self._waypoints[self._current_index]
        dx, dy = goal_x - self._current_pose.x, goal_y - self._current_pose.y
        distance = math.hypot(dx, dy)
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(heading - self._current_pose.yaw)

        if self._current_index != self._p2_exec_for_idx:
            self._p2_exec = 'rotate'
            self._p2_exec_for_idx = self._current_index
            self._p2_blocked_since_ns = None
            self._progress_goal_index = -1

        if self._progress_goal_index != self._current_index:
            self._progress_goal_index = self._current_index
            self._best_goal_distance = distance
            self._last_progress_ns = now_ns
        elif distance < self._best_goal_distance - self._progress_epsilon:
            self._best_goal_distance = distance
            self._last_progress_ns = now_ns

        r_laser = self._effective_laser_m()
        if self._route_skip_within_laser_r and distance < r_laser:
            self._advance_waypoint(goal_name, reason='within laser range')
            return

        if distance <= self._position_tolerance:
            self._advance_waypoint(goal_name)
            return

        stuck_sec = max(self._route_goal_stuck_sec, self._goal_stuck_sec)
        if self._last_progress_ns is not None and self._p2_exec == 'drive':
            if (now_ns - self._last_progress_ns) / 1e9 >= stuck_sec:
                self.get_logger().warning(f'Route: skip (no progress) -> {goal_name}')
                self._advance_waypoint(goal_name, reason='stuck timeout')
                return

        front = self._sector_min(-28.0, 28.0)
        obs = self._obs_threshold_m()
        if front is not None and math.isfinite(front) and front < obs:
            if self._p2_blocked_since_ns is None:
                self._p2_blocked_since_ns = now_ns
            if (now_ns - self._p2_blocked_since_ns) / 1e9 >= self._route_obstacle_giveup_sec:
                self.get_logger().warning(f'Route: skip (blocked) -> {goal_name}')
                self._advance_waypoint(goal_name, reason='obstacle timeout')
                return
            self._set_status(f'Route: contour front={front:.2f}m -> {goal_name}')
            self._cmd_pub.publish(self._unblock_twist_route())
            return
        self._p2_blocked_since_ns = None

        cmd = Twist()
        if self._p2_exec == 'rotate':
            if abs(heading_error) <= self._heading_tolerance:
                self._p2_exec = 'drive'
                self._cmd_pub.publish(cmd)
                return
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(
                self._angular_kp * heading_error,
                -self._angular_speed_limit,
                self._angular_speed_limit,
            )
            self._set_status(f'Route rotate ({self._current_index + 1}/{len(self._waypoints)}) {goal_name}')
            self._cmd_pub.publish(cmd)
            return

        if abs(heading_error) > self._route_drift_realign_rad:
            self._p2_exec = 'rotate'
            self._cmd_pub.publish(cmd)
            return

        v = clamp(self._linear_kp * distance, 0.0, self._linear_speed_limit)
        slow_d = max(self._slowdown_distance, obs + 0.08)
        if front is not None and math.isfinite(front) and front < slow_d:
            scale = clamp((front - obs) / max(slow_d - obs, 1e-3), 0.0, 1.0)
            v = max(self._min_linear_speed, v * scale) if v > 0.0 else 0.0
            if front < obs:
                v = 0.0
        cmd.linear.x = v
        self._set_status(f'Route drive ({self._current_index + 1}/{len(self._waypoints)}) d={distance:.2f}m')
        self._cmd_pub.publish(cmd)

    def _advance_waypoint(self, goal_name: str, reason: Optional[str] = None) -> None:
        self._current_index += 1
        self._progress_goal_index = -1
        self._best_goal_distance = float('inf')
        self._last_progress_ns = None
        self._publish_stop()
        if self._current_index < len(self._waypoints):
            nxt = self._waypoints[self._current_index][2]
            if reason is None:
                self.get_logger().info(f'OK {goal_name} -> next {nxt}')
            else:
                self.get_logger().warning(f'Skip {goal_name} ({reason}) -> {nxt}')
        else:
            self.get_logger().info('All waypoints done.')

    def _publish_stop(self) -> None:
        try:
            self._cmd_pub.publish(Twist())
        except Exception:
            pass

    def _set_status(self, status: str) -> None:
        if status != self._last_status:
            self._last_status = status
            self.get_logger().info(status)

    def _on_timer(self) -> None:
        if self._completed:
            self._publish_stop()
            return

        if self._current_pose is None or not self._route_ready:
            return

        if not self._perimeter_done:
            self._tick_perimeter_wall_follow()
            return

        if self._current_index >= len(self._waypoints):
            self._completed = True
            self._publish_stop()
            self.get_logger().info('coverage_patrol: mission complete.')
            self._schedule_map_save_if_needed()
            return

        self._tick_route_follow(self.get_clock().now().nanoseconds)

    def destroy_node(self) -> bool:
        self._publish_stop()
        if self._map_save_timer is not None:
            self._map_save_timer.cancel()
            self._map_save_timer = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CoveragePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
