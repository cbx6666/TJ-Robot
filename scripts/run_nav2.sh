#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

# 使用 TB3_MODEL 作为显式入口，避免终端残留 TURTLEBOT3_MODEL=burger 污染导航启动。
TB3_MODEL="${TB3_MODEL:-waffle}"
MAP_FILE="${MAP_FILE:-${ROS_WS}/src/robot_bringup/maps/map.yaml}"
PLANNER_TYPE="${PLANNER_TYPE:-astar}"
PARAMS_FILE="${PARAMS_FILE:-}"
NAV_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/navigation}"
RVIZ_ENABLED="${TB3_ENABLE_RVIZ:-1}"
GZCLIENT_ENABLED="${TB3_ENABLE_GZCLIENT:-1}"
PATROL_ENABLED="${TB3_AUTO_PATROL:-1}"
COVERAGE_MONITOR_ENABLED="${COVERAGE_MONITOR_ENABLED:-1}"
COVERAGE_REQUIRED_PERCENT="${COVERAGE_REQUIRED_PERCENT:-95.0}"
PATROL_WAYPOINT_TIMEOUT_SEC="${PATROL_WAYPOINT_TIMEOUT_SEC:-6.0}"
PATROL_WAYPOINT_TIMEOUT_PROGRESS_M="${PATROL_WAYPOINT_TIMEOUT_PROGRESS_M:-0.1}"
PATROL_STUCK_ENABLED="${PATROL_STUCK_ENABLED:-1}"
PATROL_STUCK_MIN_PROGRESS_M="${PATROL_STUCK_MIN_PROGRESS_M:-0.08}"
PATROL_STUCK_WINDOW_SEC="${PATROL_STUCK_WINDOW_SEC:-4.0}"
PATROL_STUCK_CMD_REQUIRED="${PATROL_STUCK_CMD_REQUIRED:-1}"
PATROL_STUCK_NEAR_GOAL_TOLERANCE_M="${PATROL_STUCK_NEAR_GOAL_TOLERANCE_M:-0.35}"
PATROL_FAILED_CORRIDOR_ENABLED="${PATROL_FAILED_CORRIDOR_ENABLED:-1}"
PATROL_FAILED_CORRIDOR_RADIUS_M="${PATROL_FAILED_CORRIDOR_RADIUS_M:-0.6}"
PATROL_FAILED_CORRIDOR_BEHIND_M="${PATROL_FAILED_CORRIDOR_BEHIND_M:-0.8}"
PATROL_FAILED_CORRIDOR_AHEAD_M="${PATROL_FAILED_CORRIDOR_AHEAD_M:-2.0}"
PATROL_FAILED_CORRIDOR_TTL_SEC="${PATROL_FAILED_CORRIDOR_TTL_SEC:-240}"
PATROL_FAILED_CORRIDOR_MAX="${PATROL_FAILED_CORRIDOR_MAX:-12}"
PATROL_APPROACH_OFFSETS_ENABLED="${PATROL_APPROACH_OFFSETS_ENABLED:-1}"
PATROL_APPROACH_OFFSET_M="${PATROL_APPROACH_OFFSET_M:-0.6}"
PATROL_APPROACH_YAW_VARIANTS_ENABLED="${PATROL_APPROACH_YAW_VARIANTS_ENABLED:-1}"
PATROL_ESCAPE_ENABLED="${PATROL_ESCAPE_ENABLED:-1}"
PATROL_ESCAPE_CMD_TOPIC="${PATROL_ESCAPE_CMD_TOPIC:-/cmd_vel}"
PATROL_ESCAPE_MIN_MOVE_M="${PATROL_ESCAPE_MIN_MOVE_M:-0.10}"
PATROL_ESCAPE_MAX_ATTEMPTS="${PATROL_ESCAPE_MAX_ATTEMPTS:-2}"
PATROL_ESCAPE_SEQUENCE_ATTEMPTS="${PATROL_ESCAPE_SEQUENCE_ATTEMPTS:-1}"
PATROL_ESCAPE_BACKWARD_SPEED="${PATROL_ESCAPE_BACKWARD_SPEED:-0.12}"
PATROL_ESCAPE_FORWARD_SPEED="${PATROL_ESCAPE_FORWARD_SPEED:-0.08}"
PATROL_ESCAPE_TURN_SPEED="${PATROL_ESCAPE_TURN_SPEED:-0.50}"
PATROL_ESCAPE_BACKWARD_DURATION_SEC="${PATROL_ESCAPE_BACKWARD_DURATION_SEC:-1.0}"
PATROL_ESCAPE_ROTATE_DURATION_SEC="${PATROL_ESCAPE_ROTATE_DURATION_SEC:-0.7}"
PATROL_ESCAPE_ARC_DURATION_SEC="${PATROL_ESCAPE_ARC_DURATION_SEC:-1.0}"
PATROL_NO_MOTION_AFTER_GOAL_SEC="${PATROL_NO_MOTION_AFTER_GOAL_SEC:-3.0}"
PATROL_NO_MOTION_MAX_COUNT="${PATROL_NO_MOTION_MAX_COUNT:-2}"
COVERAGE_PLANNER_ENABLED="${COVERAGE_PLANNER_ENABLED:-1}"
COVERAGE_SAMPLE_SPACING_M="${COVERAGE_SAMPLE_SPACING_M:-1.10}"
COVERAGE_MAX_WAYPOINTS="${COVERAGE_MAX_WAYPOINTS:-40}"
WAYPOINT_MIN_OBSTACLE_DISTANCE_M="${WAYPOINT_MIN_OBSTACLE_DISTANCE_M:-0.35}"
DEFAULT_NAV2_RVIZ="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
NAV_LAUNCH_PID=""

cleanup() {
  local exit_code=$?
  if [[ -n "${NAV_LAUNCH_PID}" ]] && kill -0 "${NAV_LAUNCH_PID}" >/dev/null 2>&1; then
    # 只 kill launch 父进程时，rviz2 / 各 nav2 子进程有时仍存活，终端会像「卡死」。
    pkill -TERM -P "${NAV_LAUNCH_PID}" 2>/dev/null || true
    sleep 0.4
    kill -TERM "${NAV_LAUNCH_PID}" 2>/dev/null || true
    wait "${NAV_LAUNCH_PID}" 2>/dev/null || true
    kill -KILL "${NAV_LAUNCH_PID}" 2>/dev/null || true
  fi
  bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" stop >/dev/null 2>&1 || true
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

require_command ros2
require_command gzserver
if [[ "${RVIZ_ENABLED}" == "1" ]]; then
  require_command rviz2
fi

if ! ros2 pkg prefix robot_navigation >/dev/null 2>&1; then
  echo "ERROR: robot_navigation package is not visible in this shell." >&2
  echo "Run first: bash scripts/build.sh --symlink-install --packages-select robot_navigation" >&2
  echo "Then source: source ${ROS_WS}/install/setup.bash" >&2
  exit 1
fi

mkdir -p "${NAV_LOG_DIR}"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "ERROR: map file not found: ${MAP_FILE}" >&2
  exit 1
fi

if [[ -n "${PARAMS_FILE}" && ! -f "${PARAMS_FILE}" ]]; then
  echo "ERROR: Nav2 params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi

if [[ "${PLANNER_TYPE}" != "astar" && "${PLANNER_TYPE}" != "dijkstra" ]]; then
  echo "ERROR: PLANNER_TYPE must be astar or dijkstra, got: ${PLANNER_TYPE}" >&2
  exit 1
fi

if [[ "${RVIZ_ENABLED}" != "0" && "${RVIZ_ENABLED}" != "1" ]]; then
  echo "ERROR: TB3_ENABLE_RVIZ must be 0 or 1, got: ${RVIZ_ENABLED}" >&2
  exit 1
fi

if [[ "${PATROL_ENABLED}" != "0" && "${PATROL_ENABLED}" != "1" ]]; then
  echo "ERROR: TB3_AUTO_PATROL must be 0 or 1, got: ${PATROL_ENABLED}" >&2
  exit 1
fi
for bool_var in \
  PATROL_STUCK_ENABLED \
  PATROL_STUCK_CMD_REQUIRED \
  PATROL_FAILED_CORRIDOR_ENABLED \
  PATROL_APPROACH_OFFSETS_ENABLED \
  PATROL_APPROACH_YAW_VARIANTS_ENABLED \
  PATROL_ESCAPE_ENABLED; do
  bool_value="${!bool_var}"
  if [[ "${bool_value}" != "0" && "${bool_value}" != "1" ]]; then
    echo "ERROR: ${bool_var} must be 0 or 1, got: ${bool_value}" >&2
    exit 1
  fi
done
if [[ "${COVERAGE_MONITOR_ENABLED}" != "0" && "${COVERAGE_MONITOR_ENABLED}" != "1" ]]; then
  echo "ERROR: COVERAGE_MONITOR_ENABLED must be 0 or 1, got: ${COVERAGE_MONITOR_ENABLED}" >&2
  exit 1
fi
if [[ "${COVERAGE_PLANNER_ENABLED}" != "0" && "${COVERAGE_PLANNER_ENABLED}" != "1" ]]; then
  echo "ERROR: COVERAGE_PLANNER_ENABLED must be 0 or 1, got: ${COVERAGE_PLANNER_ENABLED}" >&2
  exit 1
fi

if [[ "${RVIZ_ENABLED}" == "1" ]]; then
  RVIZ_LAUNCH_VALUE="true"
else
  RVIZ_LAUNCH_VALUE="false"
fi

if [[ "${PATROL_ENABLED}" == "1" ]]; then
  PATROL_LAUNCH_VALUE="true"
else
  PATROL_LAUNCH_VALUE="false"
fi
if [[ "${COVERAGE_MONITOR_ENABLED}" == "1" ]]; then
  COVERAGE_MONITOR_LAUNCH_VALUE="true"
else
  COVERAGE_MONITOR_LAUNCH_VALUE="false"
fi
if [[ "${COVERAGE_PLANNER_ENABLED}" == "1" ]]; then
  COVERAGE_PLANNER_LAUNCH_VALUE="true"
else
  COVERAGE_PLANNER_LAUNCH_VALUE="false"
fi

# map_server 对 map:= 路径在非 ASCII 目录下可能编码错误；拷到 /tmp 下 ASCII 路径再传给 Nav2（与 run_full_system 一致）
NAV2_MAP_FILE="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")/$(basename "${MAP_FILE}")"

export TURTLEBOT3_MODEL="${TB3_MODEL}"
export TB3_STACK_MODE="${TB3_STACK_MODE:-laser}"
export TB3_ENABLE_SLAM=0
export TB3_ENABLE_CAMERA="${TB3_ENABLE_CAMERA:-0}"
export TB3_ASSIST_RGBD_BRIDGE=0
export TB3_ASSIST_SCAN_FILTER=0
export TB3_ENABLE_GZCLIENT="${GZCLIENT_ENABLED}"
# Nav2 owns RViz in this script. The base stack only starts Gazebo and the robot,
# otherwise one RViz is launched by tb3_stack.sh and another by Nav2.
export TB3_ENABLE_RVIZ=0
export TB3_LOG_DIR="${NAV_LOG_DIR}"
if [[ -z "${RVIZ_CONFIG_FILE:-}" ]]; then
  if [[ -f "${DEFAULT_NAV2_RVIZ}" ]]; then
    export RVIZ_CONFIG_FILE="${DEFAULT_NAV2_RVIZ}"
  else
    export RVIZ_CONFIG_FILE="${ROS_WS}/src/robot_bringup/config/test1.rviz"
    echo "[run_nav2] WARNING: Nav2 default RViz config not found, fallback to ${RVIZ_CONFIG_FILE}" >&2
  fi
fi

echo "[run_nav2] project_root=${PROJECT_ROOT}"
echo "[run_nav2] workspace=${ROS_WS}"
echo "[run_nav2] model=${TURTLEBOT3_MODEL}"
echo "[run_nav2] map_source=${MAP_FILE}"
echo "[run_nav2] map_nav2=${NAV2_MAP_FILE}"
echo "[run_nav2] planner_type=${PLANNER_TYPE}"
echo "[run_nav2] auto_patrol=${PATROL_ENABLED}"
echo "[run_nav2] coverage_monitor=${COVERAGE_MONITOR_ENABLED} required=${COVERAGE_REQUIRED_PERCENT}%"
echo "[run_nav2] coverage_planner=${COVERAGE_PLANNER_ENABLED} spacing=${COVERAGE_SAMPLE_SPACING_M}m max=${COVERAGE_MAX_WAYPOINTS} clearance=${WAYPOINT_MIN_OBSTACLE_DISTANCE_M}m"
echo "[run_nav2] patrol_timeout=${PATROL_WAYPOINT_TIMEOUT_SEC}s progress=${PATROL_WAYPOINT_TIMEOUT_PROGRESS_M}m"
echo "[run_nav2] patrol_stuck=${PATROL_STUCK_ENABLED} min_progress=${PATROL_STUCK_MIN_PROGRESS_M}m window=${PATROL_STUCK_WINDOW_SEC}s cmd_required=${PATROL_STUCK_CMD_REQUIRED} near_goal=${PATROL_STUCK_NEAR_GOAL_TOLERANCE_M}m"
echo "[run_nav2] failed_corridor=${PATROL_FAILED_CORRIDOR_ENABLED} radius=${PATROL_FAILED_CORRIDOR_RADIUS_M}m behind=${PATROL_FAILED_CORRIDOR_BEHIND_M}m ahead=${PATROL_FAILED_CORRIDOR_AHEAD_M}m ttl=${PATROL_FAILED_CORRIDOR_TTL_SEC}s max=${PATROL_FAILED_CORRIDOR_MAX}"
echo "[run_nav2] approach_offsets=${PATROL_APPROACH_OFFSETS_ENABLED} offset=${PATROL_APPROACH_OFFSET_M}m yaw_variants=${PATROL_APPROACH_YAW_VARIANTS_ENABLED}"
echo "[run_nav2] escape=${PATROL_ESCAPE_ENABLED} cmd_topic=${PATROL_ESCAPE_CMD_TOPIC} min_move=${PATROL_ESCAPE_MIN_MOVE_M}m events=${PATROL_ESCAPE_MAX_ATTEMPTS} seq_attempts=${PATROL_ESCAPE_SEQUENCE_ATTEMPTS} backward=${PATROL_ESCAPE_BACKWARD_SPEED}m/s@${PATROL_ESCAPE_BACKWARD_DURATION_SEC}s forward=${PATROL_ESCAPE_FORWARD_SPEED}m/s arc=${PATROL_ESCAPE_ARC_DURATION_SEC}s turn=${PATROL_ESCAPE_TURN_SPEED}rad/s rotate=${PATROL_ESCAPE_ROTATE_DURATION_SEC}s"
echo "[run_nav2] no_motion_after_goal=${PATROL_NO_MOTION_AFTER_GOAL_SEC}s max_count=${PATROL_NO_MOTION_MAX_COUNT}"
if [[ -n "${PARAMS_FILE}" ]]; then
  echo "[run_nav2] params=${PARAMS_FILE}"
else
  echo "[run_nav2] params=selected by planner_type"
fi
echo "[run_nav2] rviz_config=${RVIZ_CONFIG_FILE}"
echo "[run_nav2] rviz_owner=Nav2 launch"
echo "[run_nav2] log_dir=${TB3_LOG_DIR}"

echo "[1/2] Starting Gazebo + robot base"
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

echo "[2/2] Starting Nav2 localization + navigation stack"
NAV2_ARGS=(
  "planner_type:=${PLANNER_TYPE}"
  "use_sim_time:=true"
  "map:=${NAV2_MAP_FILE}"
  "use_rviz:=${RVIZ_LAUNCH_VALUE}"
  "rviz_config:=${RVIZ_CONFIG_FILE}"
  "start_patrol:=${PATROL_LAUNCH_VALUE}"
  "start_coverage_monitor:=${COVERAGE_MONITOR_LAUNCH_VALUE}"
  "coverage_required_percent:=${COVERAGE_REQUIRED_PERCENT}"
  "waypoint_timeout_sec:=${PATROL_WAYPOINT_TIMEOUT_SEC}"
  "waypoint_timeout_progress_m:=${PATROL_WAYPOINT_TIMEOUT_PROGRESS_M}"
  "patrol_stuck_enabled:=${PATROL_STUCK_ENABLED}"
  "patrol_stuck_min_progress_m:=${PATROL_STUCK_MIN_PROGRESS_M}"
  "patrol_stuck_window_sec:=${PATROL_STUCK_WINDOW_SEC}"
  "patrol_stuck_cmd_required:=${PATROL_STUCK_CMD_REQUIRED}"
  "patrol_stuck_near_goal_tolerance_m:=${PATROL_STUCK_NEAR_GOAL_TOLERANCE_M}"
  "patrol_failed_corridor_enabled:=${PATROL_FAILED_CORRIDOR_ENABLED}"
  "patrol_failed_corridor_radius_m:=${PATROL_FAILED_CORRIDOR_RADIUS_M}"
  "patrol_failed_corridor_behind_m:=${PATROL_FAILED_CORRIDOR_BEHIND_M}"
  "patrol_failed_corridor_ahead_m:=${PATROL_FAILED_CORRIDOR_AHEAD_M}"
  "patrol_failed_corridor_ttl_sec:=${PATROL_FAILED_CORRIDOR_TTL_SEC}"
  "patrol_failed_corridor_max:=${PATROL_FAILED_CORRIDOR_MAX}"
  "patrol_approach_offsets_enabled:=${PATROL_APPROACH_OFFSETS_ENABLED}"
  "patrol_approach_offset_m:=${PATROL_APPROACH_OFFSET_M}"
  "patrol_approach_yaw_variants_enabled:=${PATROL_APPROACH_YAW_VARIANTS_ENABLED}"
  "patrol_escape_enabled:=${PATROL_ESCAPE_ENABLED}"
  "patrol_escape_cmd_topic:=${PATROL_ESCAPE_CMD_TOPIC}"
  "patrol_escape_min_move_m:=${PATROL_ESCAPE_MIN_MOVE_M}"
  "patrol_escape_max_attempts:=${PATROL_ESCAPE_MAX_ATTEMPTS}"
  "patrol_escape_sequence_attempts:=${PATROL_ESCAPE_SEQUENCE_ATTEMPTS}"
  "patrol_escape_backward_speed:=${PATROL_ESCAPE_BACKWARD_SPEED}"
  "patrol_escape_forward_speed:=${PATROL_ESCAPE_FORWARD_SPEED}"
  "patrol_escape_turn_speed:=${PATROL_ESCAPE_TURN_SPEED}"
  "patrol_escape_backward_duration_sec:=${PATROL_ESCAPE_BACKWARD_DURATION_SEC}"
  "patrol_escape_rotate_duration_sec:=${PATROL_ESCAPE_ROTATE_DURATION_SEC}"
  "patrol_escape_arc_duration_sec:=${PATROL_ESCAPE_ARC_DURATION_SEC}"
  "patrol_no_motion_after_goal_sec:=${PATROL_NO_MOTION_AFTER_GOAL_SEC}"
  "patrol_no_motion_max_count:=${PATROL_NO_MOTION_MAX_COUNT}"
  "coverage_planner_enabled:=${COVERAGE_PLANNER_LAUNCH_VALUE}"
  "coverage_sample_spacing_m:=${COVERAGE_SAMPLE_SPACING_M}"
  "coverage_max_waypoints:=${COVERAGE_MAX_WAYPOINTS}"
  "waypoint_min_obstacle_distance_m:=${WAYPOINT_MIN_OBSTACLE_DISTANCE_M}"
)
if [[ -n "${PARAMS_FILE}" ]]; then
  NAV2_ARGS+=("params_file:=${PARAMS_FILE}")
fi

ros2 launch robot_navigation nav2_patrol.launch.py "${NAV2_ARGS[@]}" \
  >"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
NAV_LAUNCH_PID=$!

echo "[run_nav2] Nav2 launch PID=${NAV_LAUNCH_PID}"
if [[ "${PATROL_ENABLED}" == "1" ]]; then
  echo "[run_nav2] Auto patrol is enabled. The robot will publish its initial pose and visit the configured waypoints."
else
  echo "[run_nav2] Auto patrol is disabled. Use RViz '2D Pose Estimate' and 'Nav2 Goal' manually."
fi
echo "[run_nav2] Logs: ${TB3_LOG_DIR}"
echo "[run_nav2] 脚本会阻塞在此（wait Nav2）；结束请在本终端按 Ctrl+C，将清理 launch 子进程并 tb3_stack stop。"
echo "[run_nav2] 若 Ctrl+C 无效：另开终端 kill ${NAV_LAUNCH_PID} 或 bash scripts/tb3_stack.sh stop"

wait "${NAV_LAUNCH_PID}"
