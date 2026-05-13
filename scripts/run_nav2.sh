#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

TB3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
MAP_FILE="${MAP_FILE:-${ROS_WS}/src/robot_bringup/maps/map.yaml}"
PLANNER_TYPE="${PLANNER_TYPE:-astar}"
PARAMS_FILE="${PARAMS_FILE:-}"
NAV_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/navigation}"
RVIZ_ENABLED="${TB3_ENABLE_RVIZ:-1}"
GZCLIENT_ENABLED="${TB3_ENABLE_GZCLIENT:-1}"
PATROL_ENABLED="${TB3_AUTO_PATROL:-1}"
DEFAULT_NAV2_RVIZ="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
NAV_LAUNCH_PID=""

cleanup() {
  local exit_code=$?
  if [[ -n "${NAV_LAUNCH_PID}" ]] && kill -0 "${NAV_LAUNCH_PID}" >/dev/null 2>&1; then
    kill "${NAV_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${NAV_LAUNCH_PID}" 2>/dev/null || true
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
echo "[run_nav2] map=${MAP_FILE}"
echo "[run_nav2] planner_type=${PLANNER_TYPE}"
echo "[run_nav2] auto_patrol=${PATROL_ENABLED}"
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
  "map:=${MAP_FILE}"
  "use_rviz:=${RVIZ_LAUNCH_VALUE}"
  "rviz_config:=${RVIZ_CONFIG_FILE}"
  "start_patrol:=${PATROL_LAUNCH_VALUE}"
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

wait "${NAV_LAUNCH_PID}"
