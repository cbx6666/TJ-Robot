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
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/robot_navigation/config/nav2_params.yaml}"
NAV_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/navigation}"
RVIZ_ENABLED="${TB3_ENABLE_RVIZ:-1}"
GZCLIENT_ENABLED="${TB3_ENABLE_GZCLIENT:-1}"
DEFAULT_NAV2_RVIZ="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
NAV_LAUNCH_FILE="${ROS_WS}/src/robot_navigation/launch/navigation.launch.py"
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

mkdir -p "${NAV_LOG_DIR}"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "ERROR: map file not found: ${MAP_FILE}" >&2
  exit 1
fi

if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "ERROR: Nav2 params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi

if [[ ! -f "${NAV_LAUNCH_FILE}" ]]; then
  echo "ERROR: navigation launch file not found: ${NAV_LAUNCH_FILE}" >&2
  exit 1
fi

NAV2_MAP_FILE="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")/$(basename "${MAP_FILE}")"

export TURTLEBOT3_MODEL="${TB3_MODEL}"
export TB3_STACK_MODE="${TB3_STACK_MODE:-laser}"
export TB3_ENABLE_SLAM=0
export TB3_ENABLE_CAMERA="${TB3_ENABLE_CAMERA:-0}"
export TB3_ASSIST_RGBD_BRIDGE=0
export TB3_ASSIST_SCAN_FILTER=0
export TB3_ENABLE_GZCLIENT="${GZCLIENT_ENABLED}"
export TB3_ENABLE_RVIZ="${RVIZ_ENABLED}"
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
echo "[run_nav2] params=${PARAMS_FILE}"
echo "[run_nav2] launch=${NAV_LAUNCH_FILE}"
echo "[run_nav2] rviz_config=${RVIZ_CONFIG_FILE}"
echo "[run_nav2] log_dir=${TB3_LOG_DIR}"

echo "[1/2] Starting Gazebo + robot base"
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

echo "[2/2] Starting Nav2 localization + navigation stack"
ros2 launch "${NAV_LAUNCH_FILE}" \
  "use_sim_time:=true" \
  "map:=${NAV2_MAP_FILE}" \
  "params_file:=${PARAMS_FILE}" \
  >"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
NAV_LAUNCH_PID=$!

echo "[run_nav2] Nav2 launch PID=${NAV_LAUNCH_PID}"
echo "[run_nav2] RViz: use '2D Pose Estimate' to initialize AMCL, then use the Nav2 goal tool to send a target."
echo "[run_nav2] Logs: ${TB3_LOG_DIR}"
echo "[run_nav2] 脚本会阻塞在此（wait Nav2）；结束请在本终端按 Ctrl+C，将清理 launch 子进程并 tb3_stack stop。"
echo "[run_nav2] 若 Ctrl+C 无效：另开终端 kill ${NAV_LAUNCH_PID} 或 bash scripts/tb3_stack.sh stop"

wait "${NAV_LAUNCH_PID}"
