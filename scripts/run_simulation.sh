#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
# 统一使用 waffle，避免终端残留环境变量（如 TURTLEBOT3_MODEL=burger）干扰。
export TURTLEBOT3_MODEL="waffle"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
export TB3_ENABLE_SLAM=0
# 工程内统一日志目录（须 export：子进程 tb3_stack 会继承；否则会落回 /tmp/tb3_stack 且本脚本里 TB3_LOG_DIR 未定义）
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/simulation}"
export TB3_QUIET_NAV2_CLEANUP="${TB3_QUIET_NAV2_CLEANUP:-1}"
# shellcheck source=lib/tb3_sim_assist_env.sh
source "${SCRIPT_DIR}/lib/tb3_sim_assist_env.sh"
MAP_FILE="${MAP_FILE:-${ROS_WS}/src/robot_bringup/maps/map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/robot_navigation/config/nav2_params.yaml}"
NAV_LAUNCH_FILE="${ROS_WS}/src/robot_navigation/launch/navigation.launch.py"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "ERROR: map file not found: ${MAP_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "ERROR: nav2 params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi
if [[ ! -f "${NAV_LAUNCH_FILE}" ]]; then
  echo "ERROR: navigation launch file not found: ${NAV_LAUNCH_FILE}" >&2
  exit 1
fi

NAV2_MAP_FILE="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")/$(basename "${MAP_FILE}")"

echo "Starting simulation stack (static-map mode): model=${TURTLEBOT3_MODEL}, mode=${TB3_STACK_MODE}, rgbd_bridge=${TB3_ASSIST_RGBD_BRIDGE}, yolo=${TB3_ASSIST_SCAN_FILTER}"
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

echo "Stopping duplicate Nav2 from previous runs (if any) ..."
tj_kill_nav2_background_launch

echo "Starting Nav2 -> ${TB3_LOG_DIR}/nav2.launch.log（控制台仅启动摘要）"
mkdir -p "${TB3_LOG_DIR}"
{
  echo "===== Nav2 launch $(date -Iseconds 2>/dev/null || date) ====="
  echo "  map:=${NAV2_MAP_FILE}"
} >"${TB3_LOG_DIR}/nav2.launch.log"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
WS_SETUP_BASH="${PROJECT_ROOT}/ros_ws/install/setup.bash"
(
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP_BASH}"
  if [[ -f "${WS_SETUP_BASH}" ]]; then
    # shellcheck source=/dev/null
    source "${WS_SETUP_BASH}"
  else
    echo "ERROR: workspace not built, missing ${WS_SETUP_BASH}" >&2
    exit 1
  fi
  set -u
  exec ros2 launch "${NAV_LAUNCH_FILE}" \
    "use_sim_time:=true" \
    "map:=${NAV2_MAP_FILE}" \
    "params_file:=${PARAMS_FILE}" \
    "defer_navigation_autostart:=true"
) >>"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
echo "Nav2 started in background (PID=$!). tail -f ${TB3_LOG_DIR}/nav2.launch.log"
(
  set +e
  tj_nav2_trigger_navigation_manager_startup_after_map_server
) >>"${TB3_LOG_DIR}/nav2_deferred_navigation.log" 2>&1 &
echo "Deferred Nav2 lifecycle -> ${TB3_LOG_DIR}/nav2_deferred_navigation.log"
echo "停仿真请执行: bash scripts/tb3_stack.sh stop  （仍卡请: bash scripts/kill_simulation_stack.sh）"
