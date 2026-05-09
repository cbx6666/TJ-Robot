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
# 默认相机刷新率 8Hz（与此前 run_simulation 行为一致）；未 export 时 tb3_stack 默认 2Hz
export TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-8}"
# 避免 Gazebo 传感器在“无订阅时才渲染”的路径上抖停；与 tb3_stack 默认 0 区分开
export TB3_CAMERA_ALWAYS_ON="${TB3_CAMERA_ALWAYS_ON:-1}"
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

echo "Starting Nav2 with static map: ${MAP_FILE} -> ${NAV2_MAP_FILE} (ASCII workdir avoids launch path mangling)"
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
    "params_file:=${PARAMS_FILE}"
) >"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
echo "Nav2 started in background (PID=$!). Logs: ${TB3_LOG_DIR}/nav2.launch.log"
