#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

# 使用 TB3_MODEL 作为显式入口，避免继承终端残留的 TURTLEBOT3_MODEL（常见为 burger）。
TB3_MODEL="${TB3_MODEL:-waffle}"
export TURTLEBOT3_MODEL="${TB3_MODEL}"
export TB3_STACK_MODE="laser"
export TB3_ENABLE_CAMERA="${TB3_ENABLE_CAMERA:-0}"
export TB3_CAMERA_UPDATE_RATE="0"
export TB3_CAMERA_ALWAYS_ON="0"
export TB3_ASSIST_SCAN_FILTER="0"
export TB3_ASSIST_RGBD_BRIDGE="0"
export TB3_ENABLE_SLAM="${TB3_ENABLE_SLAM:-1}"
export TB3_ENABLE_RVIZ="${TB3_ENABLE_RVIZ:-1}"
# 建图常用默认：仅 RViz，不开 Gazebo GUI；需要时可手动覆盖为 1。
export TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-0}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/mapping}"

echo "Starting mapping stack: model=${TURTLEBOT3_MODEL}, rviz=${TB3_ENABLE_RVIZ}, gazebo_gui=${TB3_ENABLE_GZCLIENT}"
exec bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start
