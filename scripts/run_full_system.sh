#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
# 强制 RGBD 默认机型，避免用户 shell 中旧的 burger 环境变量覆盖。
export TURTLEBOT3_MODEL="waffle"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/full_system}"

echo "Starting RGBD simulation + YOLO recognition stack first."
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

echo "Starting voice/LLM/task/manipulation pipeline. Press Ctrl-C here, then run scripts/tb3_stack.sh stop to stop Gazebo."
exec ros2 launch robot_bringup task_pipeline.launch.py "$@"
