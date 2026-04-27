#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/navigation_postmap}"
export TJ_NAV2_PROFILE_PARAMS="${TJ_NAV2_PROFILE_PARAMS:-${PROJECT_ROOT}/ros_ws/src/robot_bringup/config/nav2/profiles/conservative.yaml}"

echo "Starting post-map navigation (RGBD+laser): model=${TURTLEBOT3_MODEL}, profile=${TJ_NAV2_PROFILE_PARAMS}"
exec ros2 launch robot_bringup nav2.launch.py "$@"
