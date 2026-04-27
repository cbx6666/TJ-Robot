#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TB3_STACK_MODE="${TB3_STACK_MODE:-laser}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-0}"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-0}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/mapping_laser}"
export TJ_ROBOT_SAVED_MAPS_DIR="${TJ_ROBOT_SAVED_MAPS_DIR:-${PROJECT_ROOT}/data/maps/baseline_raw}"

echo "Starting mapping-laser stack: model=${TURTLEBOT3_MODEL}, mode=${TB3_STACK_MODE}"
exec bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start
