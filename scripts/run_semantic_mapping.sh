#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
export TB3_ASSIST_SCAN_FILTER="1"
export TB3_PERSON_SLAM_MODE="${TB3_PERSON_SLAM_MODE:-mark_then_strip}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/semantic_mapping}"
export TJ_ROBOT_SAVED_MAPS_DIR="${TJ_ROBOT_SAVED_MAPS_DIR:-${PROJECT_ROOT}/data/maps/before_strip}"

echo "Starting semantic mapping: SLAM + YOLO/person regions + strip-ready outputs."
exec bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start
