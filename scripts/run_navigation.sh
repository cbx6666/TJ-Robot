#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

echo "Starting Nav2 navigation wrapper. Start simulation first if it is not running."

extra_args=()
if [[ -n "${TJ_NAV2_BASE_PARAMS:-}" ]]; then
  extra_args+=("nav2_base_params:=${TJ_NAV2_BASE_PARAMS}")
fi
if [[ -n "${TJ_NAV2_PROFILE_PARAMS:-}" ]]; then
  extra_args+=("nav2_profile_params:=${TJ_NAV2_PROFILE_PARAMS}")
fi

exec ros2 launch robot_bringup nav2.launch.py "${extra_args[@]}" "$@"
