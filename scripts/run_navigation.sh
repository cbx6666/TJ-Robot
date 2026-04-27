#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

echo "Starting Nav2 navigation wrapper. Start simulation first if it is not running."
exec ros2 launch robot_bringup nav2.launch.py "$@"
