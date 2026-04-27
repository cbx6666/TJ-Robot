#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
require_command colcon
prepare_output_dirs

cd "${ROS_WS}"
colcon build "$@"
echo "Build complete. Source: ${ROS_WS}/install/setup.bash"
