#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
require_command colcon
prepare_output_dirs

cd "${ROS_WS}"
# symlink-install：install/share 下的 config、launch 等尽量链到源码，避免“只改了几行 yaml 忘了 colcon install”的运行时仍为旧副本
OPTS=(--symlink-install "$@")
colcon build "${OPTS[@]}"
echo "Build complete. Source: ${ROS_WS}/install/setup.bash"
