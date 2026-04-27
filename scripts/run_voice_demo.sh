#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"

echo "Starting voice demo: interaction + llm router + task manager + mock manipulation."
exec ros2 launch robot_bringup voice_demo.launch.py "$@"
