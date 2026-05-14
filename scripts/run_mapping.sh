#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

# 既然已使用静态地图（pgm/yaml），这里切换为“定位/导航”模式（不再启动 SLAM 建图）。
TB3_MODEL="${TB3_MODEL:-waffle}"
export TB3_MODEL
export TB3_ENABLE_RVIZ="${TB3_ENABLE_RVIZ:-1}"
export TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-0}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/mapping}"

echo "Starting static-map localization stack: model=${TB3_MODEL}, rviz=${TB3_ENABLE_RVIZ}, gazebo_gui=${TB3_ENABLE_GZCLIENT}"
exec bash "${PROJECT_ROOT}/scripts/run_nav2.sh"
