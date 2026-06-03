#!/usr/bin/env bash
# 激光 SLAM 建图：Gazebo + /scan + slam_toolbox + RViz（不启动 Nav2 / YOLO）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

TB3_MODEL="${TB3_MODEL:-waffle}"
export TB3_MODEL
export TURTLEBOT3_MODEL="${TB3_MODEL}"
export TB3_STACK_MODE="${TB3_STACK_MODE:-laser}"
export TB3_ENABLE_SLAM=1
export TB3_ASSIST_SCAN_FILTER=0
export TB3_ASSIST_RGBD_BRIDGE=0
export TB3_ENABLE_CAMERA=0
export TB3_ENABLE_RVIZ="${TB3_ENABLE_RVIZ:-1}"
export TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-0}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/mapping}"

_default_slam_params="${ROS_WS}/src/robot_bringup/config/mapper_params_online_async_full_scan.yaml"
if [[ -z "${TB3_SLAM_PARAMS_FILE:-}" && -f "${PROJECT_ROOT}/config/slam.yaml" ]]; then
  _params_rel="$(grep -E '^[[:space:]]*params_file:' "${PROJECT_ROOT}/config/slam.yaml" 2>/dev/null | head -1 | sed -E 's/^[[:space:]]*params_file:[[:space:]]*//')"
  if [[ -n "${_params_rel}" ]]; then
    _candidate="${PROJECT_ROOT}/${_params_rel}"
    if [[ -f "${_candidate}" ]]; then
      export TB3_SLAM_PARAMS_FILE="${_candidate}"
    fi
  fi
fi
export TB3_SLAM_PARAMS_FILE="${TB3_SLAM_PARAMS_FILE:-${_default_slam_params}}"

MAP_SAVE_DIR="${MAP_SAVE_DIR:-${ROS_WS}/src/robot_bringup/maps}"

echo "=========================================="
echo " 激光 SLAM 建图（slam_toolbox + /scan）"
echo "=========================================="
echo "  model=${TB3_MODEL}  slam_params=${TB3_SLAM_PARAMS_FILE}"
echo "  rviz=${TB3_ENABLE_RVIZ}  gzclient=${TB3_ENABLE_GZCLIENT}"
echo "  日志: ${TB3_LOG_DIR}"
echo ""
echo "  新终端遥控（需已 source install/setup.bash）:"
echo "    export TURTLEBOT3_MODEL=${TB3_MODEL}"
echo "    ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "  建图满意后保存地图（先停 teleop）:"
echo "    ros2 run nav2_map_server map_saver_cli -f ${MAP_SAVE_DIR}/map"
echo "  停止: bash scripts/tb3_stack.sh stop"
echo "=========================================="
echo ""

bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

echo ""
echo "Stack started. 在 RViz 中查看 /map；遥控走动后地图逐步扩展。"
echo "  tail -f ${TB3_LOG_DIR}/slam_toolbox.log"
echo "  bash scripts/tb3_stack.sh check"
