#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

LLM_ENV_FILE="${PROJECT_ROOT}/local_llm.env"
if [[ -f "${LLM_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${LLM_ENV_FILE}"
  set +a
  echo "Loaded LLM env from ${LLM_ENV_FILE} (TJ_LLM_API_URL / key / optional TJ_LLM_MODEL)"
fi

export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
# 强制 RGBD 默认机型，避免用户 shell 中旧的 burger 环境变量覆盖。
export TURTLEBOT3_MODEL="waffle"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
export TB3_ENABLE_SLAM=0
export TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-8}"
# 与 run_simulation.sh 对齐：避免 Gazebo 传感器在无订阅路径上抖停
export TB3_CAMERA_ALWAYS_ON="${TB3_CAMERA_ALWAYS_ON:-1}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/full_system}"
# RViz 使用 tb3_stack 默认 test1.rviz（含 YOLO/地图/激光等）；已在 test1.rviz 中合并 Nav2 GoalTool + Navigation 2 面板。

MAP_FILE="${MAP_FILE:-${ROS_WS}/src/robot_bringup/maps/map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/robot_navigation/config/nav2_params.yaml}"
NAV_LAUNCH_FILE="${ROS_WS}/src/robot_navigation/launch/navigation.launch.py"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "ERROR: map file not found: ${MAP_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "ERROR: nav2 params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi
if [[ ! -f "${NAV_LAUNCH_FILE}" ]]; then
  echo "ERROR: navigation launch file not found: ${NAV_LAUNCH_FILE}" >&2
  exit 1
fi

NAV2_MAP_FILE="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")/$(basename "${MAP_FILE}")"

echo "Full system: 仿真栈（与 run_simulation 同参）+ Nav2（defer 启动，与 run_simulation 一致）+ 语音/LLM/任务/操控。"
echo "纯激光建图/日常导航请用: bash scripts/run_mapping.sh（不经由本脚本）。"
echo "Starting RGBD simulation + YOLO recognition stack first (static-map mode)."
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start
# tb3_stack start 内已 cleanup_old（含旧 Nav2），此处不再重复杀进程，避免日志噪音。

echo "Starting Nav2 with static map: ${MAP_FILE} -> ${NAV2_MAP_FILE} (ASCII workdir avoids launch path mangling)"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
WS_SETUP_BASH="${PROJECT_ROOT}/ros_ws/install/setup.bash"
(
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP_BASH}"
  if [[ -f "${WS_SETUP_BASH}" ]]; then
    # shellcheck source=/dev/null
    source "${WS_SETUP_BASH}"
  else
    echo "ERROR: workspace not built, missing ${WS_SETUP_BASH}" >&2
    exit 1
  fi
  set -u
  exec ros2 launch "${NAV_LAUNCH_FILE}" \
    "use_sim_time:=true" \
    "map:=${NAV2_MAP_FILE}" \
    "params_file:=${PARAMS_FILE}" \
    "defer_navigation_autostart:=true"
) >"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
echo "Nav2 started in background (PID=$!). Logs: ${TB3_LOG_DIR}/nav2.launch.log"
(
  set +e
  tj_nav2_trigger_navigation_manager_startup_after_map_server
) >>"${TB3_LOG_DIR}/nav2_deferred_navigation.log" 2>&1 &
echo "Deferred navigation lifecycle STARTUP helper started (log: ${TB3_LOG_DIR}/nav2_deferred_navigation.log)"

# 语音：task_pipeline 默认 asr_backend=mock 且 enable_mock_voice=false 时不会产生任何识别。
# 此处默认注入 whisper_mic（需 pip install -r robot_interaction/requirements-voice.txt 及 PortAudio）。
# 覆盖方式：export TJ_FULL_SYSTEM_ASR=mock 且启动时传 enable_mock_voice:=true；或显式传 asr_backend:=whisper_file。
echo "Voice/LLM: RViz 使用 test1.rviz（YOLO + Nav2 目标工具）；ASR 默认 \${TJ_FULL_SYSTEM_ASR:-whisper_mic}；LLM 见 TJ_LLM_*。"
echo "Starting voice/LLM/task/manipulation pipeline. Press Ctrl-C here, then run scripts/tb3_stack.sh stop to stop Gazebo."
_extra_launch=()
if [[ "${*:-}" != *asr_backend* ]]; then
  _extra_launch+=("asr_backend:=${TJ_FULL_SYSTEM_ASR:-whisper_mic}")
fi
exec ros2 launch robot_bringup task_pipeline.launch.py "${_extra_launch[@]}" "$@"
