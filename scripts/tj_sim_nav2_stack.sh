#!/usr/bin/env bash
# 单一入口：Gazebo+assist（tb3_stack）→ 清旧 Nav2 → 延迟 → ros2 launch tj_static_map_nav2（Nav2）。
# 由 run_simulation.sh / run_mapping.sh / run_full_system.sh（前半段）调用；勿直接改三处重复逻辑，只改本文件。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

PROFILE="${TJ_STACK_PROFILE:-simulation}"

case "${PROFILE}" in
  simulation)
    export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
    export TB3_MODEL="${TB3_MODEL:-waffle}"
    export TURTLEBOT3_MODEL="${TB3_MODEL}"
    export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
    export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
    export TB3_ENABLE_SLAM=0
    export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/simulation}"
    export TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-3}"
    export TB3_CAMERA_ALWAYS_ON="${TB3_CAMERA_ALWAYS_ON:-1}"
    echo "TJ_STACK_PROFILE=simulation (assist + Nav2). Logs: ${TB3_LOG_DIR}"
    ;;
  mapping)
    export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
    export TB3_MODEL="${TB3_MODEL:-waffle}"
    export TURTLEBOT3_MODEL="${TB3_MODEL}"
    export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
    export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
    export TB3_ENABLE_SLAM=0
    export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/mapping}"
    export TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-0}"
    export TB3_ENABLE_RVIZ="${TB3_ENABLE_RVIZ:-1}"
    export TB3_PURGE_MODEL_CACHE_ON_START="${TB3_PURGE_MODEL_CACHE_ON_START:-1}"
    export NAV2_LAUNCH_DELAY_SEC="${NAV2_LAUNCH_DELAY_SEC:-15}"
    echo "TJ_STACK_PROFILE=mapping (assist + Nav2). Logs: ${TB3_LOG_DIR}"
    ;;
  full_system)
    export TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
    export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
    export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
    export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
    export TB3_ENABLE_SLAM=0
    export TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-3}"
    export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/full_system}"
    echo "TJ_STACK_PROFILE=full_system (assist + Nav2，随后由 run_full_system 起 task_pipeline). Logs: ${TB3_LOG_DIR}"
    ;;
  *)
    echo "ERROR: Unknown TJ_STACK_PROFILE=${PROFILE} (use simulation|mapping|full_system)" >&2
    exit 1
    ;;
esac

MAP_FILE="${MAP_FILE:-$(tj_repo_static_map_yaml)}"
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/robot_navigation/config/nav2_params.yaml}"
NAV2_PKG="robot_navigation"
NAV2_LAUNCH_FILE="tj_static_map_nav2.launch.py"

if [[ ! -f "${MAP_FILE}" ]]; then
  echo "ERROR: map file not found: ${MAP_FILE}" >&2
  exit 1
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
  echo "ERROR: nav2 params file not found: ${PARAMS_FILE}" >&2
  exit 1
fi

_map_workdir="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")" || exit 1
[[ -n "${_map_workdir}" ]] || {
  echo "ERROR: 地图临时目录为空（tj_nav2_map_yaml_ascii_workdir）" >&2
  exit 1
}
NAV2_MAP_FILE="${_map_workdir}/$(basename "${MAP_FILE}")"
tj_assert_nav2_static_map_ready "${NAV2_MAP_FILE}" || exit 1
NAV2_PARAMS_FILE="$(tj_nav2_params_yaml_ascii_copy "${PARAMS_FILE}")" || exit 1
[[ -f "${NAV2_PARAMS_FILE}" ]] || {
  echo "ERROR: Nav2 参数临时文件未生成: ${NAV2_PARAMS_FILE}" >&2
  exit 1
}

NAV2_LAUNCH_DELAY_SEC="${NAV2_LAUNCH_DELAY_SEC:-12}"

echo "TIP: /mnt/e 等挂载上 Gazebo spawn 可能较慢；改源码后需 bash scripts/build.sh 并 source install/setup.bash"
echo "Starting tb3_stack + Nav2: model=${TURTLEBOT3_MODEL}, mode=${TB3_STACK_MODE}, rgbd_bridge=${TB3_ASSIST_RGBD_BRIDGE}, yolo=${TB3_ASSIST_SCAN_FILTER}"

echo "Stopping any previous TB3 stack ..."
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" stop >/dev/null 2>&1 || true
sleep 1

export TB3_DEFER_RVIZ_FOR_MAP="${TB3_DEFER_RVIZ_FOR_MAP:-0}"
_TB3_WANT_RVIZ="${TB3_ENABLE_RVIZ:-1}"
if [[ "${TB3_DEFER_RVIZ_FOR_MAP}" == "1" && "${_TB3_WANT_RVIZ}" == "1" ]]; then
  export TB3_ENABLE_RVIZ=0
  _TJ_RVIZ_DEFER=1
else
  _TJ_RVIZ_DEFER=0
fi
bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start
if [[ "${_TJ_RVIZ_DEFER}" == "1" ]]; then
  export TB3_ENABLE_RVIZ="${_TB3_WANT_RVIZ}"
fi

echo "Stopping any previous Nav2 ..."
tj_kill_nav2_background_launch

if [[ "${NAV2_LAUNCH_DELAY_SEC}" =~ ^[0-9]+$ ]] && [[ "${NAV2_LAUNCH_DELAY_SEC}" -gt 0 ]]; then
  echo "Waiting ${NAV2_LAUNCH_DELAY_SEC}s before Nav2 (NAV2_LAUNCH_DELAY_SEC=0 to skip)"
  sleep "${NAV2_LAUNCH_DELAY_SEC}"
fi

echo "Starting Nav2 (ros2 launch ${NAV2_PKG} ${NAV2_LAUNCH_FILE}):"
echo "  MAP_FILE (源)     : ${MAP_FILE}"
echo "  map:=             : ${NAV2_MAP_FILE}"
echo "  params_file       : ${NAV2_PARAMS_FILE}"

ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
WS_SETUP_BASH="${PROJECT_ROOT}/ros_ws/install/setup.bash"
(
  set +u
  # shellcheck source=/dev/null
  source "${ROS_SETUP_BASH}"
  if [[ ! -f "${WS_SETUP_BASH}" ]]; then
    echo "ERROR: workspace not built, missing ${WS_SETUP_BASH}" >&2
    exit 1
  fi
  # shellcheck source=/dev/null
  source "${WS_SETUP_BASH}"
  set -u
  exec ros2 launch "${NAV2_PKG}" "${NAV2_LAUNCH_FILE}" \
    "use_sim_time:=true" \
    "map:=${NAV2_MAP_FILE}" \
    "params_file:=${NAV2_PARAMS_FILE}" \
    "use_composition:=False"
) >"${TB3_LOG_DIR}/nav2.launch.log" 2>&1 &
NAV2_LAUNCH_PID=$!
echo "Nav2 started in background (PID=${NAV2_LAUNCH_PID}). Logs: ${TB3_LOG_DIR}/nav2.launch.log"

if [[ "${_TJ_RVIZ_DEFER:-0}" == "1" ]]; then
  echo "RViz 将在 /map 有发布者后启动（TB3_DEFER_RVIZ_FOR_MAP=1）；最长等 ${TB3_DEFER_RVIZ_MAP_WAIT_SEC:-120}s"
  _rviz_cfg="${RVIZ_CONFIG_FILE:-${PROJECT_ROOT}/ros_ws/src/robot_bringup/config/test1.rviz}"
  tj_launch_rviz2_after_map_ready "${TB3_DEFER_RVIZ_MAP_WAIT_SEC:-120}" "${_rviz_cfg}" "${TB3_LOG_DIR}/rviz2.log"
fi

sleep 4
if ! kill -0 "${NAV2_LAUNCH_PID}" 2>/dev/null; then
  echo "ERROR: Nav2 进程已退出（请查看日志）:" >&2
  tail -80 "${TB3_LOG_DIR}/nav2.launch.log" >&2 || true
elif [[ "${TB3_MAP_CLI_CHECK:-0}" == "1" ]]; then
  (
    exec >>"${TB3_LOG_DIR}/nav2_map_cli_check.log" 2>&1
    set +u
    echo ""
    echo "======== nav2_map_cli_check $(date -Is 2>/dev/null || date) expect_nav2_pid=${NAV2_LAUNCH_PID} ========"
    # shellcheck source=/dev/null
    source "${ROS_SETUP_BASH}"
    if [[ ! -f "${WS_SETUP_BASH}" ]]; then
      echo "[tj_sim_nav2_stack] skip map check: no workspace setup"
      exit 0
    fi
    # shellcheck source=/dev/null
    source "${WS_SETUP_BASH}"
    set -u
    _map_ok=0
    _waited=0
    while ((_waited < 70)); do
      if ros2 topic info /map 2>/dev/null | grep -qiE 'publisher count:[[:space:]]*[1-9]'; then
        echo "[tj_sim_nav2_stack] /map 已有发布者（约 ${_waited}s）"
        _map_ok=1
        break
      fi
      sleep 3
      _waited=$((_waited + 3))
    done
    if [[ "${_map_ok}" != "1" ]]; then
      echo "[tj_sim_nav2_stack] WARN: 70s 内 ros2 topic info /map 无发布者"
      tail -80 "${TB3_LOG_DIR}/nav2.launch.log" 2>/dev/null || true
    fi
  ) &
  echo "[tj_sim_nav2_stack] TB3_MAP_CLI_CHECK=1: 见 ${TB3_LOG_DIR}/nav2_map_cli_check.log"
fi
