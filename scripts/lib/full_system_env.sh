# shellcheck shell=bash
# run_full_system / run_full_system_base / run_voice_stack 共用环境
# 用法: source scripts/lib/full_system_env.sh

: "${SCRIPT_DIR:?SCRIPT_DIR must be set before sourcing full_system_env.sh}"

# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh"

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
export TURTLEBOT3_MODEL="waffle"
export TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
export TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
export TB3_ENABLE_SLAM=0
export TB3_LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/full_system}"
# 减少 Nav2 清理时的控制台刷屏（仍可在 nav2*.log 中查看）
export TB3_QUIET_NAV2_CLEANUP="${TB3_QUIET_NAV2_CLEANUP:-1}"
# shellcheck source=lib/tb3_sim_assist_env.sh
source "${SCRIPT_DIR}/lib/tb3_sim_assist_env.sh"

MAP_FILE="${MAP_FILE:-${ROS_WS}/src/robot_bringup/maps/map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-${ROS_WS}/src/robot_navigation/config/nav2_params.yaml}"
NAV_LAUNCH_FILE="${ROS_WS}/src/robot_navigation/launch/navigation.launch.py"
NAV2_MAP_PATH_FILE="${TB3_LOG_DIR}/nav2_map_yaml.path"
FULL_SYSTEM_NAV2_LOG="${TB3_LOG_DIR}/nav2.launch.log"
FULL_SYSTEM_NAV2_DEFERRED_LOG="${TB3_LOG_DIR}/nav2_deferred_navigation.log"
FULL_SYSTEM_TASK_PIPELINE_LOG="${TB3_LOG_DIR}/task_pipeline.launch.log"

full_system_echo_log_index() {
  echo "日志目录: ${TB3_LOG_DIR}"
  echo "  仿真栈: gzserver.log, gzclient.log, rviz2.log, yolo_object_seg.log, rgbd_to_scan.log, spawn_entity.log, …"
  echo "  导航:   nav2.launch.log, nav2_deferred_navigation.log"
  echo "  语音/任务/操控: task_pipeline.launch.log"
  echo "  语音健康 JSONL: voice_gateway_health.jsonl（故障快照 voice_health_incidents/）"
  echo "  跟踪语音: tail -f ${FULL_SYSTEM_TASK_PIPELINE_LOG}"
  echo "  复现 voice+base 故障: 第三终端 bash scripts/watch_voice_health.sh"
  echo "  跟踪导航: tail -f ${FULL_SYSTEM_NAV2_LOG}"
  echo "  列出全部: bash scripts/tb3_stack.sh logs all"
}

full_system_launch_task_pipeline_foreground() {
  local -a extra_launch=()
  if [[ "${TJ_SIM_SPEECH_UI:-0}" == "1" ]]; then
    if [[ "${*:-}" != *asr_backend* ]]; then
      extra_launch+=("asr_backend:=none")
    fi
    if [[ "${*:-}" != *enable_sim_speech_gui* ]]; then
      extra_launch+=("enable_sim_speech_gui:=false")
    fi
  elif [[ "${*:-}" != *asr_backend* ]]; then
    extra_launch+=("asr_backend:=${TJ_FULL_SYSTEM_ASR:-whisper_mic}")
  fi
  if [[ "${*:-}" != *mic_speech_rms_threshold* ]]; then
    extra_launch+=("mic_speech_rms_threshold:=${TJ_MIC_SPEECH_RMS_THRESHOLD:-0.02}")
  fi
  if [[ "${*:-}" != *mic_min_speech_sec* ]]; then
    extra_launch+=("mic_min_speech_sec:=${TJ_MIC_MIN_SPEECH_SEC:-0.38}")
  fi
  export TJ_VOICE_HEALTH_LOG="${TJ_VOICE_HEALTH_LOG:-${TB3_LOG_DIR}/voice_gateway_health.jsonl}"
  if [[ "${*:-}" != *mic_health_log_path* ]]; then
    extra_launch+=("mic_health_log_path:=${TJ_VOICE_HEALTH_LOG}")
  fi
  mkdir -p "${TB3_LOG_DIR}"
  local _session_ts
  _session_ts="$(date -Iseconds 2>/dev/null || date)"
  {
    echo "===== task_pipeline ${_session_ts} ====="
    echo "  map_yaml=${NAV2_MAP_FILE}"
    echo "  args: use_sim_time:=true ${extra_launch[*]} $*"
  } >"${FULL_SYSTEM_TASK_PIPELINE_LOG}"
  echo "语音/任务栈 -> ${FULL_SYSTEM_TASK_PIPELINE_LOG}"
  echo "  本次 session 已清空日志，起始: ${_session_ts}"
  echo "  （节点输出只写文件、不刷本终端；Cursor 里打开 .log 不会自动刷新）"
  echo "  请在 WSL 另开/watch: tail -f ${FULL_SYSTEM_TASK_PIPELINE_LOG}"
  if [[ "${TJ_SIM_SPEECH_UI:-0}" == "1" ]]; then
    echo "  模拟语音（默认）: RViz 左侧/底部面板「Sim Speech」选句后点「发送 → LLM」"
    echo "  未看到面板: colcon build --packages-select robot_rviz_plugins 后重启 RViz"
    echo "  备用窗口: … run_voice_stack.sh enable_sim_speech_gui:=true"
    echo "  恢复真麦: export TJ_SIM_SPEECH_UI=0 后再 run_voice_stack.sh"
  else
    echo "  提示: 日志里出现「Whisper 模型就绪」后再说话；说完停顿约 0.65s 才会「切段」"
    echo "  麦克风门限: rms>=${TJ_MIC_SPEECH_RMS_THRESHOLD:-0.02}（仍误触可 export TJ_MIC_SPEECH_RMS_THRESHOLD=0.03）"
  fi
  # 重定向到文件时避免 ROS/Python 块缓冲，否则 tail -f 像“很久才蹦一行”
  export PYTHONUNBUFFERED=1
  export RCUTILS_LOGGING_USE_STDOUT=1
  export RCUTILS_LOGGING_BUFFERED_STREAM=0
  local _launch=(ros2 launch robot_bringup task_pipeline.launch.py)
  if command -v stdbuf >/dev/null 2>&1; then
    _launch=(stdbuf -oL -eL "${_launch[@]}")
  fi
  exec "${_launch[@]}" \
    "use_sim_time:=true" \
    "map_yaml:=${NAV2_MAP_FILE}" \
    "${extra_launch[@]}" "$@" >>"${FULL_SYSTEM_TASK_PIPELINE_LOG}" 2>&1
}

full_system_resolve_nav2_map() {
  if [[ ! -f "${MAP_FILE}" ]]; then
    echo "ERROR: map file not found: ${MAP_FILE}" >&2
    return 1
  fi
  if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "ERROR: nav2 params file not found: ${PARAMS_FILE}" >&2
    return 1
  fi
  if [[ ! -f "${NAV_LAUNCH_FILE}" ]]; then
    echo "ERROR: navigation launch file not found: ${NAV_LAUNCH_FILE}" >&2
    return 1
  fi
  NAV2_MAP_FILE="$(tj_nav2_map_yaml_ascii_workdir "${MAP_FILE}")/$(basename "${MAP_FILE}")"
  mkdir -p "${TB3_LOG_DIR}"
  printf '%s\n' "${NAV2_MAP_FILE}" >"${NAV2_MAP_PATH_FILE}"
  export NAV2_MAP_FILE
}

full_system_load_nav2_map_from_file() {
  if [[ -f "${NAV2_MAP_PATH_FILE}" ]]; then
    NAV2_MAP_FILE="$(tr -d '\r' <"${NAV2_MAP_PATH_FILE}")"
    if [[ -f "${NAV2_MAP_FILE}" ]]; then
      export NAV2_MAP_FILE
      return 0
    fi
    echo "WARN: stale map path in ${NAV2_MAP_PATH_FILE}, re-resolving…" >&2
  fi
  full_system_resolve_nav2_map
}

full_system_start_sim_and_nav2() {
  if pgrep -af voice_gateway_node >/dev/null 2>&1 \
    || pgrep -af 'task_pipeline\.launch\.py' >/dev/null 2>&1; then
    export TB3_SKIP_DAEMON_STOP=1
    echo "WARN: 检测到语音栈已在运行；tb3_stack 将跳过 ros2 daemon stop（避免语音节点失联）。"
    echo "      日常请用 bash scripts/run_full_system.sh 一键起栈；若语音异常请 Ctrl+C 后 bash scripts/run_voice_stack.sh"
  fi
  echo "Starting RGBD simulation + YOLO (YOLO_IMAGE_TOPIC=${YOLO_IMAGE_TOPIC}, unified=${TB3_SIM_UNIFIED_RGBD})."
  bash "${PROJECT_ROOT}/scripts/tb3_stack.sh" start

  echo "Starting Nav2 (日志 -> ${FULL_SYSTEM_NAV2_LOG}，控制台仅启动摘要)"
  mkdir -p "${TB3_LOG_DIR}"
  {
    echo "===== Nav2 launch $(date -Iseconds 2>/dev/null || date) ====="
    echo "  map:=${NAV2_MAP_FILE}"
    echo "  params_file:=${PARAMS_FILE}"
  } >"${FULL_SYSTEM_NAV2_LOG}"
  local ros_setup="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
  local ws_setup="${PROJECT_ROOT}/ros_ws/install/setup.bash"
  (
    set +u
    # shellcheck source=/dev/null
    source "${ros_setup}"
    if [[ -f "${ws_setup}" ]]; then
      # shellcheck source=/dev/null
      source "${ws_setup}"
    else
      echo "ERROR: workspace not built, missing ${ws_setup}" >&2
      exit 1
    fi
    set -u
    exec ros2 launch "${NAV_LAUNCH_FILE}" \
      "use_sim_time:=true" \
      "map:=${NAV2_MAP_FILE}" \
      "params_file:=${PARAMS_FILE}" \
      "defer_navigation_autostart:=true"
  ) >>"${FULL_SYSTEM_NAV2_LOG}" 2>&1 &
  echo "Nav2 started in background (PID=$!). tail -f ${FULL_SYSTEM_NAV2_LOG}"
  {
    echo "===== Nav2 deferred navigation STARTUP $(date -Iseconds 2>/dev/null || date) ====="
  } >"${FULL_SYSTEM_NAV2_DEFERRED_LOG}"
  (
    set +e
    tj_nav2_trigger_navigation_manager_startup_after_map_server
  ) >>"${FULL_SYSTEM_NAV2_DEFERRED_LOG}" 2>&1 &
  echo "Deferred Nav2 lifecycle helper -> ${FULL_SYSTEM_NAV2_DEFERRED_LOG}"
}

full_system_stop_voice_stack() {
  echo "Stopping voice/LLM/task pipeline (Gazebo/Nav2 不受影响)…"
  pkill -f 'task_pipeline\.launch\.py' 2>/dev/null || true
  pkill -f 'interaction\.launch\.py' 2>/dev/null || true
  pkill -f 'voice_gateway_node' 2>/dev/null || true
  pkill -f 'sim_speech_gui_node' 2>/dev/null || true
  pkill -f 'llm_router_node' 2>/dev/null || true
  pkill -f 'task_manager_node' 2>/dev/null || true
  pkill -f 'command_executor_node' 2>/dev/null || true
  pkill -f 'patrol_waypoints' 2>/dev/null || true
  sleep 1
  if pgrep -af voice_gateway_node >/dev/null 2>&1; then
    echo "WARN: voice_gateway 仍在，强杀以免占用 WSL 麦克风…"
    pkill -9 -f voice_gateway_node 2>/dev/null || true
    sleep 0.5
  fi
}
