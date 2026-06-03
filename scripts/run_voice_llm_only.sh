#!/usr/bin/env bash
# 单独测：麦克风 ASR + LLM（无 Gazebo / Nav2 / task_manager）
# 全栈验证通过后再: bash scripts/run_full_system_real_mic.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

LOG_DIR="${TJ_VOICE_LLM_LOG_DIR:-${PROJECT_ROOT}/data/logs/voice_llm_only}"
mkdir -p "${LOG_DIR}"
HEALTH_LOG="${LOG_DIR}/voice_gateway_health.jsonl"

LLM_ENV_FILE="${PROJECT_ROOT}/local_llm.env"
if [[ -f "${LLM_ENV_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${LLM_ENV_FILE}"
  set +a
  echo "已加载 LLM: ${LLM_ENV_FILE}"
elif [[ -z "${TJ_LLM_API_KEY:-}" ]]; then
  echo "WARN: 未找到 local_llm.env 且未设置 TJ_LLM_API_KEY；LLM 可能无法调用。" >&2
  echo "      可复制: cp local_llm.env.example local_llm.env 并填入 Key" >&2
fi

if [[ "${TJ_VOICE_LLM_KILL_OLD:-1}" == "1" ]]; then
  pkill -f 'voice_gateway_node' 2>/dev/null || true
  pkill -f 'llm_router_node' 2>/dev/null || true
  pkill -f 'interaction.launch.py' 2>/dev/null || true
  sleep 0.5
fi

if pgrep -af 'run_full_system|task_pipeline.launch' >/dev/null 2>&1; then
  echo "WARN: 检测到全栈/语音栈可能在跑；WSL Pulse 通常只能稳定供一路录音。" >&2
  echo "      建议先 bash scripts/kill_simulation_stack.sh 或 Ctrl+C 停当前语音栈" >&2
fi

_mic_idx="-1"
if grep -qi microsoft /proc/version 2>/dev/null; then
  _mic_idx="${TJ_MIC_DEVICE_INDEX:-0}"
  if [[ -z "${PULSE_SERVER:-}" ]]; then
    echo "WARN: WSLg Pulse 未就绪；可 wsl --shutdown 后重开，并检查 Windows 麦克风隐私" >&2
  else
    echo "Pulse: ${PULSE_SERVER}  mic_device_index:=${_mic_idx}"
  fi
fi

echo "=========================================="
echo " 仅语音 + LLM（无仿真/导航/任务）"
echo "=========================================="
echo "  ASR: whisper_mic"
echo "  日志: 本终端 screen；健康 JSONL: ${HEALTH_LOG}"
echo ""
echo "  另开终端观察:"
echo "    ros2 topic echo /interaction/speech_text"
echo "    ros2 topic echo /interaction/parsed_intent"
echo "    tail -f ${HEALTH_LOG}"
echo ""
echo "  说完停顿 ~0.65s 才会切段；等终端出现「Whisper 模型就绪」再说话。"
echo "  全栈: bash scripts/run_full_system_real_mic.sh"
echo "=========================================="
echo ""

export PYTHONUNBUFFERED=1
exec ros2 launch robot_bringup interaction.launch.py \
  use_sim_time:=false \
  asr_backend:=whisper_mic \
  "mic_device_index:=${_mic_idx}" \
  "mic_health_log_path:=${HEALTH_LOG}" \
  "$@"
