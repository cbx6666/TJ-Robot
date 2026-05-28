#!/usr/bin/env bash
# 【可选】仅重开 LLM/任务栈（要求仿真+Nav2 已在跑）。一键全栈: bash scripts/run_full_system.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 与 run_full_system 一致：默认 RViz 模拟语音，不启 ASR
export TJ_SIM_SPEECH_UI="${TJ_SIM_SPEECH_UI:-1}"
# shellcheck source=lib/full_system_env.sh
source "${SCRIPT_DIR}/lib/full_system_env.sh"

if [[ "${TJ_VOICE_STACK_KILL_OLD:-1}" == "1" ]]; then
  full_system_stop_voice_stack
fi

full_system_load_nav2_map_from_file

echo "Voice/LLM stack only（仿真应已由 run_full_system 或 run_full_system_base 拉起）"
if [[ "${TJ_SIM_SPEECH_UI}" == "1" ]]; then
  echo "  模拟语音: RViz「Sim Speech」面板 → /interaction/speech_text → LLM"
  echo "  真麦: export TJ_SIM_SPEECH_UI=0  （ASR=\${TJ_FULL_SYSTEM_ASR:-whisper_mic}）"
else
  echo "  ASR: \${TJ_FULL_SYSTEM_ASR:-whisper_mic}；LLM 见 TJ_LLM_* / local_llm.env"
  if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "  WSL 麦克风: PULSE_SERVER=${PULSE_SERVER:-<未设置>}"
    echo "  测麦请先停语音栈，再: bash scripts/check_mic_devices.sh"
  fi
fi
echo "  仅停语音: 本终端 Ctrl+C；改代码后重新 bash scripts/run_voice_stack.sh"
echo "  停仿真: bash scripts/tb3_stack.sh stop"
echo "  日常一键: bash scripts/run_full_system.sh（无需先 base 再 voice）"
echo "  诊断: 第三终端 bash scripts/watch_voice_health.sh（抓 base 起后何时 mic_stall）"
echo "  健康: tail -f data/logs/full_system/voice_gateway_health.jsonl"
full_system_launch_task_pipeline_foreground "$@"
