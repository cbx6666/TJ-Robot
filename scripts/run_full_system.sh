#!/usr/bin/env bash
# 一键全栈：仿真 + Nav2 + YOLO + LLM/任务（默认 RViz「Sim Speech」模拟语音，无麦克风 ASR）。
# 高级：仅底座 TJ_FULL_SYSTEM_SPLIT=1 → run_full_system_base.sh；仅重开 LLM 栈 → run_voice_stack.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${TJ_FULL_SYSTEM_SPLIT:-}" == "1" ]]; then
  exec bash "${SCRIPT_DIR}/run_full_system_base.sh"
fi

# shellcheck source=lib/full_system_env.sh
source "${SCRIPT_DIR}/lib/full_system_env.sh"

export TJ_SIM_SPEECH_UI="${TJ_SIM_SPEECH_UI:-1}"

full_system_resolve_nav2_map

echo "=========================================="
echo " Full system（一键）"
echo "  仿真 + Nav2 + YOLO + LLM/任务/mock 抓放"
echo "=========================================="
if [[ "${TJ_SIM_SPEECH_UI}" == "1" ]]; then
  echo "  指令输入: RViz 面板「Sim Speech」→ 发送 → LLM"
  echo "  真麦模式: export TJ_SIM_SPEECH_UI=0 后重新运行本脚本"
else
  echo "  指令输入: 麦克风 ASR（voice_gateway）"
  if grep -qi microsoft /proc/version 2>/dev/null; then
    echo "  WSL 麦克风: PULSE_SERVER=${PULSE_SERVER:-<未检测到 wslg pulse>}"
  fi
fi
echo "  停全套: bash scripts/kill_simulation_stack.sh"
echo "  仅停 LLM/任务（保留仿真）: 本终端 Ctrl+C 后 bash scripts/run_voice_stack.sh"
echo ""

full_system_start_sim_and_nav2

_boot_wait="${TJ_FULL_SYSTEM_BOOT_WAIT_SEC:-}"
if [[ -z "${_boot_wait}" ]]; then
  if [[ "${TJ_SIM_SPEECH_UI}" == "1" ]]; then
    _boot_wait=25
  else
    _boot_wait=15
  fi
fi
if [[ "${_boot_wait}" =~ ^[0-9]+$ ]] && [[ "${_boot_wait}" -gt 0 ]]; then
  echo "等待 ${_boot_wait}s 供 Gazebo/RViz/Nav2 就绪（可调 TJ_FULL_SYSTEM_BOOT_WAIT_SEC）…"
  sleep "${_boot_wait}"
fi

echo ""
full_system_echo_log_index
echo ""
echo "启动 LLM/任务栈（前台；Ctrl+C 仅停本栈，仿真仍在后台）…"
full_system_launch_task_pipeline_foreground "$@"
