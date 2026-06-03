#!/usr/bin/env bash
# 一键全栈 + 真麦 ASR（无 RViz Sim Speech）。等价于 TJ_SIM_SPEECH_UI=0 run_full_system.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export TJ_SIM_SPEECH_UI=0

echo "=========================================="
echo " Full system — 真麦 (whisper_mic)"
echo "=========================================="
if grep -qi microsoft /proc/version 2>/dev/null; then
  # shellcheck source=lib/wsl_pulse_env.sh
  source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh"
  if [[ -z "${PULSE_SERVER:-}" ]]; then
    echo "WARN: 未连上 WSLg Pulse；请检查 WSLg 与 Windows 麦克风隐私，必要时 wsl --shutdown" >&2
  else
    echo "  Pulse: ${PULSE_SERVER}"
  fi
  echo "  Windows: 设置 → 隐私 → 麦克风 → 允许「适用于 Linux 的 Windows 子系统」"
fi
echo "  改回模拟语音: bash scripts/run_full_system.sh"
echo "  启动: 仿真与 Whisper 并行预热，GUI 就绪且模型就绪后可直接说话"
echo "  关闭并行预启: export TJ_FULL_SYSTEM_EARLY_VOICE=0"
echo ""

exec bash "${SCRIPT_DIR}/run_full_system.sh" "$@"
