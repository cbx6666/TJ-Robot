#!/usr/bin/env bash
# 【可选】仅仿真+Nav2 底座（不含 LLM/任务）。日常请用: bash scripts/run_full_system.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/full_system_env.sh
source "${SCRIPT_DIR}/lib/full_system_env.sh"

full_system_resolve_nav2_map

echo "Full system base（可选分步）: 仅仿真 + Nav2。地图: ${NAV2_MAP_PATH_FILE}"
echo "  一键全栈请用: bash scripts/run_full_system.sh"
echo "  本脚本仅底座；LLM/任务: bash scripts/run_voice_stack.sh"
echo "  停止全套: bash scripts/kill_simulation_stack.sh（含语音+仿真；勿只 tb3_stack stop 否则麦可能被占死）"
echo "  纯激光建图: bash scripts/run_mapping.sh"
full_system_start_sim_and_nav2

echo ""
echo "Base stack 已在后台运行，本脚本即将退出（终端可继续使用）。"
full_system_echo_log_index
echo "  下一步: bash scripts/run_voice_stack.sh"
echo "  勿在 base 运行时裸测: python3 -c \"import sounddevice\"（易卡死无输出）；"
echo "  可选: bash scripts/probe_sounddevice.sh  或  bash scripts/check_mic_devices.sh"
