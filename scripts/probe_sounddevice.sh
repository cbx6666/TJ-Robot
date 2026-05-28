#!/usr/bin/env bash
# 带超时的 sounddevice 探测（避免裸跑 python3 -c "import sounddevice" 在仿真/Pulse 僵死时一直无输出）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh" 2>/dev/null || true

SEC="${TJ_PROBE_SD_TIMEOUT_SEC:-8}"

echo "========== sounddevice 快速探测（超时 ${SEC}s）=========="
echo "PULSE_SERVER=${PULSE_SERVER:-<未设置>}"

if pgrep -af voice_gateway_node >/dev/null 2>&1; then
  echo "警告: voice_gateway 在运行，本探测可能卡住或枚举为空 — 请先停语音栈。"
fi
if pgrep -af 'gzserver|gzclient' >/dev/null 2>&1; then
  echo "提示: 仿真 GUI 在跑（如 run_full_system_base）。WSLg Pulse 常变慢或僵死；"
  echo "      无输出/超时属正常，请直接 bash scripts/run_voice_stack.sh，勿反复裸测 sounddevice。"
fi

if ! command -v timeout >/dev/null 2>&1; then
  echo "需要 timeout 命令: sudo apt-get install -y coreutils"
  exit 1
fi

set +e
out="$(timeout "${SEC}" python3 -u -c "
import os, sys
print('(1/3) 正在 import sounddevice …', flush=True)
import sounddevice as sd
print('(2/3) import 完成，枚举设备 …', flush=True)
devs = sd.query_devices()
print('(3/3) query_devices 完成', flush=True)
print(devs)
" 2>&1)"
rc=$?
set -e

if [[ "${rc}" -eq 0 ]]; then
  echo "${out}"
  echo ""
  echo "========== 结论: PortAudio 可用 =========="
  exit 0
fi

if [[ "${rc}" -eq 124 ]]; then
  echo "超时 (${SEC}s)：卡在 import 或 query_devices（多为 WSLg Pulse 僵死 / 仿真占满）。"
else
  echo "${out:-<无输出>}"
  echo "失败 rc=${rc}"
fi

echo ""
echo "建议:"
echo "  1) 不要停 base 反复测麦 — 直接: bash scripts/run_voice_stack.sh"
echo "  2) 恢复: bash scripts/recover_wsl_audio.sh"
echo "  3) 仍不行: Windows PowerShell → wsl --shutdown，重开 WSL"
exit 2
