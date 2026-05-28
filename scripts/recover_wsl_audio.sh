#!/usr/bin/env bash
# WSL：sounddevice 卡在 Pa_Initialize() / pactl 无响应时的恢复脚本。
# 若 pkill voice + unset PULSE_SERVER 仍超时，只能重启 WSL 会话（与关掉所有 WSL 窗口 / 重启 Cursor 等效）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_sd_timeout="${TJ_RECOVER_AUDIO_TIMEOUT_SEC:-5}"

echo "========== 0) 说明 =========="
echo "本脚本只能清理本发行版内的进程；WSLg 的 Pulse 僵死时需重启 WSL。"
echo "  轻量: PowerShell → wsl --terminate \$(wsl -l -q | head -1)"
echo "  彻底: PowerShell(管理员) → wsl --shutdown"
echo "  等效于你平时的做法: 关掉所有 WSL 终端 + 重启 Cursor"
echo ""

echo "========== 1) 结束可能占用麦克风的进程 =========="
for pat in voice_gateway_node task_pipeline.launch.py interaction.launch.py \
  llm_router_node command_executor_node patrol_waypoints; do
  if pgrep -af "${pat}" >/dev/null 2>&1; then
    echo "  pkill -9: ${pat}"
    pkill -9 -f "${pat}" 2>/dev/null || true
  fi
done
# 仿真栈有时也拖慢 Pulse 探测（不杀则跳过）
if [[ "${TJ_RECOVER_AUDIO_KILL_SIM:-0}" == "1" ]]; then
  bash "${SCRIPT_DIR}/kill_simulation_stack.sh" 2>/dev/null || true
fi
sleep 1
if pgrep -af voice_gateway_node >/dev/null 2>&1; then
  echo "  ERROR: voice_gateway 仍在，请手动: pkill -9 -f voice_gateway_node"
else
  echo "  voice_gateway: 无残留"
fi

_probe_sd() {
  local label="$1"
  shift
  echo ""
  echo "--- sounddevice (${label}, ${_sd_timeout}s 超时) ---"
  local out rc
  set +e
  out="$(timeout "${_sd_timeout}" env "$@" python3 -c "
import sounddevice as sd
print(sd.query_devices())
" 2>&1)"
  rc=$?
  set -e
  if [[ "${rc}" -eq 0 && -n "${out}" ]]; then
    echo "${out}"
    return 0
  fi
  if [[ "${rc}" -eq 124 ]]; then
    echo "  超时 (${_sd_timeout}s)：PortAudio/Pulse 无响应（rc=124）"
  else
    echo "  失败 rc=${rc}: ${out:-<无输出>}"
  fi
  return 1
}

_probe_pactl() {
  local label="$1"
  shift
  if ! command -v pactl >/dev/null 2>&1; then
    echo "  pactl 未安装，跳过 (${label})"
    return 1
  fi
  echo ""
  echo "--- pactl (${label}, 3s 超时) ---"
  if timeout 3 env "$@" pactl info 2>/dev/null | head -3; then
    timeout 3 env "$@" pactl list sources short 2>/dev/null | head -5 || true
    return 0
  fi
  echo "  pactl 超时或失败 (${label})"
  return 1
}

echo ""
echo "========== 2) 探测（先不连 WSLg Pulse 套接字）=========="
_sd_ok=0
_pactl_ok=0
if _probe_sd "无 PULSE_SERVER" PULSE_SERVER=; then _sd_ok=1; fi
if _probe_pactl "无 PULSE_SERVER" PULSE_SERVER=; then _pactl_ok=1; fi

echo ""
echo "========== 3) 探测（WSLg Pulse 套接字）=========="
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh" 2>/dev/null || true
echo "PULSE_SERVER=${PULSE_SERVER:-<未设置>}"
if _probe_sd "当前 PULSE_SERVER"; then _sd_ok=1; fi
if _probe_pactl "当前 PULSE_SERVER"; then _pactl_ok=1; fi

echo ""
if [[ "${_sd_ok}" -eq 1 ]]; then
  echo "========== 结论: PortAudio 已恢复，可 bash scripts/run_voice_stack.sh =========="
  exit 0
fi

echo "========== 结论: 本发行版内无法恢复 Pulse（与你的现象一致）=========="
echo ""
echo "  你已做的 pkill / unset PULSE_SERVER 仍卡住 → 不是 ROS 节点没杀干净，"
echo "  而是 WSLg 侧 Pulse 服务僵死，Linux 里无法 pulseaudio -k 修复。"
echo ""
echo "  请任选一种（与「关光所有 WSL 窗口 + 重启 Cursor」等效）："
echo ""
echo "  [推荐] Windows PowerShell:"
echo "    wsl --shutdown"
echo "  然后重新打开 Cursor / WSL 终端。"
echo ""
echo "  或仅结束当前发行版（有时够用）:"
echo "    wsl --terminate <你的发行版名>    # 例: wsl -l -v 查看名称"
echo ""
echo "  恢复后请先测:"
echo "    bash scripts/check_mic_devices.sh"
echo "  再: bash scripts/run_full_system_base.sh → run_voice_stack.sh"
echo ""
echo "  预防: 停实验用 bash scripts/kill_simulation_stack.sh（含语音），不要只 tb3_stack stop"
exit 2
