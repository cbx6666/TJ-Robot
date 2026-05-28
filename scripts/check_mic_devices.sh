#!/usr/bin/env bash
# 启动全链路前检查 WSL/本机是否有可用麦克风（PortAudio 枚举）
# full_system / voice_gateway 已占用 Pulse 时，本脚本可能卡住或显示无设备——请先停全链路再测。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_sd_timeout_sec="${TJ_CHECK_MIC_TIMEOUT_SEC:-10}"

echo "========== 环境 =========="
if grep -qi microsoft /proc/version 2>/dev/null; then
  echo "WSL: $(grep -i '^PRETTY_NAME' /etc/os-release 2>/dev/null | cut -d= -f2- | tr -d '\"' || echo unknown)"
  if [[ -d /mnt/wslg ]]; then
    echo "WSLg: 已挂载 /mnt/wslg"
    ls -la /mnt/wslg/runtime-dir/pulse/ 2>/dev/null | head -5 || true
  else
    echo "WSLg: 未找到 /mnt/wslg（纯 WSL 无 GUI 时通常无麦克风转发）"
  fi
fi

if pgrep -af voice_gateway_node >/dev/null 2>&1 || pgrep -af "robot_interaction.*voice_gateway" >/dev/null 2>&1; then
  echo ""
  echo "========== 警告 =========="
  echo "检测到 voice_gateway 正在运行（常见于 bash scripts/run_full_system.sh）。"
  echo "WSLg 的 Pulse 通常只能稳定供一个录音客户端使用；此时在另一终端："
  echo "  - pactl / sounddevice 可能卡住（需 Ctrl+C）"
  echo "  - 或枚举为「无输入设备」"
  echo "请先在同一 WSL 里停掉全链路（Ctrl+C full_system + tb3_stack stop），再运行本脚本。"
  echo "若必须边跑全链路边测麦，只能看 full_system 终端里 voice_gateway 的日志，不要另开检测。"
  echo ""
elif pgrep -af 'gzserver|gzclient' >/dev/null 2>&1; then
  echo ""
  echo "========== 警告 =========="
  echo "检测到 Gazebo 仿真在跑（常见于 bash scripts/run_full_system_base.sh）。"
  echo "此时裸跑 python3 -c \"import sounddevice\" 常会长时间无输出或 Ctrl+C 后直接回到提示符。"
  echo "本脚本已带 ${_sd_timeout_sec}s 超时；更短探测: bash scripts/probe_sounddevice.sh"
  echo "推荐: base 稳定后直接 bash scripts/run_voice_stack.sh，不必先单独测 sounddevice。"
  echo ""
fi

_run_pactl_probe() {
  local label="$1"
  echo ""
  echo "========== pactl: ${label} =========="
  echo "PULSE_SERVER=${PULSE_SERVER:-<未设置>}"
  if ! command -v pactl >/dev/null 2>&1; then
    echo "未安装 pactl: sudo apt-get install -y pulseaudio-utils"
    return 1
  fi
  if [[ -z "${PULSE_SERVER:-}" ]]; then
    echo "跳过（未设置 PULSE_SERVER）"
    return 1
  fi
  if timeout 3 pactl info 2>/dev/null | grep -E 'Server Name|Default Source'; then
    echo "--- sources ---"
    timeout 3 pactl list sources short 2>/dev/null || echo "pactl list sources 超时/失败"
    return 0
  fi
  echo "pactl info 失败或超时（Pulse 未响应或被其它进程占满）"
  return 1
}

_run_sounddevice_probe() {
  local label="$1"
  echo ""
  echo "========== sounddevice: ${label}（超时 ${_sd_timeout_sec}s）=========="
  if ! command -v timeout >/dev/null 2>&1; then
    echo "需要 coreutils 的 timeout 命令"
    return 1
  fi
  if PULSE_SERVER="${PULSE_SERVER:-}" timeout "${_sd_timeout_sec}" python3 - "$label" <<'PY'
import os
import sys

label = sys.argv[1]
print(f"(PULSE_SERVER={os.environ.get('PULSE_SERVER', '<未设置>')})")
try:
    import sounddevice as sd
except ImportError:
    print("未安装 sounddevice: pip install -r ros_ws/src/robot_interaction/requirements-voice.txt")
    sys.exit(1)

print("全部 PortAudio 设备（含仅输出）:")
any_dev = False
for i, d in enumerate(sd.query_devices()):
    inch = int(d.get("max_input_channels", 0) or 0)
    outch = int(d.get("max_output_channels", 0) or 0)
    if inch > 0 or outch > 0:
        any_dev = True
        print(f"  [{i}] in={inch} out={outch} | {d.get('name', '')}")
if not any_dev:
    print("  （无任何 in/out 通道的设备 — PortAudio 可能连不上 Pulse）")

devs = []
for i, d in enumerate(sd.query_devices()):
    ch = int(d.get("max_input_channels", 0) or 0)
    if ch > 0:
        devs.append((i, d.get("name", ""), ch))
if not devs:
    print()
    print("结论: 无输入设备（max_input_channels>0）。")
    sys.exit(2)
print()
print("可用麦克风:")
for i, name, ch in devs:
    print(f"  [{i}] {name} (in={ch})")
if any("pulse" in (n or "").lower() for _, n, _ in devs):
    print()
    print("WSL: 常见 [0]=pulse（推荐），[1]=default（full_system 下易 Timeout）。")
    print("  全链路: bash scripts/run_full_system.sh  （已默认 mic_device_index:=0）")
try:
    d = sd.query_devices(kind="input")
    print(f"默认输入: {d.get('name', d)}")
except Exception as e:
    print(f"默认输入: 无 ({e})")
sys.exit(0)
PY
  then
    return 0
  fi
  echo "sounddevice 探测超时或失败（${label}）"
  return 1
}

# 1) 不强制 PULSE_SERVER（与部分「单独 interaction 能用的」终端一致）
unset PULSE_SERVER
_run_pactl_probe "未设置 PULSE_SERVER" || true
if _run_sounddevice_probe "未设置 PULSE_SERVER"; then
  echo ""
  echo "========== 总结 =========="
  echo "在未设置 PULSE_SERVER 时已找到麦克风；若 full_system 仍失败，多为 Pulse 被占用或应用了错误 index。"
  exit 0
fi

# 2) 尝试 WSLg Pulse（需 pactl 可用且服务未被打满）
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh"
if _run_pactl_probe "WSLg Pulse"; then
  _run_sounddevice_probe "WSLg Pulse" && exit 0
else
  echo ""
  echo "WSLg Pulse 不可用，清除 PULSE_SERVER 避免 PortAudio 挂死…"
  unset PULSE_SERVER
fi

echo ""
echo "========== 结论 =========="
echo "两种路径均未找到可用麦克风。常见原因:"
echo "  1) voice_gateway / full_system 仍在运行（占满 Pulse）→ 先停再测"
echo "  2) Windows 未允许: 设置 -> 隐私 -> 麦克风 ->「适用于 Linux 的 Windows 子系统」"
echo "  3) 需重启 WSL: PowerShell 执行  wsl --shutdown  后重开 Cursor/WSL"
echo "  4) 安装: sudo apt-get install -y pulseaudio-utils portaudio19-dev"
echo ""
echo "无麦联调语音/LLM:"
echo "  ros2 launch robot_bringup interaction.launch.py use_sim_time:=false \\"
echo "    asr_backend:=mock enable_mock_voice:=true"
echo "  或 asr_backend:=whisper_file + 向 /interaction/transcribe_wav_path 发 wav 路径"
exit 2
