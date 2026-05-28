# shellcheck shell=bash
# WSL/WSLg：让 PortAudio/sounddevice 能见到 Windows 麦克风（无需每次手打，可写入 ~/.bashrc）
# 用法: source scripts/lib/wsl_pulse_env.sh   或 run_full_system.sh 已自动 source
#
# 注意: WSLg Pulse 往往同一时刻只够一个进程稳定录音；voice_gateway 已运行时勿在另一终端测麦。

_wsl_pulse_env_apply() {
  if ! grep -qi microsoft /proc/version 2>/dev/null; then
    return 0
  fi

  if ! command -v pactl >/dev/null 2>&1; then
    echo "[wsl_pulse_env] 未安装 pactl，不设置 PULSE_SERVER（避免连到僵尸套接字后 PortAudio 卡死）。" >&2
    echo "[wsl_pulse_env] 安装: sudo apt-get install -y pulseaudio-utils" >&2
    return 0
  fi

  _wsl_pulse_try() {
    local sock="$1"
    [[ -S "${sock}" || -e "${sock}" ]] || return 1
    PULSE_SERVER="unix:${sock}" pactl info >/dev/null 2>&1 || return 1
    export PULSE_SERVER="unix:${sock}"
    return 0
  }

  local cand
  for cand in \
    /mnt/wslg/runtime-dir/pulse/native \
    /mnt/wslg/PulseServer \
    /mnt/wslg/pulse/native; do
    _wsl_pulse_try "${cand}" && break
  done

  if [[ -z "${PULSE_SERVER:-}" && -n "${XDG_RUNTIME_DIR:-}" ]]; then
    _wsl_pulse_try "${XDG_RUNTIME_DIR}/pulse/native" || true
  fi

  if [[ -z "${PULSE_SERVER:-}" ]]; then
    echo "[wsl_pulse_env] 未找到可用的 Pulse（套接字在但 pactl 无响应时，可 wsl --shutdown 后重开）。" >&2
  fi

  export SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}"
}

_wsl_pulse_env_apply
