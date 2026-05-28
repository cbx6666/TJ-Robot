#!/usr/bin/env bash
# 复现「先 voice 后 base → 过一会不收音」：第三终端运行本脚本，与 voice/base 并行。
# 不另开 sounddevice（避免抢 Pulse）；读 voice_gateway 写入的 JSONL + 系统进程状态。
#
# 用法:
#   终端1: bash scripts/run_voice_stack.sh
#   终端2: bash scripts/run_full_system_base.sh
#   终端3: bash scripts/watch_voice_health.sh
#
# 日志:
#   data/logs/full_system/voice_health_watch.log
#   data/logs/full_system/voice_health_incidents/  （故障时由 voice_gateway 或本脚本写入）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

LOG_DIR="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/full_system}"
HEALTH_JSONL="${TJ_VOICE_HEALTH_LOG:-${LOG_DIR}/voice_gateway_health.jsonl}"
WATCH_LOG="${LOG_DIR}/voice_health_watch.log"
INC_DIR="${LOG_DIR}/voice_health_incidents"
INTERVAL="${TJ_WATCH_VOICE_INTERVAL_SEC:-5}"
STALL_WARN="${TJ_WATCH_VOICE_STALL_SEC:-4}"

mkdir -p "${LOG_DIR}" "${INC_DIR}"

_prev_state="unknown"
_last_segment_count=""

echo "watch_voice_health: 每 ${INTERVAL}s 采样 → ${WATCH_LOG}"
echo "  读节点 JSONL: ${HEALTH_JSONL}"
echo "  voice 运行时不要在本脚本里 import sounddevice（会抢 WSL Pulse）"
echo "  Ctrl+C 结束"
echo ""

_snapshot_incident() {
  local reason="$1"
  local stamp
  stamp="$(date +%Y%m%d_%H%M%S)"
  local path="${INC_DIR}/watch_${stamp}.txt"
  {
    echo "===== watch_voice_health incident $(date -Iseconds) ====="
    echo "reason: ${reason}"
    echo ""
    echo "===== last watch lines ====="
    tail -20 "${WATCH_LOG}" 2>/dev/null || true
    echo ""
    echo "===== last voice_gateway health jsonl ====="
    tail -5 "${HEALTH_JSONL}" 2>/dev/null || true
    echo ""
    echo "===== pgrep ====="
    pgrep -af 'voice_gateway|task_pipeline|gzserver|gzclient|yolo_object|whisper' 2>/dev/null || true
    echo ""
    echo "===== pactl (3s) ====="
    if command -v pactl >/dev/null 2>&1; then
      timeout 3 pactl info 2>&1 || echo "pactl timeout/fail"
    fi
    echo ""
    echo "===== task_pipeline 末 15 行（切段/health/stall）====="
    grep -E '切段|voice_health|STALL|溢出|麦克风线程' "${LOG_DIR}/task_pipeline.launch.log" 2>/dev/null | tail -15 || true
  } >"${path}"
  echo "  → 已写快照: ${path}"
}

_read_last_health() {
  if [[ ! -f "${HEALTH_JSONL}" ]]; then
    echo ""
    return
  fi
  tail -1 "${HEALTH_JSONL}" 2>/dev/null || true
}

while true; do
  ts="$(date -Iseconds 2>/dev/null || date)"
  vg=0
  pgrep -af voice_gateway_node >/dev/null 2>&1 && vg=1
  gz=$(pgrep -c gzserver 2>/dev/null || echo 0)
  yolo=$(pgrep -c yolo_object 2>/dev/null || echo 0)
  nav=$(pgrep -c 'navigation\.launch' 2>/dev/null || echo 0)

  health_json="$(_read_last_health)"
  stall="-"
  peak="-"
  segs="-"
  pulse_ok="-"
  stream_open="-"
  json_age="-"

  if [[ -n "${health_json}" ]]; then
    eval "$(python3 -c "
import json, sys, os, time
try:
    o=json.loads(sys.argv[1])
except Exception:
    print('stall=-;peak=-;segs=-;pulse_ok=-;stream_open=-')
    raise SystemExit(0)
print('stall=%s' % o.get('read_stall_sec', '-'))
print('peak=%s' % o.get('peak_rms', '-'))
print('segs=%s' % o.get('segment_count', '-'))
print('pulse_ok=%s' % o.get('pulse_ok', '-'))
print('stream_open=%s' % o.get('stream_open', '-'))
" "${health_json}" 2>/dev/null)" || true
    if [[ -f "${HEALTH_JSONL}" ]]; then
      json_age="$(python3 -c "import os,time; print(int(time.time()-os.path.getmtime('${HEALTH_JSONL}')))" 2>/dev/null || echo -)"
    fi
  fi

  # 外部 pactl：仅当 voice 未运行时才探测（避免第二客户端）
  pactl_ext="skip"
  if [[ "${vg}" -eq 0 ]] && command -v pactl >/dev/null 2>&1; then
    if timeout 2 pactl info >/dev/null 2>&1; then pactl_ext="ok"; else pactl_ext="fail"; fi
  fi

  line="${ts} vg=${vg} gz=${gz} yolo=${yolo} nav=${nav} stream=${stream_open} stall_s=${stall} peak_rms=${peak} segs=${segs} pulse_ok=${pulse_ok} json_age_s=${json_age} pactl_ext=${pactl_ext}"

  state="ok"
  reason=""
  if [[ "${vg}" -eq 0 ]]; then
    state="no_voice"
  elif [[ "${json_age}" != "-" && "${json_age}" -ge 30 ]]; then
    state="stale_json"
    reason="voice_gateway 健康 JSONL ${json_age}s 未更新（节点可能卡死）"
  elif [[ "${stream_open}" == "True" && "${stall}" != "-" ]] \
    && python3 -c "exit(0 if float('${stall}')>=float('${STALL_WARN}') else 1)" 2>/dev/null; then
    state="mic_stall"
    reason="stream 已开但 read 停滞 ${stall}s（>=${STALL_WARN}s）"
  fi
  if [[ "${vg}" -eq 1 && "${pulse_ok}" == "False" ]]; then
    state="pulse_dead"
    reason="节点报告 pulse_ok=False"
  fi

  line="${line} state=${state}"
  echo "${line}" | tee -a "${WATCH_LOG}"

  if [[ "${state}" != "ok" && "${state}" != "no_voice" && "${state}" != "${_prev_state}" ]]; then
    echo "  *** 状态变化: ${_prev_state} → ${state}: ${reason}"
    _snapshot_incident "${reason}"
  fi
  if [[ "${state}" == "mic_stall" || "${state}" == "pulse_dead" || "${state}" == "stale_json" ]]; then
    if [[ "${_prev_state}" == "${state}" ]]; then
      : # 持续故障不重复刷 incident
    fi
  fi

  _prev_state="${state}"
  sleep "${INTERVAL}"
done
