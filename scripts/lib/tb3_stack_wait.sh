# shellcheck shell=bash
# 由 scripts/tb3_stack.sh source；依赖: ros2 CLI、SECONDS。

wait_for_service() {
  local name="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 service type "${name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

wait_for_topic() {
  local name="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 topic info "${name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

format_duration() {
  local total_s="$1"
  printf "%02dm%02ds" "$((total_s / 60))" "$((total_s % 60))"
}

step_begin() {
  local idx="$1"
  local total="$2"
  local title="$3"
  echo "[${idx}/${total}] ${title}"
}

step_end() {
  local title="$1"
  local step_t0="$2"
  local stack_t0="$3"
  local step_elapsed=$((SECONDS - step_t0))
  local total_elapsed=$((SECONDS - stack_t0))
  echo "  -> ${title} | step=$(format_duration "${step_elapsed}") total=$(format_duration "${total_elapsed}")"
}

wait_topic_with_timing() {
  local topic="$1"
  local timeout_s="${2:-20}"
  local stack_t0="$3"
  local t0=$SECONDS
  if wait_for_topic "${topic}" "${timeout_s}"; then
    local elapsed=$((SECONDS - t0))
    local total_elapsed=$((SECONDS - stack_t0))
    echo "  -> topic ready ${topic} | wait=$(format_duration "${elapsed}") total=$(format_duration "${total_elapsed}")"
    return 0
  fi
  local elapsed=$((SECONDS - t0))
  local total_elapsed=$((SECONDS - stack_t0))
  echo "  -> topic timeout ${topic} (${timeout_s}s) | wait=$(format_duration "${elapsed}") total=$(format_duration "${total_elapsed}")"
  return 1
}
