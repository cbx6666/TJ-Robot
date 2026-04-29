#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_WS="${PROJECT_ROOT}/ros_ws"
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  local file="$1"
  local had_nounset=0

  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # shellcheck source=/dev/null
  source "${file}"

  if (( had_nounset )); then
    set -u
  fi
}

require_ros() {
  if [[ ! -f "${ROS_SETUP_BASH}" ]]; then
    echo "ERROR: ROS setup not found: ${ROS_SETUP_BASH}" >&2
    echo "Install ROS 2 Humble or set ROS_SETUP_BASH=/path/to/setup.bash" >&2
    exit 1
  fi
  safe_source "${ROS_SETUP_BASH}"
}

source_workspace_if_available() {
  local setup_file="${ROS_WS}/install/setup.bash"
  if [[ -f "${setup_file}" ]]; then
    safe_source "${setup_file}"
  else
    echo "WARNING: workspace is not built yet: ${setup_file}" >&2
    echo "Run: bash scripts/build.sh" >&2
  fi
}

prepare_output_dirs() {
  mkdir -p \
    "${PROJECT_ROOT}/data/logs" \
    "${PROJECT_ROOT}/data/results"
}

require_command() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: required command not found: ${cmd}" >&2
    exit 1
  fi
}
