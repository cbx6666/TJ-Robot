#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# 导航等 launch：navigation.launch.py 若未显式传入 params_file，会优先读取该工作区源码里的 nav2_params.yaml
ROS_WS="${PROJECT_ROOT}/ros_ws"
export ROS_WS
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
    "${PROJECT_ROOT}/data/logs/simulation" \
    "${PROJECT_ROOT}/data/results"
}

require_command() {
  local cmd="$1"
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "ERROR: required command not found: ${cmd}" >&2
    exit 1
  fi
}

# ROS 2 / Nav2 launch 将 map:= 传入 map_server 时，若路径含非 ASCII（如中文目录名），可能被错误编码成
# /mnt/e/study/u673Au5668... 这类不可用路径，导致 map_server configure 失败、/map 无发布者。
# 将地图目录整份拷到 /tmp 下仅限 ASCII 的路径即可规避。
tj_nav2_map_yaml_ascii_workdir() {
  local src_yaml="$1"
  local maps_dir
  maps_dir="$(cd "$(dirname "${src_yaml}")" && pwd)"
  local dest_dir
  dest_dir="$(mktemp -d /tmp/tj_nav2_map.XXXXXX)"
  cp -a "${maps_dir}/." "${dest_dir}/"
  echo "${dest_dir}"
}
