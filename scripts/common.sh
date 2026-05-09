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

# 结束前一次后台拉的 Nav2（run_simulation 每次都会再起一条 ros2 launch，不杀会叠多套 /amcl）。
tj_kill_nav2_background_launch() {
  echo "tj_kill_nav2_background_launch: 结束残留的 Nav2 launch ..."
  pkill -f 'navigation\.launch\.py' 2>/dev/null || true
  pkill -f 'tj_static_map_nav2\.launch\.py' 2>/dev/null || true
  sleep 1
}

# 配合 navigation.launch.py 的 defer_navigation_autostart:=true：等 map_server 进入 active 后，对
# lifecycle_manager_navigation 调用 manage_nodes(STARTUP)。避免 localization 与 navigation 同时
# autostart 时 map_server configure 卡住，导致永远没有 map TF。
tj_nav2_trigger_navigation_manager_startup_after_map_server() {
  local max_map_wait="${NAV2_MAP_SERVER_ACTIVE_WAIT_SEC:-120}"
  local ros_setup="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
  local ws_setup="${ROS_WS}/install/setup.bash"
  if [[ ! -f "${ros_setup}" ]]; then
    echo "tj_nav2_trigger_navigation_manager_startup_after_map_server: missing ${ros_setup}" >&2
    return 1
  fi
  safe_source "${ros_setup}"
  if [[ ! -f "${ws_setup}" ]]; then
    echo "tj_nav2_trigger_navigation_manager_startup_after_map_server: missing ${ws_setup}" >&2
    return 1
  fi
  safe_source "${ws_setup}"

  local i=0
  while (( i < max_map_wait )); do
    if ros2 lifecycle get /map_server 2>/dev/null | grep -qF "State: active"; then
      break
    fi
    sleep 1
    i=$((i + 1))
  done
  if (( i >= max_map_wait )); then
    echo "tj_nav2_trigger_navigation_manager_startup_after_map_server: timeout waiting for map_server active" >&2
    return 1
  fi

  i=0
  while (( i < 120 )); do
    if ros2 service list 2>/dev/null | grep -Fxq "/lifecycle_manager_navigation/manage_nodes"; then
      if ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"; then
        echo "tj_nav2_trigger_navigation_manager_startup_after_map_server: navigation STARTUP ok"
        return 0
      fi
    fi
    sleep 0.5
    i=$((i + 1))
  done
  echo "tj_nav2_trigger_navigation_manager_startup_after_map_server: failed to call manage_nodes STARTUP" >&2
  return 1
}
