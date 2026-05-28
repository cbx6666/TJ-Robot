#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP_BASH}" ]]; then
  echo "ERROR: 未找到 ROS 2 环境脚本: ${ROS_SETUP_BASH}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${ROS_SETUP_BASH}"
set +u

ROS_WS_SETUP="${SCRIPT_DIR}/../ros_ws/install/setup.bash"
if [[ -f "${ROS_WS_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${ROS_WS_SETUP}"
fi

# common.sh 会打开 nounset；tb3_stack 大量用 ${VAR:-} 可选环境变量，保持关闭 -u。
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
set +u

TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
TB3_ENABLE_SLAM="${TB3_ENABLE_SLAM:-1}"
TB3_LOG_DIR="${TB3_LOG_DIR:-/tmp/tb3_stack}"
TB3_ENABLE_GZCLIENT="${TB3_ENABLE_GZCLIENT:-1}"
TB3_ENABLE_RVIZ="${TB3_ENABLE_RVIZ:-1}"
# 兼容旧开关：TB3_NO_GUI=1 时同时关闭 gzclient 与 rviz2。
if [[ "${TB3_NO_GUI:-0}" == "1" ]]; then
  TB3_ENABLE_GZCLIENT="0"
  TB3_ENABLE_RVIZ="0"
fi
# Gazebo 相机渲染频率（Hz）。默认 2 以减轻 WSL/软件渲染负载。
# 需要更高视觉帧率时可临时设为 10/30。
TB3_CAMERA_UPDATE_RATE="${TB3_CAMERA_UPDATE_RATE:-2}"
# 是否启用相机传感器（1=启用, 0=禁用）。默认：laser 模式关，assist 模式开。
if [[ -z "${TB3_ENABLE_CAMERA:-}" ]]; then
  if [[ "${TB3_STACK_MODE}" == "laser" ]]; then
    TB3_ENABLE_CAMERA="0"
  else
    TB3_ENABLE_CAMERA="1"
  fi
fi
# 相机是否 always_on。默认 0：有订阅者时才渲染，降低空载消耗。
TB3_CAMERA_ALWAYS_ON="${TB3_CAMERA_ALWAYS_ON:-0}"

WORLD_FILE="${WORLD_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/world/small_house.world}"
MODEL_FILE="${MODEL_FILE:-/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf}"
URDF_FILE="${URDF_FILE:-/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_${TURTLEBOT3_MODEL}.urdf}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/test1.rviz}"
if [[ "${TB3_SIM_UNIFIED_RGBD:-0}" == "1" ]]; then
  # 一体 depth 插件彩色话题为 /camera/image_raw（非 /camera/rgb/*）
  YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
  YOLO_CAMERA_INFO_TOPIC="${YOLO_CAMERA_INFO_TOPIC:-/camera/camera_info}"
  RGBD_DEPTH_IMAGE_TOPIC="${RGBD_DEPTH_IMAGE_TOPIC:-/camera/depth/image_raw}"
  RGBD_DEPTH_CAMERA_INFO_TOPIC="${RGBD_DEPTH_CAMERA_INFO_TOPIC:-/camera/depth/camera_info}"
else
  YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
  YOLO_CAMERA_INFO_TOPIC="${YOLO_CAMERA_INFO_TOPIC:-/camera/camera_info}"
  RGBD_DEPTH_IMAGE_TOPIC="${RGBD_DEPTH_IMAGE_TOPIC:-/tb3_depth_only/depth/image_raw}"
  RGBD_DEPTH_CAMERA_INFO_TOPIC="${RGBD_DEPTH_CAMERA_INFO_TOPIC:-/tb3_depth_only/depth/camera_info}"
fi
YOLO_DEVICE="${YOLO_DEVICE:-auto}"
# COCO 类别 ID 逗号分隔；all 表示不限制类别（计算更重）
# COCO: 39 bottle, 41 cup, 75 vase（仿真可乐罐多标为 bottle/cup）
YOLO_TARGET_CLASS_IDS="${YOLO_TARGET_CLASS_IDS:-39,41,75}"
YOLO_DEPTH_TOPIC="${YOLO_DEPTH_TOPIC:-${RGBD_DEPTH_IMAGE_TOPIC}}"
YOLO_DEPTH_CAMERA_INFO_TOPIC="${YOLO_DEPTH_CAMERA_INFO_TOPIC:-${RGBD_DEPTH_CAMERA_INFO_TOPIC}}"
# 1=启动 depth_image_proc RegisterNode 组件，深度对齐 RGB（需 ros-humble-depth-image-proc）
# 默认 0：仿真下 RegisterNode 常输出全 NaN（TF/同步），3D 点会全部丢失
# 需要注册深度时：export TB3_YOLO_DEPTH_REGISTER=1（并确认 depth_registered 有有效像素）
if [[ -z "${TB3_YOLO_DEPTH_REGISTER:-}" ]]; then
  TB3_YOLO_DEPTH_REGISTER="0"
else
  TB3_YOLO_DEPTH_REGISTER="${TB3_YOLO_DEPTH_REGISTER}"
fi
TB3_SLAM_PARAMS_FILE="${TB3_SLAM_PARAMS_FILE:-}"
ROBOT_START_X="${ROBOT_START_X:-0.0}"
ROBOT_START_Y="${ROBOT_START_Y:-0.0}"
ROBOT_START_Z="${ROBOT_START_Z:-0.1}"
ROBOT_START_YAW="${ROBOT_START_YAW:-0.0}"
TB3_ENABLE_RSP="${TB3_ENABLE_RSP:-1}"

mkdir -p "${TB3_LOG_DIR}"

cleanup_old() {
  # Nav2（navigation / tj_static_map_nav2）；与 scripts/kill_nav2.sh、common.sh 一致
  tj_kill_nav2_background_launch
  # run_simulation / run_full_system 后台 lifecycle 脚本（会反复 ros2 service call，停栈后仍占 CPU/DDS）
  pkill -9 -f tj_nav2_trigger_navigation_manager_startup_after_map_server 2>/dev/null || true
  pkill -9 -f nav2_deferred_navigation 2>/dev/null || true
  pkill -9 -f "ros2 launch robot_navigation" 2>/dev/null || true
  pkill -9 -f component_container 2>/dev/null || true
  pkill -9 -f spawn_entity.py 2>/dev/null || true
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f "ros2 launch human_yolo_seg" 2>/dev/null || true
  pkill -9 -f "ros2 launch robot_bringup rgbd_to_scan.launch.py" 2>/dev/null || true
  pkill -9 -f "ros2 launch robot_bringup slam_laser.launch.py" 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f publish_robot_description_topic.py 2>/dev/null || true
  pkill -9 -f '/robot_description std_msgs/msg/String' 2>/dev/null || true
  pkill -9 -f yolo_detector_node 2>/dev/null || true
  pkill -9 -f yolo_object_seg_node 2>/dev/null || true
  pkill -9 -f yolo_person_seg_node 2>/dev/null || true
  pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
  pkill -9 -f slam_toolbox 2>/dev/null || true
  pkill -9 -f point_cloud_xyz_node 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null || true
  pkill -9 -f depth_image_to_viz.py 2>/dev/null || true
  pkill -9 -f scan_rviz_relay 2>/dev/null || true
  # 卡住的 ros2 CLI / daemon 会导致终端「输入无响应」
  # 分阶段启动时若 voice 已在另一终端运行，daemon stop 会扰乱其 DDS/定时器（表现为不再切段）
  if [[ "${TB3_SKIP_DAEMON_STOP:-0}" != "1" ]]; then
    timeout 3 ros2 daemon stop 2>/dev/null || true
  fi
}

wait_for_service() {
  local name="$1"
  local timeout_s="${2:-20}"
  local progress_label="${3:-}"
  # 避免 TB3_GZSERVER_WAIT_SEC=0 等导致零次循环、立刻误判失败
  if [[ "${timeout_s}" =~ ^[0-9]+$ ]] && (( timeout_s < 5 )); then
    timeout_s=5
  fi
  local t0=$SECONDS
  local deadline=$((SECONDS + timeout_s))
  local next_progress=$((t0 + 15))
  while (( SECONDS < deadline )); do
    if ros2 service type "${name}" >/dev/null 2>&1; then
      return 0
    fi
    if [[ -n "${progress_label}" && SECONDS -ge next_progress ]]; then
      echo "  -> ${progress_label}（已等 $(format_duration "$((SECONDS - t0))") / 最长 ${timeout_s}s）" >&2
      next_progress=$((SECONDS + 15))
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

publish_robot_description() {
  local urdf_file="$1"
  local pub_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/publish_robot_description_topic.py"
  if [[ ! -f "${pub_py}" ]]; then
    echo "ERROR: 未找到 ${pub_py}" >&2
    exit 1
  fi
  setsid python3 "${pub_py}" "${urdf_file}" --hz "${TB3_ROBOT_DESCRIPTION_TOPIC_HZ:-2}" \
    >"${TB3_LOG_DIR}/robot_description.log" 2>&1 < /dev/null &
}

tj_apply_urdf_camera_mount() {
  local urdf_file="$1"
  [[ -f "${urdf_file}" ]] || return 0
  [[ "${TB3_CAMERA_MOUNT_OFFICIAL:-0}" == "1" ]] && return 0
  export TB3_BRINGUP_SCRIPTS="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts"
  python3 - "${urdf_file}" <<'PYURDFZ' 2>>"${TB3_LOG_DIR}/urdf_camera_mount.log" || true
import os
import sys
from pathlib import Path
sys.path.insert(0, os.environ.get("TB3_BRINGUP_SCRIPTS", ""))
from tb3_camera_mount import patch_urdf_camera_mount, camera_rgb_z_m
p = Path(sys.argv[1])
text, n = patch_urdf_camera_mount(p.read_text(encoding="utf-8"))
if n:
    p.write_text(text, encoding="utf-8")
    print(
        f"URDF camera mount Z={camera_rgb_z_m():.3f} m (height+depth_on_rgb_frame) patches={n}",
        file=sys.stderr,
    )
PYURDFZ
}

expand_robot_urdf() {
  local input_file="$1"
  local output_file="$2"
  local frag="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/urdf/waffle_depth_camera_links.urdf.xml"
  local xraw="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_xacro_raw.urdf"
  local need_depth_links=0
  if [[ "${TURTLEBOT3_MODEL}" == "waffle" || "${TURTLEBOT3_MODEL}" == "waffle_pi" ]]; then
    need_depth_links=1
  fi
  if [[ -f "${output_file}" && "${output_file}" -nt "${input_file}" ]]; then
    if [[ "${need_depth_links}" != "1" || ! -f "${frag}" ]]; then
      tj_apply_urdf_camera_mount "${output_file}"
      return 0
    fi
    if [[ "${output_file}" -nt "${frag}" ]]; then
      tj_apply_urdf_camera_mount "${output_file}"
      return 0
    fi
  fi
  if ! ros2 run xacro xacro "${input_file}" >"${xraw}" 2>"${TB3_LOG_DIR}/xacro_tb3.log"; then
    echo "ERROR: xacro 展开失败: ${input_file}" >&2
    echo "       详情见: ${TB3_LOG_DIR}/xacro_tb3.log" >&2
    return 1
  fi
  if [[ "${need_depth_links}" == "1" && -f "${frag}" ]]; then
    export TB3_BRINGUP_SCRIPTS="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts"
    if ! python3 - "${xraw}" "${frag}" "${output_file}" <<'PYAPPEND'
import re
import sys
from pathlib import Path

xacro_out = Path(sys.argv[1])
frag_path = Path(sys.argv[2])
out_path = Path(sys.argv[3])
base = xacro_out.read_text(encoding="utf-8")
tok = "</robot>"
if tok not in base:
    print("ERROR: xacro URDF 缺少 </robot>", file=sys.stderr)
    raise SystemExit(1)


def _already_has_camera_depth_mount(urdf: str) -> bool:
    # Humble turtlebot3_description/waffle 常为 RealSense 树，已包含 camera_depth_*。
    # 旧版正则在部分 joint 排版下会漏检，导致与 fragment 二次拼装、link 重名。
    if re.search(r'<link\s+name\s*=\s*"camera_depth_frame"', urdf):
        return True
    if re.search(r"<link\s+name\s*=\s*'camera_depth_frame'", urdf):
        return True
    return False


frag = frag_path.read_text(encoding="utf-8").strip()
if _already_has_camera_depth_mount(base):
    print(
        "NOTE: URDF already has camera_depth_frame; skipping waffle_depth_camera_links fragment",
        file=sys.stderr,
    )
    frag = ""

if frag:
    head, _sep, _tail = base.rpartition(tok)
    merged = head.rstrip() + "\n" + frag + "\n" + tok + "\n"
else:
    merged = base

import os
_scripts = os.environ.get("TB3_BRINGUP_SCRIPTS", "").strip()
if _scripts and os.environ.get("TB3_CAMERA_MOUNT_OFFICIAL", "") != "1":
    import sys as _sys
    _sys.path.insert(0, _scripts)
    from tb3_camera_mount import patch_urdf_camera_mount, camera_rgb_z_m
    merged, _n = patch_urdf_camera_mount(merged)
    if _n:
        print(
            f"NOTE: URDF camera mount Z={camera_rgb_z_m():.3f} m; depth_joint->camera_rgb_frame (patches={_n})",
            file=sys.stderr,
        )

out_path.write_text(merged, encoding="utf-8")
PYAPPEND
    then
      echo "ERROR: 拼接 depth_camera_links URDF 失败" >&2
      return 1
    fi
  else
    cp -f "${xraw}" "${output_file}"
  fi
  tj_apply_urdf_camera_mount "${output_file}"
}

tb3_patch_gazebo_camera_sensors() {
  local input_file="$1"
  local output_file="$2"
  local camera_rate="$3"
  local enable_camera="$4"
  local camera_always_on="$5"
  local kinds="${6:-camera,depth}"
  local patch_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/patch_tb3_gazebo_camera_sensors.py"
  if [[ ! -f "${patch_py}" ]]; then
    echo "WARNING: missing ${patch_py}" >&2
    return 1
  fi
  python3 "${patch_py}" \
    --input "${input_file}" \
    --output "${output_file}" \
    --rate "${camera_rate}" \
    --enable "${enable_camera}" \
    --always-on "${camera_always_on}" \
    --rgb-width "${TB3_SIM_CAMERA_WIDTH:-640}" \
    --rgb-height "${TB3_SIM_CAMERA_HEIGHT:-480}" \
    --depth-width "${TB3_SIM_DEPTH_WIDTH:-640}" \
    --depth-height "${TB3_SIM_DEPTH_HEIGHT:-480}" \
    --kinds "${kinds}"
}

prepare_model_with_camera_rate() {
  tb3_patch_gazebo_camera_sensors "$1" "$2" "$3" "$4" "$5" "camera"
}

do_start() {
  local stack_t0=$SECONDS
  local step_t0

  cleanup_old

  export TURTLEBOT3_MODEL
  # 默认离线，避免网络/代理导致 gzserver 卡在远程模型拉取。
  export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

  # 保留系统默认，并补回用户/项目模型目录，确保 small_house.world 的
  # model://aws_robomaker_residential_* 能被解析。
  local _model_paths="${GAZEBO_MODEL_PATH:-}"
  for _p in \
    "/usr/share/gazebo-11/models" \
    "/usr/share/gazebo/models" \
    "/opt/ros/humble/share/turtlebot3_gazebo/models" \
    "/opt/ros/humble/share/turtlebot3_description" \
    "${SCRIPT_DIR}/../ros_ws/src/robot_bringup/models" \
    "${HOME}/.gazebo/models"
  do
    if [[ -d "${_p}" ]]; then
      _model_paths="${_model_paths:+${_model_paths}:}${_p}"
    fi
  done
  export GAZEBO_MODEL_PATH="${_model_paths}"

  # WSL 无 PulseAudio 时 OpenAL 探测会阻塞数秒；dummy 可避免拖慢 gzserver 加载世界
  export SDL_AUDIODRIVER="${SDL_AUDIODRIVER:-dummy}"

  step_t0=$SECONDS
  step_begin "1" "8" "Start gzserver"
  setsid gzserver "${WORLD_FILE}" --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >"${TB3_LOG_DIR}/gzserver.log" 2>&1 < /dev/null &
  step_end "gzserver launched (service wait deferred)" "${step_t0}" "${stack_t0}"

  step_t0=$SECONDS
  if [[ "${TB3_ENABLE_RSP}" == "1" ]]; then
    step_begin "2" "8" "Start robot_state_publisher"
    if [[ ! -f "${URDF_FILE}" ]]; then
      echo "ERROR: 未找到 URDF_FILE=${URDF_FILE}" >&2
      exit 1
    fi
    local expanded_urdf="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_expanded.urdf"
    if ! expand_robot_urdf "${URDF_FILE}" "${expanded_urdf}"; then
      exit 1
    fi
    setsid ros2 run robot_state_publisher robot_state_publisher \
      "${expanded_urdf}" \
      --ros-args -p use_sim_time:=true \
      >"${TB3_LOG_DIR}/robot_state_publisher.log" 2>&1 < /dev/null &
    publish_robot_description "${expanded_urdf}"
    step_end "robot_state_publisher started" "${step_t0}" "${stack_t0}"
  else
    step_begin "2" "8" "Start robot_state_publisher"
    echo "Skip robot_state_publisher (TB3_ENABLE_RSP=0)"
    step_end "robot_state_publisher skipped" "${step_t0}" "${stack_t0}"
  fi

  step_t0=$SECONDS
  step_begin "3" "8" "Spawn robot model"
  local wait_spawn_t0=$SECONDS
  local _gz_wait="${TB3_GZSERVER_WAIT_SEC:-90}"
  [[ "${_gz_wait}" =~ ^[0-9]+$ ]] || _gz_wait=90
  ((_gz_wait < 15)) && _gz_wait=15
  if ! wait_for_service "/spawn_entity" "${_gz_wait}" "等待 gzserver 加载世界并暴露 /spawn_entity"; then
    echo "ERROR: gzserver 未在 ${_gz_wait}s 内暴露 ROS 2 服务 /spawn_entity。" >&2
    echo "  常见原因: (1) gzserver 仍在加载 small_house 等大地图 (2) WSL/机械盘过慢 (3) 残留 gzserver 占坑。" >&2
    echo "  建议: 查看 tail -50 \"${TB3_LOG_DIR}/gzserver.log\"；尝试 bash scripts/tb3_stack.sh stop 后重试；" >&2
    echo "  或加大等待: export TB3_GZSERVER_WAIT_SEC=120" >&2
    if [[ -f "${TB3_LOG_DIR}/gzserver.log" ]]; then
      echo "----- gzserver.log (last 24 lines) -----" >&2
      tail -n 24 "${TB3_LOG_DIR}/gzserver.log" >&2 || true
      echo "----------------------------------------" >&2
    fi
    exit 1
  fi
  echo "  -> /spawn_entity ready | wait=$(format_duration "$((SECONDS - wait_spawn_t0))")"
  local spawn_model_file="${MODEL_FILE}"
  local model_prep_t0=$SECONDS
  if [[ "${TB3_ENABLE_CAMERA}" == "0" ]]; then
    # 建图模式下生成并复用“禁用相机”的 SDF，避免每次都走深度相机链路。
    local patched_model="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_camera_disabled.sdf"
    if [[ ! -f "${patched_model}" || "${MODEL_FILE}" -nt "${patched_model}" ]]; then
      if prepare_model_with_camera_rate "${MODEL_FILE}" "${patched_model}" "0" "0" "0" \
        >"${TB3_LOG_DIR}/camera_rate_patch.log" 2>&1; then
        spawn_model_file="${patched_model}"
      else
        echo "Using original model file: ${MODEL_FILE}"
      fi
    else
      spawn_model_file="${patched_model}"
    fi
    echo "Using no-camera mode: enable=0 (${spawn_model_file})"
  elif [[ -n "${TB3_CAMERA_UPDATE_RATE}" ]]; then
    local _sim_w="${TB3_SIM_CAMERA_WIDTH:-640}"
    local _sim_h="${TB3_SIM_CAMERA_HEIGHT:-480}"
    local patched_model="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_sim_${TB3_CAMERA_UPDATE_RATE}hz_${_sim_w}x${_sim_h}.sdf"
    if [[ ! -f "${patched_model}" || "${MODEL_FILE}" -nt "${patched_model}" ]]; then
      if prepare_model_with_camera_rate "${MODEL_FILE}" "${patched_model}" "${TB3_CAMERA_UPDATE_RATE}" "${TB3_ENABLE_CAMERA}" "${TB3_CAMERA_ALWAYS_ON}" \
        >"${TB3_LOG_DIR}/camera_rate_patch.log" 2>&1; then
        spawn_model_file="${patched_model}"
      else
        echo "Using original model file: ${MODEL_FILE}"
      fi
    else
      spawn_model_file="${patched_model}"
    fi
    echo "Using camera mode: enable=${TB3_ENABLE_CAMERA}, always_on=${TB3_CAMERA_ALWAYS_ON}, update_rate=${TB3_CAMERA_UPDATE_RATE} Hz, rgb=${_sim_w}x${_sim_h} (${spawn_model_file})"
  fi
  # 官方 waffle 仅 RGB；assist 配置一体 RGB-D 或旧版双相机 tb3_depth_only。
  if [[ "${TB3_ENABLE_CAMERA}" == "1" ]] &&
    { [[ "${TURTLEBOT3_MODEL}" == "waffle" ]] || [[ "${TURTLEBOT3_MODEL}" == "waffle_pi" ]]; }; then
    local assist_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/prepare_assist_waffle_sdf.py"
    if [[ -f "${assist_py}" ]]; then
      local _dw="${TB3_SIM_DEPTH_WIDTH:-640}"
      local _dh="${TB3_SIM_DEPTH_HEIGHT:-480}"
      local _unified="${TB3_SIM_UNIFIED_RGBD:-0}"
      local _assist_args=()
      local _sensor_kinds="camera,depth"
      local _out_sdf
      if [[ "${_unified}" == "1" ]]; then
        _assist_args+=(--unified-rgbd)
        _sensor_kinds="depth"
        _out_sdf="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_rgbd_${TB3_CAMERA_UPDATE_RATE}hz_${_dw}x${_dh}.sdf"
      else
        _out_sdf="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_depth_${TB3_CAMERA_UPDATE_RATE}hz_${_dw}x${_dh}.sdf"
      fi
      if python3 "${assist_py}" "${spawn_model_file}" "${_out_sdf}" "${_assist_args[@]}" \
        >"${TB3_LOG_DIR}/assist_waffle_depth_sdf.log" 2>&1; then
        if tb3_patch_gazebo_camera_sensors "${_out_sdf}" "${_out_sdf}" \
          "${TB3_CAMERA_UPDATE_RATE}" "${TB3_ENABLE_CAMERA}" "${TB3_CAMERA_ALWAYS_ON}" "${_sensor_kinds}" \
          >>"${TB3_LOG_DIR}/camera_rate_patch.log" 2>&1; then
          spawn_model_file="${_out_sdf}"
          if [[ "${_unified}" == "1" ]]; then
            echo "  -> unified RGB-D @ ${TB3_CAMERA_UPDATE_RATE}Hz ${_dw}x${_dh} -> $(basename "${_out_sdf}") (/camera/image_raw + /camera/depth/*)"
          else
            echo "  -> injected tb3_depth_only @ ${TB3_CAMERA_UPDATE_RATE}Hz ${_dw}x${_dh} -> $(basename "${_out_sdf}")"
          fi
        else
          echo "WARNING: sensor patch failed; using assist output as-is (see camera_rate_patch.log)" >&2
          spawn_model_file="${_out_sdf}"
        fi
      else
        echo "WARNING: prepare_assist_waffle_sdf failed (see ${TB3_LOG_DIR}/assist_waffle_depth_sdf.log)" >&2
      fi
    else
      echo "WARNING: missing ${assist_py}; cannot configure RGB-D" >&2
    fi
  fi
  # 官方 waffle 相机碰撞盒常与 LDS 扫掠高度重叠 → 正前方扇形盲区；测距默认 0.01m 高斯噪声会带来“抖动”观感。
  if [[ "${TB3_PATCH_SPAWN_SDF_EXTRAS:-1}" == "1" ]] && [[ -f "${spawn_model_file}" ]]; then
    local patch_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/patch_tb3_spawn_sdf_assist.py"
    if [[ -f "${patch_py}" ]]; then
      local _lstd="${TB3_SIM_LASER_RANGE_STDDEV:-0}"
      local _strip="${TB3_STRIP_CAMERA_RGB_COLLISION:-1}"
      local _mount_args=()
      if [[ "${TB3_CAMERA_MOUNT_OFFICIAL:-0}" == "1" ]]; then
        _mount_args+=(--no-camera-mount-raise)
      fi
      if [[ -n "${TB3_CAMERA_RGB_Z_M:-}" ]]; then
        _mount_args+=(--camera-rgb-z "${TB3_CAMERA_RGB_Z_M}")
      fi
      if [[ "${_strip}" == "1" ]]; then
        if python3 "${patch_py}" "${spawn_model_file}" --laser-noise-stddev "${_lstd}" --strip-camera-rgb-collisions \
          "${_mount_args[@]}" \
          >"${TB3_LOG_DIR}/patch_spawn_sdf_assist.log" 2>&1; then
          echo "  -> patched spawn SDF (rgb link pose Z≈${TB3_CAMERA_RGB_Z_M:-0.89} m, 关节保持官方, strip collisions, laser stddev=${_lstd})"
        else
          echo "WARNING: patch_tb3_spawn_sdf_assist failed (see ${TB3_LOG_DIR}/patch_spawn_sdf_assist.log)" >&2
        fi
      else
        if python3 "${patch_py}" "${spawn_model_file}" --laser-noise-stddev "${_lstd}" \
          "${_mount_args[@]}" \
          >"${TB3_LOG_DIR}/patch_spawn_sdf_assist.log" 2>&1; then
          echo "  -> patched spawn SDF (camera Z≈${TB3_CAMERA_RGB_Z_M:-0.89} m, laser stddev=${_lstd})"
        else
          echo "WARNING: patch_tb3_spawn_sdf_assist failed (see ${TB3_LOG_DIR}/patch_spawn_sdf_assist.log)" >&2
        fi
      fi
    fi
  fi
  echo "  -> model preprocess done | took=$(format_duration "$((SECONDS - model_prep_t0))")"
  # spawn_entity.py 内部会再次等待 /spawn_entity（默认 30s）。预处理较重时，偶发「bash 侧已看到服务、
  # 到本进程启动时服务又未就绪」或 DDS 发现滞后；先小睡并二次确认，再带重试执行 spawn。
  sleep 2
  if ! wait_for_service "/spawn_entity" 45; then
    echo "ERROR: 预处理结束后 /spawn_entity 仍不可用，请查看 ${TB3_LOG_DIR}/gzserver.log" >&2
    exit 1
  fi
  local spawn_t0=$SECONDS
  local spawn_attempt=1
  local spawn_max=3
  while (( spawn_attempt <= spawn_max )); do
    if ros2 run gazebo_ros spawn_entity.py \
      -timeout 90.0 \
      -entity "${TURTLEBOT3_MODEL}" \
      -file "${spawn_model_file}" \
      -x "${ROBOT_START_X}" -y "${ROBOT_START_Y}" -z "${ROBOT_START_Z}" -Y "${ROBOT_START_YAW}" \
      >"${TB3_LOG_DIR}/spawn_entity.log" 2>&1
    then
      break
    fi
    if (( spawn_attempt < spawn_max )); then
      echo "WARNING: spawn_entity 第 ${spawn_attempt}/${spawn_max} 次失败，3s 后重试（见 ${TB3_LOG_DIR}/spawn_entity.log）" >&2
      sleep 3
      wait_for_service "/spawn_entity" 30 || true
    else
      echo "ERROR: gazebo_ros spawn_entity.py 失败（已重试 ${spawn_max} 次）。常见: gzserver 异常、实体名冲突、WSL/DDS 滞后。" >&2
      echo "  完整日志: ${TB3_LOG_DIR}/spawn_entity.log" >&2
      if [[ -f "${TB3_LOG_DIR}/spawn_entity.log" ]]; then
        echo "----- spawn_entity.log (last 40 lines) -----" >&2
        tail -n 40 "${TB3_LOG_DIR}/spawn_entity.log" >&2 || true
        echo "--------------------------------------------" >&2
      fi
      echo "  建议: bash scripts/tb3_stack.sh stop；可试 ros2 daemon stop 后再启动。" >&2
      exit 1
    fi
    spawn_attempt=$((spawn_attempt + 1))
  done
  echo "  -> spawn_entity done | took=$(format_duration "$((SECONDS - spawn_t0))")"
  step_end "robot model spawned" "${step_t0}" "${stack_t0}"

  step_t0=$SECONDS
  step_begin "4" "8" "Start YOLO"
  local yolo_enabled=0
  if [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" && "${TB3_ENABLE_CAMERA}" == "1" ]]; then
    yolo_enabled=1
    _yolo_launch_args=(
      use_sim_time:=true
      image_topic:="${YOLO_IMAGE_TOPIC}"
      camera_info_topic:="${YOLO_CAMERA_INFO_TOPIC}"
      device:="${YOLO_DEVICE}"
      target_class_ids_csv:="${YOLO_TARGET_CLASS_IDS}"
      depth_sample_stat:="${TB3_YOLO_DEPTH_SAMPLE_STAT:-min}"
      depth_range_to_optical_z:="${TB3_YOLO_DEPTH_RANGE_TO_OPTICAL_Z:-false}"
    )
    if [[ "${TB3_YOLO_DEPTH_REGISTER}" == "1" ]]; then
      _yolo_launch_args+=(enable_depth_register:=true)
    else
      _yolo_launch_args+=(
        depth_topic:="${YOLO_DEPTH_TOPIC}"
        depth_camera_info_topic:="${YOLO_DEPTH_CAMERA_INFO_TOPIC}"
      )
    fi
    if [[ "${TB3_SIM_UNIFIED_RGBD:-0}" == "1" ]]; then
      # 仿真深度帧率不稳：不用 ApproxSync 卡死 annotated；RGB 直订 + 最新深度
      _yolo_launch_args+=(
        depth_pixels_aligned:=true
        use_depth_sync:=false
        max_depth_age_sec:=2.0
      )
    fi
    setsid ros2 launch human_yolo_seg yolo_object_seg.launch.py \
      "${_yolo_launch_args[@]}" \
      >"${TB3_LOG_DIR}/yolo_object_seg.log" 2>&1 < /dev/null &
  else
    if [[ "${TB3_ASSIST_SCAN_FILTER}" != "1" ]]; then
      echo "Skip YOLO detection (TB3_ASSIST_SCAN_FILTER=0)"
    else
      echo "Skip YOLO detection (camera disabled: TB3_ENABLE_CAMERA=0)"
    fi
  fi
  step_end "YOLO phase done" "${step_t0}" "${stack_t0}"

  step_t0=$SECONDS
  step_begin "5" "8" "Optional RGBD bridge"
  if [[ "${TB3_STACK_MODE}" == "assist" && "${TB3_ASSIST_RGBD_BRIDGE}" == "1" ]]; then
    setsid ros2 launch robot_bringup rgbd_to_scan.launch.py \
      use_sim_time:=true \
      depth_image_topic:="${RGBD_DEPTH_IMAGE_TOPIC}" \
      depth_camera_info_topic:="${RGBD_DEPTH_CAMERA_INFO_TOPIC}" \
      >"${TB3_LOG_DIR}/rgbd_to_scan.log" 2>&1 < /dev/null &
  fi
  step_end "RGBD bridge phase done" "${step_t0}" "${stack_t0}"

  step_t0=$SECONDS
  if [[ "${TB3_ENABLE_SLAM}" == "1" ]]; then
    step_begin "6" "8" "Start SLAM toolbox (map publisher)"
    if [[ -n "${TB3_SLAM_PARAMS_FILE}" ]]; then
      setsid ros2 launch robot_bringup slam_laser.launch.py \
        use_sim_time:=true \
        slam_params_file:="${TB3_SLAM_PARAMS_FILE}" \
        >"${TB3_LOG_DIR}/slam_toolbox.log" 2>&1 < /dev/null &
    else
      setsid ros2 launch robot_bringup slam_laser.launch.py \
        use_sim_time:=true \
        >"${TB3_LOG_DIR}/slam_toolbox.log" 2>&1 < /dev/null &
    fi
    step_end "SLAM launched (map wait in step 8)" "${step_t0}" "${stack_t0}"
  else
    step_begin "6" "8" "Start SLAM toolbox (map publisher)"
    echo "Skip SLAM toolbox (TB3_ENABLE_SLAM=0)"
    step_end "SLAM phase skipped" "${step_t0}" "${stack_t0}"
  fi

  step_t0=$SECONDS
  step_begin "7" "8" "Start GUI (optional)"
  if [[ "${TB3_ENABLE_GZCLIENT}" == "1" || "${TB3_ENABLE_RVIZ}" == "1" ]]; then
    # 渲染兼容开关：默认走软件渲染（TB3_GAZEBO_HARDWARE_GL=0）以适配 WSL。
    # 若本机 GPU 环境稳定，可设 TB3_GAZEBO_HARDWARE_GL=1 走硬件渲染。
    if [[ "${TB3_GAZEBO_HARDWARE_GL:-0}" != "1" ]]; then
      export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
      export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
      export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
      export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
      export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
      export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
      export SVGA_VGPU10="${SVGA_VGPU10:-0}"
      export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
      echo "GUI render mode: software (TB3_GAZEBO_HARDWARE_GL=0)"
      echo "  WSL: RViz 仅显示 YOLO 标注图 (/yolo_objects/annotated_image)"
    else
      echo "GUI render mode: hardware (TB3_GAZEBO_HARDWARE_GL=1)"
    fi

    if [[ "${TB3_ENABLE_GZCLIENT}" == "1" ]]; then
      setsid gzclient >"${TB3_LOG_DIR}/gzclient.log" 2>&1 < /dev/null &
    else
      echo "Skip gzclient (TB3_ENABLE_GZCLIENT=0)"
    fi

    if [[ "${TB3_ENABLE_RVIZ}" == "1" ]]; then
      setsid rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:=true \
        >"${TB3_LOG_DIR}/rviz2.log" 2>&1 < /dev/null &
    else
      echo "Skip rviz2 (TB3_ENABLE_RVIZ=0)"
    fi
  else
    echo "Skip GUI (TB3_ENABLE_GZCLIENT=0, TB3_ENABLE_RVIZ=0)"
  fi
  step_end "GUI phase done" "${step_t0}" "${stack_t0}"

  # 仅等待已启用模块对应话题，避免无效等待拖慢启动。
  step_t0=$SECONDS
  step_begin "8" "8" "Warm-up topic checks"
  wait_topic_with_timing "/scan" 20 "${stack_t0}" || true
  if [[ "${TB3_ENABLE_SLAM}" == "1" ]]; then
    wait_topic_with_timing "/map" 20 "${stack_t0}" || true
  fi
  if [[ "${yolo_enabled}" == "1" ]]; then
    wait_topic_with_timing "/yolo_objects/annotated_image" 20 "${stack_t0}" || true
  fi
  step_end "warm-up checks done" "${step_t0}" "${stack_t0}"
  echo "Total startup time: $(format_duration "$((SECONDS - stack_t0))")"
  echo "Stack started. Logs: ${TB3_LOG_DIR}"
}

do_stop() {
  cleanup_old
  echo "Stack stopped."
}

do_check() {
  local status=0
  local topics=(/clock /scan /odom /tf /tf_static /yolo_objects/annotated_image)
  if [[ "${TB3_ENABLE_SLAM:-1}" == "1" ]]; then
    topics+=(/map)
  fi
  for topic in "${topics[@]}"; do
    if ros2 topic info "${topic}" >/dev/null 2>&1; then
      echo "OK   ${topic}"
    else
      echo "MISS ${topic}"
      status=1
    fi
  done
  if (( status == 0 )); then
    echo "Smoke test passed"
  else
    echo "Smoke test failed"
    exit 1
  fi
}

do_logs() {
  local target="${2:-all}"
  case "${target}" in
    gzserver) tail -f "${TB3_LOG_DIR}/gzserver.log" ;;
    gzclient) tail -f "${TB3_LOG_DIR}/gzclient.log" ;;
    rviz|rviz2) tail -f "${TB3_LOG_DIR}/rviz2.log" ;;
    rsp|robot_state_publisher) tail -f "${TB3_LOG_DIR}/robot_state_publisher.log" ;;
    yolo|yolo_object|yolo_person) tail -f "${TB3_LOG_DIR}/yolo_object_seg.log" ;;
    rgbd|rgbd_to_scan|depth) tail -f "${TB3_LOG_DIR}/rgbd_to_scan.log" ;;
    all) ls -1 "${TB3_LOG_DIR}" ;;
    *) echo "Unknown log target: ${target}"; exit 1 ;;
  esac
}

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tb3_stack.sh start
  bash scripts/tb3_stack.sh stop   (含 Nav2: navigation / tj_static_map_nav2 launch)
  bash scripts/tb3_stack.sh check
  bash scripts/tb3_stack.sh logs [all|gzserver|gzclient|rviz|rsp|yolo|rgbd]

Common GUI env switches:
  TB3_ENABLE_GZCLIENT=0/1   (disable/enable Gazebo client window)
  TB3_ENABLE_RVIZ=0/1       (disable/enable RViz window)
  TB3_NO_GUI=1              (legacy switch, disable both)
  TB3_PATCH_SPAWN_SDF_EXTRAS=1 Patch spawn SDF: strip RGB link collisions / laser noise (defaults below).
  TB3_STRIP_CAMERA_RGB_COLLISION=1  Remove camera_rgb_frame collisions (reduces LDS self-occlusion in sim).
  TB3_CAMERA_RGB_Z_M=0.89       主相机高度（m，相对 base_link；改 waffle 的 camera_joint；官方约 0.094）
  TB3_CAMERA_MOUNT_OFFICIAL=1     不抬高相机，保持官方高度
  TB3_SIM_LASER_RANGE_STDDEV=0     Gazebo ray range Gaussian stddev for hls_lfcd_lds (meters).
  TB3_SIM_CAMERA_WIDTH=640         Gazebo RGB sensor width (run_simulation defaults).
  TB3_SIM_CAMERA_HEIGHT=480        Gazebo RGB sensor height.
  TB3_SIM_UNIFIED_RGBD=1           1=单传感器 RGB-D (/camera/image_raw + /camera/depth/*); 0=tb3_depth_only.
  TB3_SIM_DEPTH_WIDTH=640          Gazebo depth sensor width (matched to RGB by default).
  TB3_SIM_DEPTH_HEIGHT=480         Gazebo depth sensor height.
  TB3_DEPTH_CLIP_FAR_M=12.0        Depth far clip (m); align with YOLO depth_max when possible.
  TB3_YOLO_DEPTH_REGISTER=0        depth_image_proc register; 1=RGB grid (需 TF 正常).
  TB3_YOLO_DEPTH_SAMPLE_STAT=min     ROI depth: min|median|trimmed_mean (min 避免远墙 median).
  TB3_YOLO_DEPTH_RANGE_TO_OPTICAL_Z=true  Tilted camera: range->optical Z.
  TB3_GZSERVER_WAIT_SEC=90         Max wait for /spawn_entity after gzserver start (run_simulation uses 120).
  TB3_ROBOT_DESCRIPTION_TOPIC_HZ=2 Rate for publishing /robot_description (std_msgs/String) for RViz.

This simplified stack keeps only:
- RGBD robot simulation
- laser SLAM map (/map)
- YOLO recognition
- optional depth->scan bridge
EOF
}

case "${1:-}" in
  start) do_start ;;
  stop) do_stop ;;
  check) do_check ;;
  logs) do_logs "$@" ;;
  *) usage; exit 1 ;;
esac
