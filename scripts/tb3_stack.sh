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

TB3_STACK_MODE="${TB3_STACK_MODE:-assist}"
TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-waffle}"
TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-1}"
TB3_ENABLE_SLAM="${TB3_ENABLE_SLAM:-1}"
TB3_LOG_DIR="${TB3_LOG_DIR:-/tmp/tb3_stack}"
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
YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
YOLO_CAMERA_INFO_TOPIC="${YOLO_CAMERA_INFO_TOPIC:-/camera/camera_info}"
YOLO_DEVICE="${YOLO_DEVICE:-auto}"
RGBD_DEPTH_IMAGE_TOPIC="${RGBD_DEPTH_IMAGE_TOPIC:-/tb3_depth_only/depth/image_raw}"
RGBD_DEPTH_CAMERA_INFO_TOPIC="${RGBD_DEPTH_CAMERA_INFO_TOPIC:-/tb3_depth_only/depth/camera_info}"
TB3_SLAM_PARAMS_FILE="${TB3_SLAM_PARAMS_FILE:-}"
ROBOT_START_X="${ROBOT_START_X:-0.0}"
ROBOT_START_Y="${ROBOT_START_Y:-0.0}"
ROBOT_START_Z="${ROBOT_START_Z:-0.1}"
ROBOT_START_YAW="${ROBOT_START_YAW:-0.0}"
TB3_ENABLE_RSP="${TB3_ENABLE_RSP:-1}"

mkdir -p "${TB3_LOG_DIR}"

cleanup_old() {
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f "ros2 launch human_yolo_seg yolo_person_seg.launch.py" 2>/dev/null || true
  pkill -9 -f "ros2 launch robot_bringup rgbd_to_scan.launch.py" 2>/dev/null || true
  pkill -9 -f "ros2 launch robot_bringup slam_laser.launch.py" 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f '/robot_description std_msgs/msg/String' 2>/dev/null || true
  pkill -9 -f yolo_person_seg_node 2>/dev/null || true
  pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
  pkill -9 -f slam_toolbox 2>/dev/null || true
  pkill -9 -f point_cloud_xyz_node 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null || true
  pkill -9 -f depth_image_to_viz.py 2>/dev/null || true
}

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

publish_robot_description() {
  local urdf_file="$1"
  local payload
  payload="$(python3 -c 'import json,sys; print("{data: " + json.dumps(open(sys.argv[1], encoding="utf-8").read()) + "}")' "${urdf_file}")"
  setsid ros2 topic pub -r 1 --qos-durability transient_local /robot_description std_msgs/msg/String "${payload}" \
    >"${TB3_LOG_DIR}/robot_description.log" 2>&1 < /dev/null &
}

expand_robot_urdf() {
  local input_file="$1"
  local output_file="$2"
  if ! ros2 run xacro xacro "${input_file}" >"${output_file}" 2>"${TB3_LOG_DIR}/xacro_tb3.log"; then
    echo "ERROR: xacro 展开失败: ${input_file}" >&2
    echo "       详情见: ${TB3_LOG_DIR}/xacro_tb3.log" >&2
    return 1
  fi
}

prepare_model_with_camera_rate() {
  local input_file="$1"
  local output_file="$2"
  local camera_rate="$3"
  local enable_camera="$4"
  local camera_always_on="$5"

  if ! python3 - "${input_file}" "${output_file}" "${camera_rate}" "${enable_camera}" "${camera_always_on}" <<'PY'
import re
import sys
from pathlib import Path

input_file = Path(sys.argv[1])
output_file = Path(sys.argv[2])
rate = sys.argv[3].strip()
enable_camera = sys.argv[4].strip() == "1"
camera_always_on = sys.argv[5].strip() == "1"

text = input_file.read_text(encoding="utf-8", errors="ignore")

sensor_pattern = re.compile(
    r"(<sensor\b[^>]*\btype=['\"]camera['\"][^>]*>)(.*?)(</sensor>)",
    flags=re.S,
)

def patch_one_sensor(match: re.Match) -> str:
    head, body, tail = match.groups()
    patched = body
    target_always_on = "true" if (enable_camera and camera_always_on) else "false"
    target_rate = rate if enable_camera else "0"

    if re.search(r"<always_on>.*?</always_on>", patched, flags=re.S):
        patched = re.sub(
            r"<always_on>.*?</always_on>",
            f"<always_on>{target_always_on}</always_on>",
            patched,
            count=1,
            flags=re.S,
        )
    else:
        patched = f"\n        <always_on>{target_always_on}</always_on>" + patched

    if re.search(r"<update_rate>.*?</update_rate>", patched, flags=re.S):
        patched = re.sub(
            r"<update_rate>.*?</update_rate>",
            f"<update_rate>{target_rate}</update_rate>",
            patched,
            count=1,
            flags=re.S,
        )
    else:
        patched = f"\n        <update_rate>{target_rate}</update_rate>" + patched

    if not enable_camera:
        if re.search(r"<visualize>.*?</visualize>", patched, flags=re.S):
            patched = re.sub(
                r"<visualize>.*?</visualize>",
                "<visualize>false</visualize>",
                patched,
                count=1,
                flags=re.S,
            )
        else:
            patched = f"\n        <visualize>false</visualize>" + patched

    return head + patched + tail

new_text, count = sensor_pattern.subn(patch_one_sensor, text)
output_file.write_text(new_text, encoding="utf-8")
print(f"UPDATED_CAMERA_SENSORS={count}")
PY
  then
    echo "WARNING: 生成低帧率模型失败，回退原始 MODEL_FILE=${input_file}" >&2
    return 1
  fi
}

do_start() {
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

  echo "[1/6] Start gzserver"
  setsid gzserver "${WORLD_FILE}" --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >"${TB3_LOG_DIR}/gzserver.log" 2>&1 < /dev/null &

  if ! wait_for_service "/spawn_entity" "${TB3_GZSERVER_WAIT_SEC:-45}"; then
    echo "ERROR: gzserver 未暴露 /spawn_entity" >&2
    exit 1
  fi

  if [[ "${TB3_ENABLE_RSP}" == "1" ]]; then
    echo "[2/6] Start robot_state_publisher"
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
  else
    echo "[2/6] Skip robot_state_publisher (TB3_ENABLE_RSP=0)"
  fi

  echo "[3/6] Spawn robot model"
  local spawn_model_file="${MODEL_FILE}"
  if [[ -n "${TB3_CAMERA_UPDATE_RATE}" ]]; then
    local patched_model="${TB3_LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_camera_${TB3_CAMERA_UPDATE_RATE}hz.sdf"
    if prepare_model_with_camera_rate "${MODEL_FILE}" "${patched_model}" "${TB3_CAMERA_UPDATE_RATE}" "${TB3_ENABLE_CAMERA}" "${TB3_CAMERA_ALWAYS_ON}" \
      >"${TB3_LOG_DIR}/camera_rate_patch.log" 2>&1; then
      spawn_model_file="${patched_model}"
      echo "Using camera mode: enable=${TB3_ENABLE_CAMERA}, always_on=${TB3_CAMERA_ALWAYS_ON}, update_rate=${TB3_CAMERA_UPDATE_RATE} Hz (${spawn_model_file})"
    else
      echo "Using original model file: ${MODEL_FILE}"
    fi
  fi
  ros2 run gazebo_ros spawn_entity.py \
    -entity "${TURTLEBOT3_MODEL}" \
    -file "${spawn_model_file}" \
    -x "${ROBOT_START_X}" -y "${ROBOT_START_Y}" -z "${ROBOT_START_Z}" -Y "${ROBOT_START_YAW}" \
    >"${TB3_LOG_DIR}/spawn_entity.log" 2>&1

  echo "[4/6] Start YOLO detection"
  local yolo_enabled=0
  if [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" && "${TB3_ENABLE_CAMERA}" == "1" ]]; then
    yolo_enabled=1
    setsid ros2 launch human_yolo_seg yolo_person_seg.launch.py \
      use_sim_time:=true \
      image_topic:="${YOLO_IMAGE_TOPIC}" \
      camera_info_topic:="${YOLO_CAMERA_INFO_TOPIC}" \
      device:="${YOLO_DEVICE}" \
      >"${TB3_LOG_DIR}/yolo_person_seg.log" 2>&1 < /dev/null &
  else
    if [[ "${TB3_ASSIST_SCAN_FILTER}" != "1" ]]; then
      echo "Skip YOLO detection (TB3_ASSIST_SCAN_FILTER=0)"
    else
      echo "Skip YOLO detection (camera disabled: TB3_ENABLE_CAMERA=0)"
    fi
  fi

  echo "[5/6] Optional RGBD bridge"
  if [[ "${TB3_STACK_MODE}" == "assist" && "${TB3_ASSIST_RGBD_BRIDGE}" == "1" ]]; then
    setsid ros2 launch robot_bringup rgbd_to_scan.launch.py \
      use_sim_time:=true \
      depth_image_topic:="${RGBD_DEPTH_IMAGE_TOPIC}" \
      depth_camera_info_topic:="${RGBD_DEPTH_CAMERA_INFO_TOPIC}" \
      >"${TB3_LOG_DIR}/rgbd_to_scan.log" 2>&1 < /dev/null &
  fi

  if [[ "${TB3_ENABLE_SLAM}" == "1" ]]; then
    echo "[6/7] Start SLAM toolbox (map publisher)"
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
    wait_for_topic "/map" 30 || true
  else
    echo "[6/7] Skip SLAM toolbox (TB3_ENABLE_SLAM=0)"
  fi

  echo "[7/7] Start GUI (optional)"
  if [[ "${TB3_NO_GUI:-0}" != "1" ]]; then
    # 渲染兼容开关：默认走软件渲染（TB3_GAZEBO_HARDWARE_GL=0）以适配 WSL。
    # 若本机 GPU 环境稳定，可设 TB3_GAZEBO_HARDWARE_GL=1 走硬件渲染。
    if [[ "${TB3_GAZEBO_HARDWARE_GL:-0}" != "1" ]]; then
      export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
      export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
      export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
      export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
      export LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
      export SVGA_VGPU10="${SVGA_VGPU10:-0}"
      export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
      echo "GUI render mode: software (TB3_GAZEBO_HARDWARE_GL=0)"
    else
      echo "GUI render mode: hardware (TB3_GAZEBO_HARDWARE_GL=1)"
    fi

    setsid gzclient >"${TB3_LOG_DIR}/gzclient.log" 2>&1 < /dev/null &
    setsid rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:=true \
      >"${TB3_LOG_DIR}/rviz2.log" 2>&1 < /dev/null &
  fi

  # 仅等待已启用模块对应话题，避免无效等待拖慢启动。
  wait_for_topic "/scan" 20 || true
  if [[ "${TB3_ENABLE_SLAM}" == "1" ]]; then
    wait_for_topic "/map" 20 || true
  fi
  if [[ "${yolo_enabled}" == "1" ]]; then
    wait_for_topic "/human_yolo/annotated_image" 20 || true
  fi
  echo "Stack started. Logs: ${TB3_LOG_DIR}"
}

do_stop() {
  cleanup_old
  echo "Stack stopped."
}

do_check() {
  local status=0
  local topics=(/clock /scan /odom /tf /tf_static /human_yolo/annotated_image)
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
    yolo|yolo_person) tail -f "${TB3_LOG_DIR}/yolo_person_seg.log" ;;
    rgbd|rgbd_to_scan|depth) tail -f "${TB3_LOG_DIR}/rgbd_to_scan.log" ;;
    all) ls -1 "${TB3_LOG_DIR}" ;;
    *) echo "Unknown log target: ${target}"; exit 1 ;;
  esac
}

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tb3_stack.sh start
  bash scripts/tb3_stack.sh stop
  bash scripts/tb3_stack.sh check
  bash scripts/tb3_stack.sh logs [all|gzserver|gzclient|rviz|rsp|yolo|rgbd]

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
