#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
ROS_SETUP_BASH="${ROS_SETUP_BASH:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ROS_SETUP_BASH}" ]]; then
  echo "ERROR: 未找到 ROS 2 环境脚本: ${ROS_SETUP_BASH}" >&2
  echo "本项目默认使用 ROS 2 Humble。在 Ubuntu / WSL 上请先安装依赖：" >&2
  echo "  bash scripts/setup_env.sh" >&2
  echo "安装完成后请新开终端，或执行: source /opt/ros/humble/setup.bash" >&2
  echo "若 Humble 装在其他路径，可设置: export ROS_SETUP_BASH=/path/to/setup.bash" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${ROS_SETUP_BASH}"
set +u

ROS_WS_SETUP="${SCRIPT_DIR}/../ros_ws/install/setup.bash"
if [[ -f "${ROS_WS_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${ROS_WS_SETUP}"
else
  echo "WARNING: 未找到工作空间 ${ROS_WS_SETUP}，请先: cd ros_ws && colcon build" >&2
  echo "         否则 robot_bringup 的 launch 不可用。" >&2
fi

# 运行模式：laser=默认 burger | assist=默认 burger（RGB+YOLO 链与项目基线一致）；显式 waffle/waffle_pi 可注入深度
TB3_STACK_MODE="${TB3_STACK_MODE:-laser}"
RGBD_DEPTH_IMAGE_TOPIC="${RGBD_DEPTH_IMAGE_TOPIC:-/tb3_depth_only/depth/image_raw}"
RGBD_DEPTH_CAMERA_INFO_TOPIC="${RGBD_DEPTH_CAMERA_INFO_TOPIC:-/tb3_depth_only/depth/camera_info}"
YOLO_IMAGE_TOPIC="${YOLO_IMAGE_TOPIC:-/camera/image_raw}"
YOLO_CAMERA_INFO_TOPIC="${YOLO_CAMERA_INFO_TOPIC:-/camera/camera_info}"
# YOLO 推理设备：auto=有 CUDA 则 cuda:0（见 human_yolo_seg 节点）；cpu / cuda:0 可显式指定
YOLO_DEVICE="${YOLO_DEVICE:-auto}"
# assist：是否启动 depth→/scan_rgbd（默认 0 省算力；需深度调试时设 1）
TB3_ASSIST_RGBD_BRIDGE="${TB3_ASSIST_RGBD_BRIDGE:-0}"
# 是否用 RGB+YOLO 人物链（需已 build human_yolo_seg）。具体建图方式见 TB3_PERSON_SLAM_MODE。
TB3_ASSIST_SCAN_FILTER="${TB3_ASSIST_SCAN_FILTER:-1}"
# mark_then_strip（默认）：SLAM 用原始 /scan，person_strip_recorder 记录人物 map 点，保存地图后靠 strip_saved_map_person_free 清障
# filtered：scan_person_filter 发布 /scan_filtered，SLAM 订阅 /scan_filtered（旧行为）
TB3_PERSON_SLAM_MODE="${TB3_PERSON_SLAM_MODE:-mark_then_strip}"
# YOLO 人物方位角：linear_fov=粗略像素×视场；tf_geometry=内参+TF+地面（显式传入以免未 colcon 时仍用旧 install 默认）
TB3_PERSON_AZIMUTH_MODE="${TB3_PERSON_AZIMUTH_MODE:-linear_fov}"
# 是否额外起 scan_map_colored_cloud（整帧 /scan 投 map 上色；默认 0 减负；filtered 且要看整帧人向激光可设 1）
TB3_ENABLE_SCAN_MAP_COLORED="${TB3_ENABLE_SCAN_MAP_COLORED:-0}"
# （已弃用）原双路 YOLO+后摄；burger 现仅前向宽视场 RGB，设 1 时脚本会告警并仍走单路
TB3_YOLO_360="${TB3_YOLO_360:-0}"

if [[ "${SLAM_SENSOR:-}" == "rgbd" ]]; then
  echo "WARNING: SLAM_SENSOR=rgbd 已弃用，等价于 TB3_STACK_MODE=assist（仍只以 /scan 建图）" >&2
  TB3_STACK_MODE=assist
fi

if [[ "${TB3_STACK_MODE}" == "assist" ]]; then
  if [[ -z "${TURTLEBOT3_MODEL:-}" ]]; then
    export TURTLEBOT3_MODEL="burger"
    echo "TB3_STACK_MODE=assist：未指定 TURTLEBOT3_MODEL，默认 burger（RGB+激光/YOLO 链；无深度注入，需深度请设 waffle）" >&2
  elif [[ "${TURTLEBOT3_MODEL}" != "waffle" && "${TURTLEBOT3_MODEL}" != "waffle_pi" && "${TURTLEBOT3_MODEL}" != "burger" ]]; then
    export TURTLEBOT3_MODEL="burger"
    echo "TB3_STACK_MODE=assist：不支持的模型，已改为 burger" >&2
  fi
else
  export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
fi

export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share/turtlebot3_description}"
# moving_obstacle.sdf 使用 model://person_standing（Gazebo 经典人形，非 TB3 自带）。把用户/本包 models 目录放在路径前，便于 ~/.gazebo/models 或 robot_bringup/models/person_standing 生效。
_BR_MODELS_INSTALL=""
if command -v ros2 >/dev/null 2>&1; then
  if _rbp="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -n "${_rbp}" ]]; then
    _BR_MODELS_INSTALL="${_rbp}/share/robot_bringup/models"
  fi
fi
_BR_MODELS_SRC="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/models"
for _gz_models_prepend in "${_BR_MODELS_SRC}" "${_BR_MODELS_INSTALL}" "${HOME}/.gazebo/models"; do
  if [[ -d "${_gz_models_prepend}" ]]; then
    export GAZEBO_MODEL_PATH="${_gz_models_prepend}:${GAZEBO_MODEL_PATH}"
  fi
done
# Gazebo Classic 必须先包含系统 share（media/shaders），否则 RTShaderSystem / RenderEngine 初始化失败、gzserver 断言崩溃
TB3_DESC_PREFIX="$(ros2 pkg prefix turtlebot3_description 2>/dev/null)/share"
GZ_RES_USER="${GAZEBO_RESOURCE_PATH:-}"
GZ_RES=""
for _gzd in /usr/share/gazebo-11 /usr/share/gazebo; do
  if [[ -d "${_gzd}" ]]; then
    GZ_RES="${GZ_RES:+${GZ_RES}:}${_gzd}"
  fi
done
if [[ -d "${TB3_DESC_PREFIX}" ]]; then
  GZ_RES="${GZ_RES:+${GZ_RES}:}${TB3_DESC_PREFIX}"
fi
if [[ -n "${GZ_RES_USER}" ]]; then
  GZ_RES="${GZ_RES:+${GZ_RES}:}${GZ_RES_USER}"
fi
export GAZEBO_RESOURCE_PATH="${GZ_RES}"
if [[ -d "/opt/ros/humble/lib" ]]; then
  export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib${GAZEBO_PLUGIN_PATH:+:${GAZEBO_PLUGIN_PATH}}"
fi
export TB3_LOG_DIR="${TB3_LOG_DIR:-/tmp/tb3_stack}"
# TB3_HEADLESS=1 等价于 TB3_NO_GUI=1：不启 gzclient 与 RViz2，仅 gzserver + 后台节点，减轻图形与 CPU 占用
if [[ "${TB3_HEADLESS:-0}" == "1" ]]; then
  export TB3_NO_GUI=1
fi
# wait_for_service / wait_for_topic / wait_for_set_entity_state 里每次探测的间隔（秒）；过小会增加 ros2 CLI 调用频率
TB3_WAIT_POLL_SEC="${TB3_WAIT_POLL_SEC:-0.5}"

WORLD_FILE="${WORLD_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/world/test1.world}"
MODEL_FILE="${MODEL_FILE:-/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf}"
URDF_FILE="${URDF_FILE:-/tmp/tb3_${TURTLEBOT3_MODEL}.urdf}"
MAP_PGM_FILE="${MAP_PGM_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/maps/test1.pgm}"
MAP_YAML_FILE="${MAP_YAML_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/maps/test1.yaml}"
LOG_DIR="${TB3_LOG_DIR}"
RVIZ_CONFIG_FILE="${RVIZ_CONFIG_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/test1.rviz}"
ROBOT_START_X="${ROBOT_START_X:--2.0}"
ROBOT_START_Y="${ROBOT_START_Y:--1.2}"
ROBOT_START_Z="${ROBOT_START_Z:-0.1}"
ROBOT_START_YAW="${ROBOT_START_YAW:-0.0}"
OBSTACLE_ENTITY_NAME="${OBSTACLE_ENTITY_NAME:-moving_obstacle_1}"
OBSTACLE_SDF_FILE="${OBSTACLE_SDF_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/models/moving_obstacle.sdf}"
OBSTACLE_MOVER_FILE="${OBSTACLE_MOVER_FILE:-${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/moving_obstacle_controller.py}"
OBSTACLE_START_X="${OBSTACLE_START_X:-0.8}"
OBSTACLE_START_Y="${OBSTACLE_START_Y:-0.0}"
# 贴地：过高则整人在激光水平面之上，/scan 扫不到；若网格陷地可略调到 0.01～0.05
OBSTACLE_START_Z="${OBSTACLE_START_Z:-0.0}"
OBSTACLE_CENTER_X="${OBSTACLE_CENTER_X:-0.8}"
OBSTACLE_CENTER_Y="${OBSTACLE_CENTER_Y:-0.0}"
OBSTACLE_AMPLITUDE_X="${OBSTACLE_AMPLITUDE_X:-0.8}"
OBSTACLE_AMPLITUDE_Y="${OBSTACLE_AMPLITUDE_Y:-0.6}"
OBSTACLE_FREQUENCY="${OBSTACLE_FREQUENCY:-0.25}"
OBSTACLE_RATE_HZ="${OBSTACLE_RATE_HZ:-20.0}"
OBSTACLE_FALLBACK_RATE_HZ="${OBSTACLE_FALLBACK_RATE_HZ:-5.0}"
OBSTACLE_TRAJECTORY="${OBSTACLE_TRAJECTORY:-line}"
OBSTACLE_LISSAJOUS_AX="${OBSTACLE_LISSAJOUS_AX:-3.0}"
OBSTACLE_LISSAJOUS_AY="${OBSTACLE_LISSAJOUS_AY:-2.0}"
OBSTACLE_LISSAJOUS_PHASE="${OBSTACLE_LISSAJOUS_PHASE:-1.57079632679}"
OBSTACLE_PATROL_SPEED="${OBSTACLE_PATROL_SPEED:-0.6}"

mkdir -p "${LOG_DIR}"
mkdir -p "$(dirname "${RVIZ_CONFIG_FILE}")"

cleanup_old() {
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 -f gazebo 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f '/robot_description std_msgs/msg/String' 2>/dev/null || true
  pkill -9 -f slam_toolbox 2>/dev/null || true
  pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f occupancy_grid_node 2>/dev/null || true
  pkill -9 -f point_cloud_xyz_node 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan_node 2>/dev/null || true
  pkill -9 -f yolo_person_seg_node 2>/dev/null || true
  pkill -9 -f person_azimuth_markers_node 2>/dev/null || true
  pkill -9 -f scan_person_filter_node 2>/dev/null || true
  pkill -9 -f person_strip_recorder_node 2>/dev/null || true
  pkill -9 -f scan_map_colored_cloud_node 2>/dev/null || true
  pkill -9 -f azimuth_union_node 2>/dev/null || true
  pkill -9 -f depth_image_to_viz 2>/dev/null || true
  pkill -9 -f tb3_moving_obstacle.py 2>/dev/null || true
  pkill -9 -f moving_obstacle_controller.py 2>/dev/null || true
}

resolve_set_entity_state_service() {
  local found_service
  found_service="$(ros2 service list | grep -E '/set_entity_state$' | head -n 1 || true)"
  if [[ -n "${found_service}" ]]; then
    echo "${found_service}"
    return 0
  fi
  return 1
}

wait_for_set_entity_state_service() {
  local timeout_s="${1:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    local service_name
    if service_name="$(resolve_set_entity_state_service)"; then
      echo "${service_name}"
      return 0
    fi
    sleep "${TB3_WAIT_POLL_SEC}"
  done
  return 1
}

wait_for_service() {
  local name="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 service list | grep -qx "${name}"; then
      return 0
    fi
    sleep "${TB3_WAIT_POLL_SEC}"
  done
  return 1
}

wait_for_topic() {
  local name="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 topic list | grep -qx "${name}"; then
      return 0
    fi
    sleep "${TB3_WAIT_POLL_SEC}"
  done
  return 1
}

# do_start 分步计时：本段耗时 + 累计（秒，精度 1s）。关闭：TB3_START_BENCH=0
tb3_bench_init() {
  _TB3_BENCH_T0="${SECONDS}"
  _TB3_BENCH_P0="${SECONDS}"
}

tb3_bench_mark() {
  if [[ "${TB3_START_BENCH:-1}" != "1" ]]; then
    return 0
  fi
  local label="$1"
  local now="${SECONDS}"
  local dphase=$((now - _TB3_BENCH_P0))
  local dtotal=$((now - _TB3_BENCH_T0))
  printf '[计时] %s: 本段 %ds | 累计 %ds\n' "${label}" "${dphase}" "${dtotal}"
  _TB3_BENCH_P0="${SECONDS}"
}

check_topic() {
  local topic="$1"
  if ros2 topic list | grep -qx "${topic}"; then
    echo "OK   topic ${topic}"
  else
    echo "FAIL topic ${topic}"
    return 1
  fi
}

# 返回话题发布者数量（ROS2 里仅有订阅者时 topic list 仍可能出现，故不能单靠 list 判断）
topic_publisher_count() {
  local topic="$1"
  local line
  line="$(ros2 topic info "${topic}" 2>/dev/null | grep -m1 '^Publisher count:')" || true
  if [[ "${line}" =~ Publisher\ count:\ ([0-9]+) ]]; then
    echo "${BASH_REMATCH[1]}"
  else
    echo "0"
  fi
}

check_echo() {
  local label="$1"
  shift
  if timeout 12s "$@" >/tmp/tb3_smoke.out 2>&1; then
    echo "OK   ${label}"
  else
    echo "FAIL ${label}"
    cat /tmp/tb3_smoke.out
    return 1
  fi
}

check_tf() {
  timeout 12s ros2 run tf2_ros tf2_echo map odom >/tmp/tb3_smoke.out 2>&1 || true
  if grep -q 'Translation:' /tmp/tb3_smoke.out; then
    echo "OK   map->odom tf"
  else
    echo "FAIL map->odom tf"
    cat /tmp/tb3_smoke.out
    return 1
  fi
}

publish_robot_description() {
  local payload
  payload="$(python3 -c 'import json,sys; print("{data: " + json.dumps(open(sys.argv[1], encoding="utf-8").read()) + "}")' "${URDF_FILE}")"
  setsid ros2 topic pub -r 1 --qos-durability transient_local /robot_description std_msgs/msg/String "${payload}" \
    >"${LOG_DIR}/robot_description.log" 2>&1 < /dev/null &
}

generate_rviz_config() {
  cat >"${RVIZ_CONFIG_FILE}" <<EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /TF1/Frames1
      Splitter Ratio: 0.5
    Tree Height: 694
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.30000001192092896
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Update Interval: 0
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 0; 0
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.019999999552965164
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: YOLO (/human_yolo/annotated_image)
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /human_yolo/annotated_image
      Value: true
    - Angle Tolerance: 0.10000000149011612
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: false
      Enabled: true
      Keep: 1
      Name: Odometry
      Position Tolerance: 0.10000000149011612
      Shape:
        Alpha: 1
        Axes Length: 1
        Axes Radius: 0.10000000149011612
        Color: 255; 25; 0
        Head Length: 0.30000001192092896
        Head Radius: 0.10000000149011612
        Shaft Length: 1
        Shaft Radius: 0.05000000074505806
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: true
      Enabled: true
      Name: Map
      Topic:
        Depth: 10
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Use Timestamp: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 0
      Color Transformer: RGB8
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: 人物标记（亮黄点云，叠在地图上）
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (m): 0.06499999761581421
      Style: Flat Squares
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /human_yolo/person_laser_map_cloud
      Use Fixed Frame: true
      Use rainbow: false
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: 人物方位角（绿圆弧）
      Topic:
        Depth: 10
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /human_yolo/person_azimuth_markers
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 8
      Focal Point:
        X: -2
        Y: -0.5
        Z: 0
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 72
  Y: 60
EOF
}

do_start() {
  tb3_bench_init
  echo "[1/12] Cleaning old processes"
  cleanup_old
  tb3_bench_mark "1/12 清理旧进程"

  echo "[2/12] Using world: ${WORLD_FILE}"
  echo "[2/12] Using map pgm: ${MAP_PGM_FILE}, yaml: ${MAP_YAML_FILE}"

  echo "[2/12] Starting gzserver (TB3_STACK_MODE=${TB3_STACK_MODE})"
  # WSL / 无可用 GL 时 Ogre 易 “Failed to initialize scene”；真机 GPU 可 export TB3_GAZEBO_HARDWARE_GL=1
  if [[ "${TB3_GAZEBO_HARDWARE_GL:-0}" != "1" ]]; then
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
    export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
  fi
  setsid gzserver "${WORLD_FILE}" --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >"${LOG_DIR}/gzserver.log" 2>&1 < /dev/null &
  local gz_pid=$!

  if ! wait_for_service "/spawn_entity" 20; then
    echo "gzserver did not expose /spawn_entity"
    tail -n 80 "${LOG_DIR}/gzserver.log" || true
    exit 1
  fi
  tb3_bench_mark "2/12 gzserver 启动并等待 /spawn_entity"

  echo "[3/12] Expanding URDF and starting robot_state_publisher"
  TB3_DESC_SHARE="$(ros2 pkg prefix turtlebot3_description 2>/dev/null)/share/turtlebot3_description"
  if [[ -z "${TB3_DESC_SHARE}" || ! -d "${TB3_DESC_SHARE}" ]]; then
    TB3_DESC_SHARE="/opt/ros/humble/share/turtlebot3_description"
  fi
  BASE_TB3_URDF="${TB3_DESC_SHARE}/urdf/turtlebot3_${TURTLEBOT3_MODEL}.urdf"
  TB3_XACRO_OUT="${LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_xacro.urdf"
  if ! ros2 run xacro xacro "${BASE_TB3_URDF}" >"${TB3_XACRO_OUT}" 2>"${LOG_DIR}/xacro_tb3.log"; then
    echo "[3/12] ERROR: xacro 展开失败，见 ${LOG_DIR}/xacro_tb3.log"
    cat "${LOG_DIR}/xacro_tb3.log" || true
    exit 1
  fi

  MERGE_BURGER_RGB_PY=""
  PREP_BURGER_RGB_SDF_PY=""
  BURGER_RGB_FRAG=""
  if ROS_BR_PREFIX="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -n "${ROS_BR_PREFIX}" ]]; then
    _br_share="${ROS_BR_PREFIX}/share/robot_bringup"
    MERGE_BURGER_RGB_PY="${_br_share}/scripts/merge_tb3_burger_rgb_urdf.py"
    PREP_BURGER_RGB_SDF_PY="${_br_share}/scripts/prepare_burger_rgb_camera_sdf.py"
    BURGER_RGB_FRAG="${_br_share}/urdf/burger_rgb_camera.urdf.fragment"
  fi
  if [[ -z "${MERGE_BURGER_RGB_PY}" || ! -f "${MERGE_BURGER_RGB_PY}" ]]; then
    _src_br="${SCRIPT_DIR}/../ros_ws/src/robot_bringup"
    MERGE_BURGER_RGB_PY="${_src_br}/scripts/merge_tb3_burger_rgb_urdf.py"
    PREP_BURGER_RGB_SDF_PY="${_src_br}/scripts/prepare_burger_rgb_camera_sdf.py"
    BURGER_RGB_FRAG="${_src_br}/urdf/burger_rgb_camera.urdf.fragment"
  fi

  local needs_burger_rgb=0
  if [[ "${TURTLEBOT3_MODEL}" == "burger" ]]; then
    if [[ "${TB3_STACK_MODE}" == "assist" ]]; then
      needs_burger_rgb=1
    elif [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" ]] && ros2 pkg prefix human_yolo_seg >/dev/null 2>&1; then
      needs_burger_rgb=1
    fi
  fi

  local BURGER_RGB_SDF=""
  local URDF_FOR_RSP="${TB3_XACRO_OUT}"
  if [[ "${needs_burger_rgb}" -eq 1 ]]; then
    if [[ ! -f "${MERGE_BURGER_RGB_PY}" || ! -f "${PREP_BURGER_RGB_SDF_PY}" || ! -f "${BURGER_RGB_FRAG}" ]]; then
      echo "[3/12] ERROR: 未找到 merge_tb3_burger_rgb_urdf / prepare_burger_rgb_camera_sdf 或 fragment；请 colcon build robot_bringup"
      exit 1
    fi
    local _burger_merged="${LOG_DIR}/turtlebot3_burger_rgb_merged.urdf"
    echo "[3/12] burger + RGB：合并 URDF（camera_rgb_optical_frame）并生成带 RGB 的 SDF（与 waffle 相机位姿一致）"
    if ! python3 "${MERGE_BURGER_RGB_PY}" "${TB3_XACRO_OUT}" "${BURGER_RGB_FRAG}" "${_burger_merged}" >"${LOG_DIR}/merge_burger_rgb.log" 2>&1; then
      echo "[3/12] ERROR: merge_tb3_burger_rgb_urdf 失败，见 ${LOG_DIR}/merge_burger_rgb.log"
      cat "${LOG_DIR}/merge_burger_rgb.log" || true
      exit 1
    fi
    URDF_FOR_RSP="${_burger_merged}"
    BURGER_RGB_SDF="${LOG_DIR}/turtlebot3_burger_rgb.sdf"
    if ! python3 "${PREP_BURGER_RGB_SDF_PY}" "${MODEL_FILE}" "${BURGER_RGB_SDF}" >"${LOG_DIR}/prepare_burger_rgb_sdf.log" 2>&1; then
      echo "[3/12] ERROR: prepare_burger_rgb_camera_sdf 失败，见 ${LOG_DIR}/prepare_burger_rgb_sdf.log"
      cat "${LOG_DIR}/prepare_burger_rgb_sdf.log" || true
      exit 1
    fi
  fi

  # assist + waffle：官方 model.sdf + 注入深度；assist + burger：仅用 BURGER_RGB_SDF（无深度）；laser + burger + YOLO 链同上
  ASSIST_SPAWN_SDF=""
  if [[ "${TB3_STACK_MODE}" == "assist" && "${TURTLEBOT3_MODEL}" != "burger" ]]; then
    if [[ "${TURTLEBOT3_MODEL}" != "waffle" && "${TURTLEBOT3_MODEL}" != "waffle_pi" ]]; then
      echo "[3/12] ERROR: assist 下非 burger 时仅支持 waffle / waffle_pi"
      exit 1
    fi
    PREP_SDF_PY=""
    if ROS_BR_PREFIX="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -n "${ROS_BR_PREFIX}" ]]; then
      _br_share="${ROS_BR_PREFIX}/share/robot_bringup"
      PREP_SDF_PY="${_br_share}/scripts/prepare_assist_waffle_sdf.py"
    fi
    if [[ -z "${PREP_SDF_PY}" || ! -f "${PREP_SDF_PY}" ]]; then
      _src_br="${SCRIPT_DIR}/../ros_ws/src/robot_bringup"
      PREP_SDF_PY="${_src_br}/scripts/prepare_assist_waffle_sdf.py"
    fi
    if [[ ! -f "${PREP_SDF_PY}" ]]; then
      echo "[3/12] ERROR: 未找到 prepare_assist_waffle_sdf.py；请 cd ros_ws && colcon build --packages-select robot_bringup"
      exit 1
    fi
    ASSIST_SPAWN_SDF="${LOG_DIR}/turtlebot3_${TURTLEBOT3_MODEL}_assist.sdf"
    echo "[3/12] assist（waffle）：由官方 model.sdf 注入深度传感器"
    if ! python3 "${PREP_SDF_PY}" "${MODEL_FILE}" "${ASSIST_SPAWN_SDF}" >"${LOG_DIR}/prepare_assist_sdf.log" 2>&1; then
      echo "[3/12] ERROR: prepare_assist_waffle_sdf 失败，见 ${LOG_DIR}/prepare_assist_sdf.log"
      cat "${LOG_DIR}/prepare_assist_sdf.log" || true
      exit 1
    fi
  elif [[ "${TB3_STACK_MODE}" == "assist" && "${TURTLEBOT3_MODEL}" == "burger" ]]; then
    echo "[3/12] assist（burger）：仅 RGB 相机 SDF，无深度注入（与 laser+YOLO 用同一车体）"
  fi

  cp -f "${URDF_FOR_RSP}" "${URDF_FILE}"
  setsid ros2 run robot_state_publisher robot_state_publisher "${URDF_FILE}" --ros-args -p use_sim_time:=true \
    >"${LOG_DIR}/robot_state_publisher.log" 2>&1 < /dev/null &
  local rsp_pid=$!

  if ! wait_for_topic "/tf_static" 15; then
    echo "robot_state_publisher did not publish /tf_static"
    tail -n 80 "${LOG_DIR}/robot_state_publisher.log" || true
    exit 1
  fi
  tb3_bench_mark "3/12 URDF/SDF、robot_state_publisher、等待 /tf_static"

  echo "[4/12] Publishing /robot_description"
  publish_robot_description
  tb3_bench_mark "4/12 发布 /robot_description"

  echo "[5/12] Spawning TurtleBot3 entity"
  TB3_SPAWN_FILE="${MODEL_FILE}"
  if [[ -n "${ASSIST_SPAWN_SDF:-}" ]]; then
    TB3_SPAWN_FILE="${ASSIST_SPAWN_SDF}"
    echo "[5/12] assist（waffle）：spawn 注入深度后的 model.sdf"
  elif [[ -n "${BURGER_RGB_SDF:-}" ]]; then
    TB3_SPAWN_FILE="${BURGER_RGB_SDF}"
    echo "[5/12] burger + RGB：spawn 带 RGB 相机的 model.sdf"
  else
    echo "[5/12] 默认：turtlebot3_gazebo 官方 model.sdf"
  fi
  if ! ros2 run gazebo_ros spawn_entity.py \
    -entity "${TURTLEBOT3_MODEL}" \
    -file "${TB3_SPAWN_FILE}" \
    -x "${ROBOT_START_X}" -y "${ROBOT_START_Y}" -z "${ROBOT_START_Z}" -Y "${ROBOT_START_YAW}" \
    >"${LOG_DIR}/spawn_entity.log" 2>&1; then
    echo "[5/12] ERROR: spawn_entity 失败，见 ${LOG_DIR}/spawn_entity.log"
    tail -n 60 "${LOG_DIR}/spawn_entity.log" || true
    exit 1
  fi
  tb3_bench_mark "5/12 spawn TurtleBot3"

  echo "[6/12] Spawning moving obstacle"
  if [[ ! -f "${OBSTACLE_SDF_FILE}" ]]; then
    echo "[6/12] ERROR: obstacle model not found: ${OBSTACLE_SDF_FILE}"
    exit 1
  fi
  # set -e：spawn 失败时必须在此处理，否则 stderr 进日志、终端无任何提示即退出
  if ! ros2 run gazebo_ros spawn_entity.py \
    -entity "${OBSTACLE_ENTITY_NAME}" \
    -file "${OBSTACLE_SDF_FILE}" \
    -x "${OBSTACLE_START_X}" -y "${OBSTACLE_START_Y}" -z "${OBSTACLE_START_Z}" \
    >"${LOG_DIR}/spawn_obstacle.log" 2>&1; then
    echo "[6/12] ERROR: spawn moving obstacle 失败（常见：gzserver 已崩、实体名冲突、SDF 路径无效）"
    echo "        日志: ${LOG_DIR}/spawn_obstacle.log"
    tail -n 40 "${LOG_DIR}/spawn_obstacle.log" || true
    echo "        若为空或 Connection refused，请查看: ${LOG_DIR}/gzserver.log"
    tail -n 30 "${LOG_DIR}/gzserver.log" || true
    exit 1
  fi
  tb3_bench_mark "6/12 spawn 动态障碍物"

  local obstacle_mover_pid=""
  local set_entity_state_service=""
  if [[ ! -f "${OBSTACLE_MOVER_FILE}" ]]; then
    echo "[7/12] WARNING: obstacle controller not found: ${OBSTACLE_MOVER_FILE}"
  elif set_entity_state_service="$(wait_for_set_entity_state_service 20)"; then
    echo "[7/12] Starting moving obstacle controller (${set_entity_state_service})"
    setsid python3 "${OBSTACLE_MOVER_FILE}" \
      --mode ros \
      --entity-name "${OBSTACLE_ENTITY_NAME}" \
      --service-name "${set_entity_state_service}" \
      --center-x "${OBSTACLE_CENTER_X}" \
      --center-y "${OBSTACLE_CENTER_Y}" \
      --z "${OBSTACLE_START_Z}" \
      --amplitude-x "${OBSTACLE_AMPLITUDE_X}" \
      --amplitude-y "${OBSTACLE_AMPLITUDE_Y}" \
      --frequency "${OBSTACLE_FREQUENCY}" \
      --trajectory "${OBSTACLE_TRAJECTORY}" \
      --lissajous-ax "${OBSTACLE_LISSAJOUS_AX}" \
      --lissajous-ay "${OBSTACLE_LISSAJOUS_AY}" \
      --lissajous-phase "${OBSTACLE_LISSAJOUS_PHASE}" \
      --patrol-speed "${OBSTACLE_PATROL_SPEED}" \
      --rate-hz "${OBSTACLE_RATE_HZ}" \
      >"${LOG_DIR}/moving_obstacle.log" 2>&1 < /dev/null &
    obstacle_mover_pid=$!
  elif command -v gz >/dev/null 2>&1; then
    echo "[7/12] WARNING: no set_entity_state service found, using gz model fallback"
    echo "[7/12] fallback rate_hz=${OBSTACLE_FALLBACK_RATE_HZ} (set OBSTACLE_FALLBACK_RATE_HZ to adjust)"
    setsid python3 "${OBSTACLE_MOVER_FILE}" \
      --mode gz \
      --entity-name "${OBSTACLE_ENTITY_NAME}" \
      --center-x "${OBSTACLE_CENTER_X}" \
      --center-y "${OBSTACLE_CENTER_Y}" \
      --z "${OBSTACLE_START_Z}" \
      --amplitude-x "${OBSTACLE_AMPLITUDE_X}" \
      --amplitude-y "${OBSTACLE_AMPLITUDE_Y}" \
      --frequency "${OBSTACLE_FREQUENCY}" \
      --trajectory "${OBSTACLE_TRAJECTORY}" \
      --lissajous-ax "${OBSTACLE_LISSAJOUS_AX}" \
      --lissajous-ay "${OBSTACLE_LISSAJOUS_AY}" \
      --lissajous-phase "${OBSTACLE_LISSAJOUS_PHASE}" \
      --patrol-speed "${OBSTACLE_PATROL_SPEED}" \
      --rate-hz "${OBSTACLE_FALLBACK_RATE_HZ}" \
      >"${LOG_DIR}/moving_obstacle.log" 2>&1 < /dev/null &
    obstacle_mover_pid=$!
  else
    echo "[7/12] WARNING: no set_entity_state service and no gz command found"
  fi
  tb3_bench_mark "7/12 等待 /set_entity_state（若有）并启动障碍物控制器"

  local yolo_pid=""
  local use_filtered_slam=0
  local filtered_slam_yaml=""
  if [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" ]] && ros2 pkg prefix human_yolo_seg >/dev/null 2>&1; then
    local _slam_cfg_name="mapper_params_online_async_full_scan.yaml"
    if [[ "${TB3_PERSON_SLAM_MODE}" == "filtered" ]]; then
      _slam_cfg_name="mapper_params_online_async_scan_filtered.yaml"
    fi
    if _brp="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -f "${_brp}/share/robot_bringup/config/${_slam_cfg_name}" ]]; then
      filtered_slam_yaml="${_brp}/share/robot_bringup/config/${_slam_cfg_name}"
    elif [[ -f "${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/${_slam_cfg_name}" ]]; then
      filtered_slam_yaml="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/${_slam_cfg_name}"
    fi
    if [[ -n "${filtered_slam_yaml}" ]]; then
      use_filtered_slam=1
    else
      echo "[8/12] WARNING: 未找到 ${_slam_cfg_name}，SLAM 将使用默认参数（通常仍为 /scan）"
    fi
    TB3_SCAN_FOV_LIMIT="${TB3_SCAN_FOV_LIMIT:-0}"
    TB3_SCAN_FOV_MIN_DEG="${TB3_SCAN_FOV_MIN_DEG:--70.0}"
    TB3_SCAN_FOV_MAX_DEG="${TB3_SCAN_FOV_MAX_DEG:-70.0}"
    local _lim_fov="false"
    if [[ "${TB3_SCAN_FOV_LIMIT}" == "1" ]]; then
      _lim_fov="true"
    fi
    local _person_mode="${TB3_PERSON_SLAM_MODE:-mark_then_strip}"
    local _enable_scan_map_colored="false"
    if [[ "${TB3_ENABLE_SCAN_MAP_COLORED}" == "1" ]]; then
      _enable_scan_map_colored="true"
    fi

    if [[ "${TB3_YOLO_360}" == "1" ]]; then
      echo "[8/12] WARNING: TB3_YOLO_360=1 已弃用（burger 无后摄）；改用单路 YOLO + 宽视场前摄" >&2
    fi
    if [[ "${_person_mode}" == "filtered" ]]; then
      echo "[8/12] YOLO + 人物链：scan_person_filter + YOLO（/scan→/scan_filtered；map 上人点需 TB3_ENABLE_SCAN_MAP_COLORED=1 或改 mark_then_strip；device=${YOLO_DEVICE}）"
    else
      echo "[8/12] YOLO + 人物链：person_strip_recorder + YOLO + 方位角 Marker（SLAM 用 /scan；可选 TB3_ENABLE_SCAN_MAP_COLORED=1 整帧上色；device=${YOLO_DEVICE}）"
    fi
    setsid ros2 launch human_yolo_seg yolo_person_seg.launch.py \
      use_sim_time:=true \
      image_topic:="${YOLO_IMAGE_TOPIC}" \
      camera_info_topic:="${YOLO_CAMERA_INFO_TOPIC}" \
      device:="${YOLO_DEVICE}" \
      person_azimuth_mode:="${TB3_PERSON_AZIMUTH_MODE}" \
      person_slam_mode:="${_person_mode}" \
      limit_scan_to_fov:="${_lim_fov}" \
      fov_min_deg:="${TB3_SCAN_FOV_MIN_DEG}" \
      fov_max_deg:="${TB3_SCAN_FOV_MAX_DEG}" \
      enable_scan_map_colored:=${_enable_scan_map_colored} \
      >"${LOG_DIR}/yolo_person_seg.log" 2>&1 < /dev/null &
    yolo_pid=$!
  else
    if [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" ]]; then
      echo "[8/12] TB3_ASSIST_SCAN_FILTER=1 但未安装 human_yolo_seg，跳过掩膜链（请 colcon build human_yolo_seg）"
    else
      echo "[8/12] TB3_ASSIST_SCAN_FILTER=0：跳过 YOLO 与 scan 过滤"
    fi
  fi
  tb3_bench_mark "8/12 YOLO + 人物链（scan 过滤 / 事后清障 或跳过）"

  # 仅激光 / 未起 YOLO 链时仍用 full_scan.yaml，与 assist+mark_then_strip 的 slam 时序一致（map_update_interval 等）
  if [[ "${use_filtered_slam}" -eq 0 ]]; then
    local _full_scan_cfg=""
    if _brp="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -f "${_brp}/share/robot_bringup/config/mapper_params_online_async_full_scan.yaml" ]]; then
      _full_scan_cfg="${_brp}/share/robot_bringup/config/mapper_params_online_async_full_scan.yaml"
    elif [[ -f "${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/mapper_params_online_async_full_scan.yaml" ]]; then
      _full_scan_cfg="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/config/mapper_params_online_async_full_scan.yaml"
    fi
    if [[ -n "${_full_scan_cfg}" ]]; then
      filtered_slam_yaml="${_full_scan_cfg}"
      use_filtered_slam=1
    else
      echo "WARNING: 未找到 mapper_params_online_async_full_scan.yaml，SLAM 将使用 slam_toolbox 包默认参数（与 assist 建图可能不一致）"
    fi
  fi

  if [[ "${use_filtered_slam}" -eq 1 ]]; then
    case "${filtered_slam_yaml}" in
      *scan_filtered.yaml)
        echo "[9/12] Starting slam_toolbox（订阅 /scan_filtered）"
        ;;
      *)
        if [[ "${TB3_ASSIST_SCAN_FILTER}" == "1" && "${TB3_PERSON_SLAM_MODE:-mark_then_strip}" != "filtered" ]]; then
          echo "[9/12] Starting slam_toolbox（订阅 /scan；人物区域见 ~/.ros/tj_person_strip_regions.yaml）"
        else
          echo "[9/12] Starting slam_toolbox（订阅 /scan；slam 使用 full_scan.yaml，与 YOLO+mark_then_strip 参数一致）"
        fi
        ;;
    esac
  else
    echo "[9/12] Starting slam_toolbox（订阅 /scan；slam_toolbox 包默认 yaml）"
  fi
  local slam_log="${LOG_DIR}/slam_backend.log"
  if [[ "${use_filtered_slam}" -eq 1 ]]; then
    setsid ros2 launch robot_bringup slam_laser.launch.py \
      use_sim_time:=true \
      slam_params_file:="${filtered_slam_yaml}" \
      >"${slam_log}" 2>&1 < /dev/null &
  else
    setsid ros2 launch robot_bringup slam_laser.launch.py \
      use_sim_time:=true \
      >"${slam_log}" 2>&1 < /dev/null &
  fi
  local slam_pid=$!

  if ! wait_for_topic "/map" 25; then
    echo "SLAM 未发布 /map（见 ${slam_log}）"
    tail -n 120 "${slam_log}" || true
    exit 1
  fi
  tb3_bench_mark "9/12 slam_toolbox 启动并等待 /map"

  local rgbd_bridge_pid=""
  if [[ "${TB3_STACK_MODE}" == "assist" && "${TB3_ASSIST_RGBD_BRIDGE}" == "1" ]]; then
    echo "[10/12] assist：RGB-D 桥 depth→/scan_rgbd（TB3_ASSIST_RGBD_BRIDGE=1）"
    setsid ros2 launch robot_bringup rgbd_to_scan.launch.py \
      use_sim_time:=true \
      depth_image_topic:="${RGBD_DEPTH_IMAGE_TOPIC}" \
      depth_camera_info_topic:="${RGBD_DEPTH_CAMERA_INFO_TOPIC}" \
      >"${LOG_DIR}/rgbd_to_scan.log" 2>&1 < /dev/null &
    rgbd_bridge_pid=$!
  elif [[ "${TB3_STACK_MODE}" == "assist" ]]; then
    echo "[10/12] assist：已跳过 rgbd_to_scan（默认 TB3_ASSIST_RGBD_BRIDGE=0；需要时 export TB3_ASSIST_RGBD_BRIDGE=1）"
  else
    echo "[10/12] laser 模式：无 RGB-D 桥（assist + TB3_ASSIST_RGBD_BRIDGE=1 可开启）"
  fi
  tb3_bench_mark "10/12 rgbd_to_scan（或跳过）"

  echo "[11/12] Preparing RViz config"
  if [[ -f "${RVIZ_CONFIG_FILE}" ]]; then
    echo "[11/12] Using existing RViz config: ${RVIZ_CONFIG_FILE}"
  else
    echo "[11/12] RViz config not found, generating default: ${RVIZ_CONFIG_FILE}"
    generate_rviz_config
  fi
  tb3_bench_mark "11/12 RViz 配置"

  echo "[12/12] Verifying key topics"
  # 只调用一次 ros2 topic list（WSL+/mnt/e 下每次 CLI 冷启动可达数秒～十余秒，循环内重复调用会把本步拖到 ~100s）
  local _tb3_topic_list
  _tb3_topic_list="$(ros2 topic list 2>/dev/null)" || _tb3_topic_list=""
  for topic in /clock /odom /tf /tf_static /robot_description /map /map_metadata; do
    if printf '%s\n' "${_tb3_topic_list}" | grep -qx "${topic}"; then
      echo "  OK  ${topic}"
    else
      echo "  MISS ${topic}"
    fi
  done
  if printf '%s\n' "${_tb3_topic_list}" | grep -qx "/scan"; then
    echo "  OK  /scan"
  else
    echo "  MISS /scan"
  fi
  if [[ "${TB3_STACK_MODE}" == "assist" && "${TB3_ASSIST_RGBD_BRIDGE}" == "1" ]]; then
    local _dc _sc
    _dc="$(topic_publisher_count "${RGBD_DEPTH_IMAGE_TOPIC}")"
    _sc="$(topic_publisher_count /scan_rgbd)"
    if [[ "${_dc}" -gt 0 ]]; then
      echo "  OK  ${RGBD_DEPTH_IMAGE_TOPIC} (publishers=${_dc})"
    else
      echo "  WARN ${RGBD_DEPTH_IMAGE_TOPIC} 无发布者"
    fi
    if [[ "${_sc}" -gt 0 ]]; then
      echo "  OK  /scan_rgbd (publishers=${_sc})"
    else
      echo "  WARN /scan_rgbd 无发布者"
    fi
  elif [[ "${TB3_STACK_MODE}" == "assist" ]]; then
    echo "  （assist 且 TB3_ASSIST_RGBD_BRIDGE=0：未检查深度与 /scan_rgbd）"
  fi
  if [[ -n "${yolo_pid}" ]]; then
    local _yc _ya _sf
    _yc="$(topic_publisher_count /human_yolo/annotated_image)"
    _ya="$(topic_publisher_count /human_yolo/person_azimuth_ranges)"
    _sf="$(topic_publisher_count /scan_filtered)"
    if [[ "${_yc}" -gt 0 ]]; then
      echo "  OK  /human_yolo/annotated_image (publishers=${_yc})"
    else
      echo "  WARN /human_yolo/annotated_image 无发布者（检查相机与 yolo_person_seg.log）"
    fi
    if [[ "${_ya}" -gt 0 ]]; then
      echo "  OK  /human_yolo/person_azimuth_ranges (publishers=${_ya})"
    else
      echo "  WARN /human_yolo/person_azimuth_ranges 无发布者"
    fi
    if [[ "${TB3_PERSON_SLAM_MODE:-mark_then_strip}" == "filtered" ]]; then
      if [[ "${_sf}" -gt 0 ]]; then
        echo "  OK  /scan_filtered (publishers=${_sf})"
      else
        echo "  WARN /scan_filtered 无发布者（检查 yolo_person_seg.log 或 yolo_person_map_pipeline.log）"
      fi
    else
      echo "  （mark_then_strip：未使用 /scan_filtered；人物记录见 ~/.ros/tj_person_strip_regions.yaml）"
    fi
  fi
  tb3_bench_mark "12/12 话题检查（含 YOLO）"

  local gzclient_pid=""
  local rviz_pid=""
  if [[ "${TB3_NO_GUI:-0}" == "1" ]]; then
    echo "GUI: Skipping gzclient and RViz2 (TB3_NO_GUI=1${TB3_HEADLESS:+ / TB3_HEADLESS=1})"
  else
    echo "GUI: Starting gzclient and RViz2"
    setsid gzclient \
      >"${LOG_DIR}/gzclient.log" 2>&1 < /dev/null &
    gzclient_pid=$!

    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
    export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
    export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"

    setsid rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:=true \
      >"${LOG_DIR}/rviz2.log" 2>&1 < /dev/null &
    rviz_pid=$!
  fi
  if [[ "${TB3_NO_GUI:-0}" == "1" ]]; then
    tb3_bench_mark "GUI 已跳过（TB3_NO_GUI）"
  else
    tb3_bench_mark "GUI 启动 gzclient + rviz2"
  fi

  if [[ "${TB3_START_BENCH:-1}" == "1" ]]; then
    printf '[计时] start 总耗时: %ds\n' "$((SECONDS - _TB3_BENCH_T0))"
  fi

  echo
  echo "PIDs:"
  echo "  gzserver: ${gz_pid}"
  echo "  robot_state_publisher: ${rsp_pid}"
  echo "  slam (laser): ${slam_pid}"
  if [[ -n "${rgbd_bridge_pid}" ]]; then
    echo "  rgbd_to_scan (depth→/scan_rgbd): ${rgbd_bridge_pid}"
  fi
  if [[ -n "${yolo_pid}" ]]; then
    echo "  yolo_person_seg (launch): ${yolo_pid}"
  fi
  if [[ -n "${obstacle_mover_pid}" ]]; then
    echo "  moving_obstacle_controller: ${obstacle_mover_pid}"
  fi
  if [[ -n "${gzclient_pid}" ]]; then
    echo "  gzclient: ${gzclient_pid}"
  fi
  if [[ -n "${rviz_pid}" ]]; then
    echo "  rviz2: ${rviz_pid}"
  fi
  echo
  echo "Logs: ${LOG_DIR}"
  echo "RViz config: ${RVIZ_CONFIG_FILE}"
  echo "Moving obstacle log: ${LOG_DIR}/moving_obstacle.log"
  if [[ "${TB3_NO_GUI:-0}" != "1" ]]; then
    echo
    echo "Keyboard teleop (new terminal):"
    echo "  source /opt/ros/humble/setup.bash"
    if [[ -f "${ROS_WS_SETUP}" ]]; then
      echo "  source ${ROS_WS_SETUP}"
    fi
    echo "  export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}  # 须与当前仿真模型一致"
    local _tp_slam=""
    if _brp="$(ros2 pkg prefix robot_bringup 2>/dev/null)" && [[ -f "${_brp}/share/robot_bringup/scripts/tb3_teleop_keyboard_slam.py" ]]; then
      _tp_slam="${_brp}/share/robot_bringup/scripts/tb3_teleop_keyboard_slam.py"
    else
      _tp_slam="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/tb3_teleop_keyboard_slam.py"
    fi
    echo "  python3 ${_tp_slam}"
    echo "  （建图推荐：线速度步进更大、角速度上限更低；可调 TB3_TELEOP_MAX_ANG_VEL 等，见脚本注释）"
    echo "  官方遥控: ros2 run turtlebot3_teleop teleop_keyboard"
    echo
    if [[ -n "${yolo_pid}" ]]; then
      if [[ "${TB3_PERSON_SLAM_MODE:-mark_then_strip}" == "filtered" ]]; then
        echo "建图: SLAM 订阅 /scan_filtered（旧模式）。恢复「先完整建图再清人」: export TB3_PERSON_SLAM_MODE=mark_then_strip"
      else
        echo "建图: SLAM 订阅原始 /scan；人物 map 点写入 ~/.ros/tj_person_strip_regions.yaml，保存地图后: ros2 run human_yolo_seg strip_saved_map_person_free <map.yaml> <regions.yaml>。旧掩膜模式: TB3_PERSON_SLAM_MODE=filtered"
      fi
      echo "深度→/scan_rgbd 默认关闭，需要时: TB3_STACK_MODE=assist TB3_ASSIST_RGBD_BRIDGE=1"
    elif [[ "${TB3_STACK_MODE}" == "assist" ]]; then
      if [[ "${TURTLEBOT3_MODEL}" == "burger" ]]; then
        echo "建图: /scan；assist+burger 仅 RGB（无深度 SDF）；深度+RGB-D 桥请 TURTLEBOT3_MODEL=waffle 且 TB3_ASSIST_RGBD_BRIDGE=1；YOLO 掩膜需 human_yolo_seg 且 TB3_ASSIST_SCAN_FILTER=1"
      else
        echo "建图: /scan；assist+${TURTLEBOT3_MODEL} 含深度相机，RGB-D 桥需 TB3_ASSIST_RGBD_BRIDGE=1；YOLO 掩膜需 human_yolo_seg 且 TB3_ASSIST_SCAN_FILTER=1"
      fi
    else
      echo "建图: /scan；人物掩膜: 确保已 build human_yolo_seg（默认 TB3_ASSIST_SCAN_FILTER=1）；深度辅助: TB3_STACK_MODE=assist TB3_ASSIST_RGBD_BRIDGE=1"
    fi
  fi
}

do_stop() {
  cleanup_old
  echo "Stopped Gazebo (gzserver/gzclient), RViz2, robot_state_publisher, robot_description publisher, slam_toolbox, YOLO、scan 过滤、depth 桥接相关进程（若曾启动）"
}

do_check() {
  local status=0
  for topic in /clock /odom /tf /tf_static /robot_description /map /map_metadata; do
    check_topic "${topic}" || status=1
  done
  check_topic "/scan" || status=1
  check_echo "scan sample" ros2 topic echo /scan --once || status=1
  check_echo "odom sample" ros2 topic echo /odom --once || status=1
  check_tf || status=1

  if [[ "${TB3_STACK_MODE:-laser}" == "assist" && "${TB3_ASSIST_RGBD_BRIDGE:-0}" == "1" ]]; then
    local _ad _as
    _ad="$(topic_publisher_count "${RGBD_DEPTH_IMAGE_TOPIC}")"
    _as="$(topic_publisher_count /scan_rgbd)"
    if [[ "${_ad}" -gt 0 ]]; then
      echo "OK   assist ${RGBD_DEPTH_IMAGE_TOPIC} (publishers=${_ad})"
    else
      echo "WARN assist ${RGBD_DEPTH_IMAGE_TOPIC} 无发布者（assist 且 TB3_ASSIST_RGBD_BRIDGE=1？）"
    fi
    if [[ "${_as}" -gt 0 ]]; then
      echo "OK   assist /scan_rgbd (publishers=${_as})"
    else
      echo "WARN assist /scan_rgbd 无发布者"
    fi
  fi
  if [[ "${TB3_ASSIST_SCAN_FILTER:-1}" == "1" ]]; then
    local _ay _ya _sf
    _ay="$(topic_publisher_count /human_yolo/annotated_image)"
    _ya="$(topic_publisher_count /human_yolo/person_azimuth_ranges)"
    _sf="$(topic_publisher_count /scan_filtered)"
    if [[ "${_ay}" -gt 0 ]]; then
      echo "OK   /human_yolo/annotated_image (publishers=${_ay})"
    else
      echo "WARN /human_yolo/annotated_image 无发布者（未 build human_yolo_seg 或 TB3_ASSIST_SCAN_FILTER=0）"
    fi
    if [[ "${_ya}" -gt 0 ]]; then
      echo "OK   /human_yolo/person_azimuth_ranges (publishers=${_ya})"
    else
      echo "WARN /human_yolo/person_azimuth_ranges 无发布者"
    fi
    if [[ "${TB3_PERSON_SLAM_MODE:-mark_then_strip}" == "filtered" ]]; then
      if [[ "${_sf}" -gt 0 ]]; then
        echo "OK   /scan_filtered (publishers=${_sf})"
      else
        echo "WARN /scan_filtered 无发布者"
      fi
    else
      echo "OK   mark_then_strip：不检查 /scan_filtered（人物区域见 ~/.ros/tj_person_strip_regions.yaml）"
    fi
  fi

  if (( status == 0 )); then
    echo "Smoke test passed"
  else
    echo "Smoke test failed"
    exit 1
  fi
}

do_rviz() {
  export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
  export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-none}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"

  if [[ -f "${RVIZ_CONFIG_FILE}" ]]; then
    echo "Using existing RViz config: ${RVIZ_CONFIG_FILE}"
  else
    echo "RViz config not found, generating default: ${RVIZ_CONFIG_FILE}"
    generate_rviz_config
  fi

  echo "Starting RViz with software rendering for WSL compatibility"
  echo "Using config: ${RVIZ_CONFIG_FILE}"

  exec rviz2 -d "${RVIZ_CONFIG_FILE}" --ros-args -p use_sim_time:=true
}

do_logs() {
  local target="${2:-all}"

  case "${target}" in
    gzserver)
      tail -f "${LOG_DIR}/gzserver.log"
      ;;
    gzclient)
      tail -f "${LOG_DIR}/gzclient.log"
      ;;
    rviz|rviz2)
      tail -f "${LOG_DIR}/rviz2.log"
      ;;
    rsp|robot_state_publisher)
      tail -f "${LOG_DIR}/robot_state_publisher.log"
      ;;
    slam|slam_toolbox)
      tail -f "${LOG_DIR}/slam_backend.log"
      ;;
    robot_description)
      tail -f "${LOG_DIR}/robot_description.log"
      ;;
    spawn|spawn_entity)
      tail -f "${LOG_DIR}/spawn_entity.log"
      ;;
    obstacle|moving_obstacle)
      if [[ -f "${LOG_DIR}/moving_obstacle.log" ]]; then
        tail -f "${LOG_DIR}/moving_obstacle.log"
      else
        echo "No obstacle log yet: ${LOG_DIR}/moving_obstacle.log"
        echo "Start stack first: bash scripts/tb3_stack.sh start"
        exit 1
      fi
      ;;
    rgbd|rgbd_to_scan|depth)
      if [[ -f "${LOG_DIR}/rgbd_to_scan.log" ]]; then
        tail -f "${LOG_DIR}/rgbd_to_scan.log"
      else
        echo "No rgbd log: ${LOG_DIR}/rgbd_to_scan.log（需 TB3_STACK_MODE=assist 启动过）"
        exit 1
      fi
      ;;
    yolo|yolo_person)
      if [[ -f "${LOG_DIR}/yolo_person_seg.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_seg.log"
      elif [[ -f "${LOG_DIR}/yolo_person_seg_front.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_seg_front.log"
      else
        echo "No yolo log: ${LOG_DIR}/yolo_person_seg.log（或 360° 时 yolo_person_seg_front.log）"
        exit 1
      fi
      ;;
    strip_recorder|person_strip)
      if [[ -f "${LOG_DIR}/person_strip_recorder.log" ]]; then
        tail -f "${LOG_DIR}/person_strip_recorder.log"
      elif [[ -f "${LOG_DIR}/yolo_person_seg.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_seg.log"
      elif [[ -f "${LOG_DIR}/yolo_person_map_pipeline.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_map_pipeline.log"
      else
        echo "无 strip 相关日志（已与 YOLO 合并到 yolo_person_seg.log / yolo_person_map_pipeline.log）"
        exit 1
      fi
      ;;
    scan_filter|scan_person)
      if [[ -f "${LOG_DIR}/scan_person_filter.log" ]]; then
        tail -f "${LOG_DIR}/scan_person_filter.log"
      elif [[ -f "${LOG_DIR}/yolo_person_seg.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_seg.log"
      elif [[ -f "${LOG_DIR}/yolo_person_map_pipeline.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_map_pipeline.log"
      else
        echo "无 scan_person_filter 独立日志（已合并到上述 yolo_person*.log）"
        exit 1
      fi
      ;;
    scan_map_colored|scan_colored)
      if [[ -f "${LOG_DIR}/scan_map_colored_cloud.log" ]]; then
        tail -f "${LOG_DIR}/scan_map_colored_cloud.log"
      elif [[ -f "${LOG_DIR}/yolo_person_seg.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_seg.log"
      elif [[ -f "${LOG_DIR}/yolo_person_map_pipeline.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_map_pipeline.log"
      else
        echo "无 scan_map_colored 独立日志（默认未起该节点；需 TB3_ENABLE_SCAN_MAP_COLORED=1，或看 yolo_person*.log）"
        exit 1
      fi
      ;;
    yolo_map_pipeline|map_pipeline)
      if [[ -f "${LOG_DIR}/yolo_person_map_pipeline.log" ]]; then
        tail -f "${LOG_DIR}/yolo_person_map_pipeline.log"
      else
        echo "无 yolo_person_map_pipeline.log（已弃用双路 YOLO；人物链日志见 yolo_person_seg.log）"
        exit 1
      fi
      ;;
    all)
      echo "Available logs:"
      ls -1 "${LOG_DIR}"
      ;;
    *)
      echo "Unknown log target: ${target}"
      echo "Use one of: all, gzserver, gzclient, rviz, rsp, slam, robot_description, spawn, obstacle, rgbd, yolo, yolo_map_pipeline, strip_recorder, scan_filter, scan_map_colored"
      exit 1
      ;;
  esac
}

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tb3_stack.sh start
  bash scripts/tb3_stack.sh stop
  bash scripts/tb3_stack.sh check
  bash scripts/tb3_stack.sh rviz
  bash scripts/tb3_stack.sh logs [all|gzserver|gzclient|rviz|rsp|slam|robot_description|spawn|obstacle|rgbd|yolo|yolo_map_pipeline|strip_recorder|scan_filter|scan_map_colored]

Environment:
  TB3_STACK_MODE=laser|assist  laser=默认 burger；assist 未指定模型时=burger（RGB、无深度注入）；需深度请 TURTLEBOT3_MODEL=waffle|waffle_pi
  TB3_ASSIST_SCAN_FILTER=0|1  默认 1：若已 build human_yolo_seg，则 ros2 launch yolo_person_seg（YOLO + strip|filter + 方位角 Marker；整帧 map 上色见 TB3_ENABLE_SCAN_MAP_COLORED）
  TB3_ENABLE_SCAN_MAP_COLORED=0|1  默认 0：为 1 时额外起 scan_map_colored_cloud（整帧激光投 map；filtered 且无 strip 时便于看人向）
  TB3_PERSON_SLAM_MODE=mark_then_strip|filtered  默认 mark_then_strip：SLAM 用 /scan + person_strip_recorder，事后 strip 地图；filtered=旧行为 /scan_filtered 建图
  TB3_PERSON_AZIMUTH_MODE=linear_fov|tf_geometry  默认 linear_fov：人物方位角粗略法；tf_geometry=内参+TF+地面（传给 yolo_person_seg.launch person_azimuth_mode）
  TB3_ASSIST_RGBD_BRIDGE=0|1  仅 TB3_STACK_MODE=assist 时有效；默认 0 不起 rgbd_to_scan；1 时起 depth→/scan_rgbd
  RGBD_DEPTH_IMAGE_TOPIC=/tb3_depth_only/depth/image_raw  assist 且 TB3_ASSIST_RGBD_BRIDGE=1 时传给 rgbd_to_scan
  RGBD_DEPTH_CAMERA_INFO_TOPIC=/tb3_depth_only/depth/camera_info
  YOLO_IMAGE_TOPIC=/camera/image_raw
  YOLO_CAMERA_INFO_TOPIC=/camera/camera_info
  TB3_YOLO_360=1  已弃用（burger 无后摄）；设 1 仅告警，仍走单路 YOLO
  YOLO_DEVICE=auto|cpu|cuda:0  YOLO 推理设备（默认 auto：检测到 CUDA 则用 GPU）
  TB3_NO_GUI=1   Skip gzclient and RViz2 on start (headless / CI)
  TB3_HEADLESS=1 Same as TB3_NO_GUI=1 (alias for no GUI)
  TB3_START_BENCH=0|1  默认 1：start 时打印 [计时] 分步与总耗时（秒）；0 关闭
  TB3_WAIT_POLL_SEC=0.5  等待 /spawn_entity、/tf_static、/map 等时的轮询间隔（秒）；可略调小以加快就绪检测（略增 CPU）
  TURTLEBOT3_MODEL=burger|waffle|waffle_pi  laser 与 assist 未指定时均默认 burger；需仿真深度 SDF 时显式 waffle 或 waffle_pi
  （已弃用）SLAM_SENSOR=rgbd 等价于 TB3_STACK_MODE=assist（主建图仍为激光）
  TB3_GAZEBO_HARDWARE_GL=1  跳过 LIBGL_ALWAYS_SOFTWARE（真机 GPU；WSL 勿设，避免 RenderEngine 崩溃）
  RVIZ_CONFIG_FILE=<path>  RViz config path (default: robot_bringup/config/test1.rviz)
  WORLD_FILE=<path>   Gazebo world file override (default: robot_bringup/world/test1.world)
  MAP_PGM_FILE=<path>  Static map image path for map server or nav stack (default: robot_bringup/maps/test1.pgm)
  MAP_YAML_FILE=<path> Static map yaml path (default: robot_bringup/maps/test1.yaml)
  ROBOT_START_X=-2.0  Robot spawn x in Gazebo world
  ROBOT_START_Y=-1.2  Robot spawn y in Gazebo world
  ROBOT_START_Z=0.1   Robot spawn z in Gazebo world
  ROBOT_START_YAW=0.0 Robot spawn yaw in Gazebo world
  OBSTACLE_TRAJECTORY=[line|circle|figure8|lissajous|patrol]
  OBSTACLE_FALLBACK_RATE_HZ=5.0  Update rate used only in gz fallback mode
EOF
}

case "${1:-}" in
  start)
    do_start
    ;;
  stop)
    do_stop
    ;;
  check)
    do_check
    ;;
  rviz)
    do_rviz
    ;;
  logs)
    do_logs "$@"
    ;;
  *)
    usage
    exit 1
    ;;
esac
