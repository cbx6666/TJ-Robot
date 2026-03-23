#!/usr/bin/env bash

set -euo pipefail

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

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/opt/ros/humble/share/turtlebot3_description}"
export TB3_LOG_DIR="${TB3_LOG_DIR:-/tmp/tb3_stack}"

WORLD_FILE="/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world"
MODEL_FILE="/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_${TURTLEBOT3_MODEL}/model.sdf"
URDF_FILE="/tmp/tb3_${TURTLEBOT3_MODEL}.urdf"
LOG_DIR="${TB3_LOG_DIR}"
RVIZ_CONFIG_FILE="${LOG_DIR}/tb3_auto.rviz"

mkdir -p "${LOG_DIR}"

cleanup_old() {
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 -f gazebo 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f robot_state_publisher 2>/dev/null || true
  pkill -9 -f '/robot_description std_msgs/msg/String' 2>/dev/null || true
  pkill -9 -f slam_toolbox 2>/dev/null || true
  pkill -9 -f async_slam_toolbox_node 2>/dev/null || true
}

wait_for_service() {
  local name="$1"
  local timeout_s="${2:-20}"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 service list | grep -qx "${name}"; then
      return 0
    fi
    sleep 1
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
    sleep 1
  done
  return 1
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
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
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
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
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
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
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
      Target Frame: map
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
  echo "[1/9] Cleaning old processes"
  cleanup_old

  echo "[2/9] Starting gzserver"
  setsid gzserver "${WORLD_FILE}" --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >"${LOG_DIR}/gzserver.log" 2>&1 < /dev/null &
  local gz_pid=$!

  if ! wait_for_service "/spawn_entity" 20; then
    echo "gzserver did not expose /spawn_entity"
    tail -n 80 "${LOG_DIR}/gzserver.log" || true
    exit 1
  fi

  echo "[3/9] Expanding URDF and starting robot_state_publisher"
  ros2 run xacro xacro "/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_${TURTLEBOT3_MODEL}.urdf" >"${URDF_FILE}"
  setsid ros2 run robot_state_publisher robot_state_publisher "${URDF_FILE}" --ros-args -p use_sim_time:=true \
    >"${LOG_DIR}/robot_state_publisher.log" 2>&1 < /dev/null &
  local rsp_pid=$!

  if ! wait_for_topic "/tf_static" 15; then
    echo "robot_state_publisher did not publish /tf_static"
    tail -n 80 "${LOG_DIR}/robot_state_publisher.log" || true
    exit 1
  fi

  echo "[4/9] Publishing /robot_description"
  publish_robot_description

  echo "[5/9] Spawning TurtleBot3 entity"
  ros2 run gazebo_ros spawn_entity.py \
    -entity "${TURTLEBOT3_MODEL}" \
    -file "${MODEL_FILE}" \
    -x -2.0 -y -0.5 -z 0.1 \
    >"${LOG_DIR}/spawn_entity.log" 2>&1

  echo "[6/9] Starting slam_toolbox"
  setsid ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    base_frame:=base_footprint \
    odom_frame:=odom \
    map_frame:=map \
    >"${LOG_DIR}/slam_toolbox.log" 2>&1 < /dev/null &
  local slam_pid=$!

  if ! wait_for_topic "/map" 25; then
    echo "slam_toolbox did not publish /map"
    tail -n 120 "${LOG_DIR}/slam_toolbox.log" || true
    exit 1
  fi

  echo "[7/9] Generating RViz config"
  generate_rviz_config

  echo "[8/9] Verifying key topics"
  for topic in /clock /scan /odom /tf /tf_static /robot_description /map /map_metadata; do
    if ros2 topic list | grep -qx "${topic}"; then
      echo "  OK  ${topic}"
    else
      echo "  MISS ${topic}"
    fi
  done

  local gzclient_pid=""
  local rviz_pid=""
  if [[ "${TB3_NO_GUI:-0}" == "1" ]]; then
    echo "[9/9] Skipping gzclient and RViz2 (TB3_NO_GUI=1)"
  else
    echo "[9/9] Starting gzclient and RViz2"
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

  echo
  echo "PIDs:"
  echo "  gzserver: ${gz_pid}"
  echo "  robot_state_publisher: ${rsp_pid}"
  echo "  slam_toolbox: ${slam_pid}"
  if [[ -n "${gzclient_pid}" ]]; then
    echo "  gzclient: ${gzclient_pid}"
  fi
  if [[ -n "${rviz_pid}" ]]; then
    echo "  rviz2: ${rviz_pid}"
  fi
  echo
  echo "Logs: ${LOG_DIR}"
  echo "RViz config: ${RVIZ_CONFIG_FILE}"
  if [[ "${TB3_NO_GUI:-0}" != "1" ]]; then
    echo
    echo "Keyboard teleop (new terminal):"
    echo "  source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=\${TURTLEBOT3_MODEL:-burger}"
    echo "  ros2 run turtlebot3_teleop teleop_keyboard"
  fi
}

do_stop() {
  cleanup_old
  echo "Stopped Gazebo (gzserver/gzclient), RViz2, robot_state_publisher, robot_description publisher, and slam_toolbox"
}

do_check() {
  local status=0
  for topic in /clock /scan /odom /tf /tf_static /robot_description /map /map_metadata; do
    check_topic "${topic}" || status=1
  done
  check_echo "scan sample" ros2 topic echo /scan --once || status=1
  check_echo "odom sample" ros2 topic echo /odom --once || status=1
  check_tf || status=1

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

  generate_rviz_config

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
      tail -f "${LOG_DIR}/slam_toolbox.log"
      ;;
    robot_description)
      tail -f "${LOG_DIR}/robot_description.log"
      ;;
    spawn|spawn_entity)
      tail -f "${LOG_DIR}/spawn_entity.log"
      ;;
    all)
      echo "Available logs:"
      ls -1 "${LOG_DIR}"
      ;;
    *)
      echo "Unknown log target: ${target}"
      echo "Use one of: all, gzserver, gzclient, rviz, rsp, slam, robot_description, spawn"
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
  bash scripts/tb3_stack.sh logs [all|gzserver|gzclient|rviz|rsp|slam|robot_description|spawn]

Environment:
  TB3_NO_GUI=1   Skip gzclient and RViz2 on start (headless / CI)
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
