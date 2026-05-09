# shellcheck shell=bash
# 由 scripts/tb3_stack.sh source；依赖: SCRIPT_DIR、TB3_LOG_DIR、TB3_ENABLE_*、RVIZ_CONFIG_FILE。

cleanup_old() {
  pkill -9 -f scan_rviz_relay 2>/dev/null || true
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  pkill -9 -x rviz2 2>/dev/null || true
  pkill -9 -f "ros2 launch human_yolo_seg yolo_object_seg.launch.py" 2>/dev/null || true
  pkill -9 -f "ros2 launch human_yolo_seg yolo_person_seg.launch.py" 2>/dev/null || true
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
}

# 在仿真主体（YOLO/深度桥）就绪前先起 RViz，缩短「空白等待」体感。
launch_tb3_visualization_gui() {
  if [[ "${TB3_ENABLE_GZCLIENT}" != "1" && "${TB3_ENABLE_RVIZ}" != "1" ]]; then
    echo "Skip GUI launch (TB3_ENABLE_GZCLIENT=0, TB3_ENABLE_RVIZ=0)"
    return 0
  fi
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
}

publish_robot_description() {
  local urdf_file="$1"
  local pub_py="${SCRIPT_DIR}/../ros_ws/src/robot_bringup/scripts/publish_robot_description_topic.py"
  if [[ ! -f "${pub_py}" ]]; then
    echo "ERROR: 未找到 ${pub_py}" >&2
    exit 1
  fi
  setsid python3 "${pub_py}" "${urdf_file}" --hz "${TB3_ROBOT_DESCRIPTION_TOPIC_HZ:-5}" \
    >"${TB3_LOG_DIR}/robot_description.log" 2>&1 < /dev/null &
}
