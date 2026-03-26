#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

# ROS 2 Humble 的包在 packages.ros.org，不在 Ubuntu 默认源里；未添加源则 apt 找不到 ros-humble-*。
if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  ${SUDO} apt update
  ${SUDO} apt install -y software-properties-common curl
  ${SUDO} add-apt-repository universe -y
  ${SUDO} curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  UBUNTU_CODENAME="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")"
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
    | ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

${SUDO} apt update
${SUDO} apt install -y \
  ros-humble-desktop \
  gazebo \
  python3-colcon-common-extensions \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-description \
  ros-humble-turtlebot3-gazebo \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-xacro

if ! grep -Fq "source /opt/ros/humble/setup.bash" "${HOME}/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "${HOME}/.bashrc"
fi
if ! grep -Fq "export TURTLEBOT3_MODEL=burger" "${HOME}/.bashrc"; then
  echo "export TURTLEBOT3_MODEL=burger" >> "${HOME}/.bashrc"
fi

set +u
source /opt/ros/humble/setup.bash
set +u
export TURTLEBOT3_MODEL=burger

status=0
for cmd in ros2 rviz2 gzserver colcon; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    echo "OK   command ${cmd}"
  else
    echo "FAIL command ${cmd}"
    status=1
  fi
done

for pkg in gazebo_ros slam_toolbox turtlebot3_gazebo turtlebot3_description nav2_bringup xacro; do
  if ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "OK   package ${pkg}"
  else
    echo "FAIL package ${pkg}"
    status=1
  fi
done

if python3 - <<'PY'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
PY
then
  echo "OK   python imports rclpy geometry_msgs nav_msgs"
else
  echo "FAIL python imports rclpy geometry_msgs nav_msgs"
  status=1
fi

echo "Environment setup complete"
echo "Open a new shell or run: source /opt/ros/humble/setup.bash"
echo "ROS environment has been persisted to ~/.bashrc"

exit "${status}"
