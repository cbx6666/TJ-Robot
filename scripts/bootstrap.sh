#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

bash "${SCRIPT_DIR}/setup_env.sh"
bash "${SCRIPT_DIR}/tb3_stack.sh" start
bash "${SCRIPT_DIR}/tb3_stack.sh" check

echo "Bootstrap complete"
echo "Next (new terminal, keyboard control):"
echo "  source /opt/ros/humble/setup.bash && export TURTLEBOT3_MODEL=waffle"
echo "  ros2 run turtlebot3_teleop teleop_keyboard"
