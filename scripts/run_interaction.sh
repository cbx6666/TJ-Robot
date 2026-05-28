#!/usr/bin/env bash
# 仅 voice_gateway + llm_router（无 task_manager）。仿真全链路请用 run_voice_stack.sh。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
# shellcheck source=lib/wsl_pulse_env.sh
source "${SCRIPT_DIR}/lib/wsl_pulse_env.sh"

require_ros
source_workspace_if_available

USE_SIM="${TJ_INTERACTION_USE_SIM_TIME:-false}"
echo "interaction only (no task_manager). 仿真+巡检请用: bash scripts/run_voice_stack.sh"
exec ros2 launch robot_bringup interaction.launch.py \
  "use_sim_time:=${USE_SIM}" \
  asr_backend:=whisper_mic \
  "$@"
