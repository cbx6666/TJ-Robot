#!/usr/bin/env bash
# 停栈后仍卡顿时：强杀仿真 + Nav2 + ros2 daemon（WSL 内执行）
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

echo "=== kill_simulation_stack: 语音栈 + tb3_stack + Nav2 + daemon ==="
# tb3_stack stop 不含 voice_gateway；不杀则 WSL Pulse 常被占死，sounddevice 会卡在 Pa_Initialize()
if [[ -f "${SCRIPT_DIR}/lib/full_system_env.sh" ]]; then
  # shellcheck source=lib/full_system_env.sh
  source "${SCRIPT_DIR}/lib/full_system_env.sh"
  full_system_stop_voice_stack
fi
pkill -9 -f voice_gateway_node 2>/dev/null || true
pkill -9 -f sim_speech_gui_node 2>/dev/null || true
pkill -9 -f 'task_pipeline\.launch\.py' 2>/dev/null || true
pkill -9 -f 'interaction\.launch\.py' 2>/dev/null || true
sleep 0.5
bash "${SCRIPT_DIR}/tb3_stack.sh" stop 2>/dev/null || true
tj_kill_nav2_background_launch
pkill -9 -f tj_nav2_trigger_navigation_manager_startup_after_map_server 2>/dev/null || true
pkill -9 -f component_container 2>/dev/null || true
pkill -9 -f spawn_entity.py 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
echo "Stopping ros2 daemon (若此处卡住 >5s，另开终端执行: ros2 daemon stop) ..."
timeout 5 ros2 daemon stop 2>/dev/null || true

echo ""
echo "Remaining (should be empty or only grep itself):"
ps aux 2>/dev/null | grep -E '[g]zserver|[g]zclient|[r]viz2|[n]av2|[y]olo|component_container|spawn_entity|map_server|amcl' || true
echo ""
echo "若 Gazebo/RViz 窗口仍无响应：在 Windows 任务管理器结束 gzclient / rviz2，或关闭 WSLg 窗口。"
echo "若整机仍慢：PowerShell 管理员执行: wsl --shutdown  （会关闭整个 WSL）"
echo "若 python3 -c 'import sounddevice' 仍卡住: bash scripts/recover_wsl_audio.sh"
echo "Done."
