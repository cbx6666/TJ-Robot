#!/usr/bin/env bash
# 手动清理本机残留的 Nav2（多 map_server 时 RViz 报 No map received）。用法: bash scripts/kill_nav2.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

if ! command -v killall >/dev/null 2>&1; then
  echo "WARNING: 未找到 killall（psmisc）。若清理不彻底请: sudo apt install -y psmisc" >&2
fi
echo "Killing Nav2-related processes (launch + nav2_* nodes) ..."
tj_kill_nav2_background_launch
echo "Remaining map_server-related PIDs (should be empty):"
ps -eo pid,comm,args 2>/dev/null | grep -E '[m]ap_server|[n]avigation\.launch|lifecycle_manager' || true
echo "Done. Run: ros2 topic info /map -v  (expect Publisher count 0 until you start Nav2 again)"
