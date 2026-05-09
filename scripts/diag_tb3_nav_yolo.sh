#!/usr/bin/env bash
# 快速看 /map、相机、YOLO 与最近日志（需在已 source 工作区的终端执行，或与 run_* 同一 ROS_DOMAIN_ID）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

LOG="${TB3_LOG_DIR:-${PROJECT_ROOT}/data/logs/simulation}"

require_ros
source_workspace_if_available

echo "=== ros2 topic info (no -v) ==="
for t in /map /clock /scan /camera/image_raw /yolo_objects/annotated_image; do
  echo "--- ${t} ---"
  ros2 topic info "${t}" 2>&1 || true
done

echo ""
echo "=== tail nav2.launch.log (${LOG}) ==="
tail -50 "${LOG}/nav2.launch.log" 2>&1 || echo "(missing)"

echo ""
echo "=== tail yolo_object_seg.log ==="
tail -35 "${LOG}/yolo_object_seg.log" 2>&1 || echo "(missing)"
