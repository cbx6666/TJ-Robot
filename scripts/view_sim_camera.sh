#!/usr/bin/env bash
# 独立窗口查看 YOLO 标注图（默认 /yolo_objects/annotated_image）；RViz Image 异常时备用。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"
require_ros
source_workspace_if_available

TOPIC="${1:-/yolo_objects/annotated_image}"
echo "Opening rqt_image_view on ${TOPIC}"
exec ros2 run rqt_image_view rqt_image_view "${TOPIC}"
