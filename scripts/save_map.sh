#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs
require_command ros2

kind="${1:-baseline_raw}"
stamp="$(date +%Y%m%d_%H%M%S)"
case "${kind}" in
  baseline_raw|semantic_pre_strip|semantic_post_strip|semantic_overlays) ;;
  raw) kind="baseline_raw" ;;
  before_strip) kind="semantic_pre_strip" ;;
  after_strip) kind="semantic_post_strip" ;;
  semantic) kind="semantic_overlays" ;;
  *)
    echo "Usage: bash scripts/save_map.sh [baseline_raw|semantic_pre_strip|semantic_post_strip|semantic_overlays]" >&2
    echo "兼容旧值: raw|before_strip|after_strip|semantic" >&2
    exit 2
    ;;
esac

prefix="${PROJECT_ROOT}/data/maps/${kind}/map_${stamp}"
echo "Saving /map to ${prefix}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${prefix}"
