#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs
require_command ros2

kind="${1:-raw}"
stamp="$(date +%Y%m%d_%H%M%S)"
case "${kind}" in
  raw|before_strip|after_strip|semantic) ;;
  *)
    echo "Usage: bash scripts/save_map.sh [raw|before_strip|after_strip|semantic]" >&2
    exit 2
    ;;
esac

prefix="${PROJECT_ROOT}/data/maps/${kind}/map_${stamp}"
echo "Saving /map to ${prefix}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli -f "${prefix}"
