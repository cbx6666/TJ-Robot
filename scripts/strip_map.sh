#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

require_ros
source_workspace_if_available
prepare_output_dirs

if [[ $# -lt 2 ]]; then
  echo "Usage: bash scripts/strip_map.sh <before_strip_map.yaml> <person_regions.yaml> [output_prefix]" >&2
  exit 2
fi

map_yaml="$1"
regions_yaml="$2"
if [[ $# -ge 3 ]]; then
  output_prefix="$3"
else
  stem="$(basename "${map_yaml}")"
  stem="${stem%.yaml}"
  output_prefix="${PROJECT_ROOT}/data/maps/after_strip/${stem}_person_free"
fi

echo "Stripping person regions:"
echo "  map:     ${map_yaml}"
echo "  regions: ${regions_yaml}"
echo "  output:  ${output_prefix}.yaml/.pgm"
ros2 run human_yolo_seg strip_saved_map_person_free "${map_yaml}" "${regions_yaml}" -o "${output_prefix}"
