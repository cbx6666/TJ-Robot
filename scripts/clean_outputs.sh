#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

targets=(
  "${PROJECT_ROOT}/data/maps/baseline_raw"
  "${PROJECT_ROOT}/data/maps/semantic_pre_strip"
  "${PROJECT_ROOT}/data/maps/semantic_post_strip"
  "${PROJECT_ROOT}/data/maps/semantic_overlays"
  "${PROJECT_ROOT}/data/logs"
  "${PROJECT_ROOT}/data/results"
)

for dir in "${targets[@]}"; do
  mkdir -p "${dir}"
  find "${dir}" -mindepth 1 ! -name ".gitkeep" -exec rm -rf {} +
done

echo "Cleaned generated outputs under data/."
