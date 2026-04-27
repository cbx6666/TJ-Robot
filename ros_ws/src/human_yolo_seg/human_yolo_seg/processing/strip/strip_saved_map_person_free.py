"""Layered entry for person-region map stripping."""

from human_yolo_seg.tools.strip_saved_map import (
    _load_map_meta,
    _strip_regions,
    _world_to_uv,
    main,
)

__all__ = ["main", "_load_map_meta", "_strip_regions", "_world_to_uv"]
