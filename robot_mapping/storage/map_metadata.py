"""Shared map metadata structures."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class MapMetadata:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
