"""Shared map-save helpers for navigation nodes."""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path


def resolve_map_save_directory(raw_directory: str) -> Path:
    """Resolve save directory from explicit param, env, then cwd fallback."""
    raw = (raw_directory or "").strip()
    if raw:
        path = Path(raw).expanduser()
    else:
        env = (os.environ.get("TJ_ROBOT_SAVED_MAPS_DIR") or "").strip()
        if env:
            path = Path(env).expanduser()
        else:
            path = Path.cwd() / "saved_maps"
    path = path.resolve()
    path.mkdir(parents=True, exist_ok=True)
    return path


def resolve_map_basename(raw_basename: str) -> str:
    """Ensure basename is non-empty and file-name-safe."""
    base = Path((raw_basename or "").strip()).name
    if base:
        return base
    return datetime.now().strftime("coverage_map_%Y%m%d_%H%M%S")
