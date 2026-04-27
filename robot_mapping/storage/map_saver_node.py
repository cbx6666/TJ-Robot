"""Map saver helpers."""

from __future__ import annotations

from pathlib import Path


def default_map_prefix(kind: str = "raw") -> Path:
    return Path("data") / "maps" / kind / "map"
