"""Map loader placeholder."""

from pathlib import Path


def resolve_map_yaml(path: str | Path) -> Path:
    return Path(path).expanduser().resolve()
