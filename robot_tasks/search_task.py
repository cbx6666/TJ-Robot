"""Indoor object search task skeleton."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class SearchTaskSpec:
    target_object: str
    target_room: str | None = None
    search_points: list[tuple[float, float, float]] = field(default_factory=list)
