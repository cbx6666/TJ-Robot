from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class TaskIntent(str, Enum):
    NAVIGATE_TO_ROOM = "navigate_to_room"
    SEARCH_OBJECT = "search_object"
    BUILD_MAP = "build_map"
    PATROL = "patrol"
    STOP = "stop"
    UNKNOWN = "unknown"


@dataclass(slots=True)
class RobotTask:
    task_id: str
    intent: TaskIntent
    target_room: str | None = None
    target_object: str | None = None
    required_modules: list[str] = field(default_factory=list)
    parameters: dict[str, Any] = field(default_factory=dict)
