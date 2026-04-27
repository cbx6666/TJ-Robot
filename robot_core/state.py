from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class RobotMode(str, Enum):
    IDLE = "idle"
    SIMULATION = "simulation"
    MAPPING = "mapping"
    NAVIGATION = "navigation"
    SEARCH = "search"
    INTERACTION = "interaction"
    ERROR = "error"


@dataclass(slots=True)
class RobotState:
    mode: RobotMode = RobotMode.IDLE
    current_task_id: str | None = None
    last_event: str | None = None
    metadata: dict[str, Any] = field(default_factory=dict)
