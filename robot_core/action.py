from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class ActionType(str, Enum):
    START_SIMULATION = "start_simulation"
    BUILD_MAP = "build_map"
    STRIP_MAP = "strip_map"
    NAVIGATE_TO_POSE = "navigate_to_pose"
    NAVIGATE_TO_ROOM = "navigate_to_room"
    SEARCH_OBJECT = "search_object"
    SPEAK = "speak"
    STOP = "stop"


@dataclass(slots=True)
class RobotAction:
    action_type: ActionType
    parameters: dict[str, Any] = field(default_factory=dict)
