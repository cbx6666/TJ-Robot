from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class EventType(str, Enum):
    SPEECH_TEXT = "speech_text"
    INTENT_PARSED = "intent_parsed"
    PERSON_DETECTED = "person_detected"
    OBJECT_DETECTED = "object_detected"
    MAP_SAVED = "map_saved"
    STRIP_DONE = "strip_done"
    NAVIGATION_DONE = "navigation_done"
    TASK_DONE = "task_done"
    ERROR = "error"


@dataclass(slots=True)
class RobotEvent:
    event_type: EventType
    source: str
    payload: dict[str, Any] = field(default_factory=dict)
    stamp: float | None = None
