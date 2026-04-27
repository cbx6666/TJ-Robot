"""Room-level navigation helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class RoomGoal:
    room: str
    x: float
    y: float
    yaw: float = 0.0
