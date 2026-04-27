"""Person region store helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class PersonRegion:
    x: float
    y: float
    radius: float = 0.08
