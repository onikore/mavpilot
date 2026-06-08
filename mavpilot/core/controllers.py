"""Lateral controllers for precision_land.

Each controller maps (err_x, err_y, dt) → (step_x, step_y) in metres.
The step is fed into the existing clamping path in PrecisionLand.update —
safety bounds and floor/handoff logic are not the controller's concern.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
import math


class LateralController(ABC):
    """Abstract lateral controller: NED error (m) → position step (m)."""

    @abstractmethod
    def update(self, err_x: float, err_y: float, dt: float) -> tuple[float, float]:
        """Return commanded position delta (step_x, step_y) in metres."""

    def reset(self) -> None:
        """Clear integrator / observer state. Called once before each landing."""


class PController(LateralController):
    """Proportional controller — reproduces the original lateral_p_gain behaviour."""

    def __init__(self, kp: float = 0.7) -> None:
        self._kp = kp

    def update(self, err_x: float, err_y: float, dt: float) -> tuple[float, float]:
        return self._kp * err_x, self._kp * err_y
