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


class _PIDAxis:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        windup_limit: float,
        alpha: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup_limit = windup_limit
        self.alpha = alpha
        self._integral = 0.0
        self._e_filtered = 0.0
        self._e_prev = 0.0

    def update(self, e: float, dt: float) -> float:
        self._integral = max(
            -self.windup_limit,
            min(self.windup_limit, self._integral + e * dt),
        )
        self._e_filtered = self.alpha * e + (1.0 - self.alpha) * self._e_filtered
        d = (self._e_filtered - self._e_prev) / dt if dt > 0 else 0.0
        self._e_prev = self._e_filtered
        return self.kp * e + self.ki * self._integral + self.kd * d

    def reset(self) -> None:
        self._integral = 0.0
        self._e_filtered = 0.0
        self._e_prev = 0.0


class PIDController(LateralController):
    """Discrete PID with integral anti-windup and first-order derivative filter."""

    def __init__(
        self,
        kp: float = 0.7,
        ki: float = 0.05,
        kd: float = 0.1,
        windup_limit: float = 2.0,
        derivative_alpha: float = 0.5,
    ) -> None:
        self._x = _PIDAxis(kp, ki, kd, windup_limit, derivative_alpha)
        self._y = _PIDAxis(kp, ki, kd, windup_limit, derivative_alpha)

    def update(self, err_x: float, err_y: float, dt: float) -> tuple[float, float]:
        return self._x.update(err_x, dt), self._y.update(err_y, dt)

    def reset(self) -> None:
        self._x.reset()
        self._y.reset()
