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


def _gl_weights(alpha: float, N: int) -> list[float]:
    """Grünwald-Letnikov coefficients for fractional operator of order alpha.

    For fractional integral of order λ call with alpha=-λ.
    For fractional derivative of order μ call with alpha=μ.
    w[0]=1, w[k] = w[k-1] * (k-1-alpha)/k
    """
    w = [1.0]
    for k in range(1, N):
        w.append(w[-1] * (k - 1.0 - alpha) / k)
    return w


class _FOPIDAxis:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        lambda_order: float,
        mu_order: float,
        N: int,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._lam = lambda_order
        self._mu = mu_order
        self._N = N
        self._wi = _gl_weights(-lambda_order, N)   # integral weights
        self._wd = _gl_weights(mu_order, N)         # derivative weights
        self._buf: deque[float] = deque([0.0] * N, maxlen=N)

    def update(self, e: float, dt: float) -> float:
        self._buf.appendleft(e)   # buf[0] = e[n], buf[1] = e[n-1], ...
        i_frac = (dt ** self._lam) * sum(self._wi[j] * self._buf[j] for j in range(self._N))
        d_frac = (dt ** (-self._mu)) * sum(self._wd[j] * self._buf[j] for j in range(self._N))
        return self.kp * e + self.ki * i_frac + self.kd * d_frac

    def reset(self) -> None:
        self._buf = deque([0.0] * self._N, maxlen=self._N)


class FOPIDController(LateralController):
    """Fractional-order PIλDμ via truncated Grünwald-Letnikov approximation.

    Reduces to standard PID when lambda_order=1.0 and mu_order=1.0.
    """

    def __init__(
        self,
        kp: float = 0.7,
        ki: float = 0.05,
        kd: float = 0.1,
        lambda_order: float = 0.8,
        mu_order: float = 0.9,
        N: int = 20,
    ) -> None:
        self._x = _FOPIDAxis(kp, ki, kd, lambda_order, mu_order, N)
        self._y = _FOPIDAxis(kp, ki, kd, lambda_order, mu_order, N)

    def update(self, err_x: float, err_y: float, dt: float) -> tuple[float, float]:
        return self._x.update(err_x, dt), self._y.update(err_y, dt)

    def reset(self) -> None:
        self._x.reset()
        self._y.reset()
