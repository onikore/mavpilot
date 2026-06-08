import pytest
from mavpilot.core.controllers import PController, PIDController


def test_p_proportionality():
    ctrl = PController(kp=0.5)
    sx, sy = ctrl.update(err_x=1.0, err_y=2.0, dt=0.1)
    assert sx == pytest.approx(0.5)
    assert sy == pytest.approx(1.0)


def test_p_reset_is_noop():
    ctrl = PController(kp=0.5)
    ctrl.reset()
    sx, sy = ctrl.update(1.0, 0.0, dt=0.1)
    assert sx == pytest.approx(0.5)


def test_p_zero_error():
    ctrl = PController(kp=2.0)
    sx, sy = ctrl.update(0.0, 0.0, dt=0.1)
    assert sx == 0.0
    assert sy == 0.0


def test_pid_integral_accumulates():
    # ki=1, kp=0, kd=0 → output = integral = Σ(e * dt)
    ctrl = PIDController(kp=0.0, ki=1.0, kd=0.0, windup_limit=100.0, derivative_alpha=1.0)
    dt = 0.1
    for _ in range(5):
        ctrl.update(1.0, 0.0, dt)
    sx, _ = ctrl.update(1.0, 0.0, dt)
    # After 6 calls: integral = 6 * 1.0 * 0.1 = 0.6, ki * integral = 0.6
    assert sx == pytest.approx(0.6, rel=1e-6)


def test_pid_antiwindup_clamps_integral():
    ctrl = PIDController(kp=0.0, ki=1.0, kd=0.0, windup_limit=0.3, derivative_alpha=1.0)
    dt = 0.1
    for _ in range(100):
        ctrl.update(1.0, 0.0, dt)
    sx, _ = ctrl.update(1.0, 0.0, dt)
    # integral clamped at 0.3, output = 1.0 * 0.3 = 0.3
    assert sx == pytest.approx(0.3, rel=1e-6)


def test_pid_derivative_on_error():
    # kp=0, ki=0, kd=1, derivative_alpha=1 → output = (e - e_prev)/dt
    ctrl = PIDController(kp=0.0, ki=0.0, kd=1.0, windup_limit=100.0, derivative_alpha=1.0)
    dt = 0.1
    ctrl.update(0.5, 0.0, dt)       # first call: e_prev=0 → d=(0.5-0)/0.1=5
    sx, _ = ctrl.update(1.0, 0.0, dt)  # e=1, e_prev=0.5 → d=(1-0.5)/0.1=5
    assert sx == pytest.approx(5.0, rel=1e-6)


def test_pid_reset_clears_state():
    ctrl = PIDController(kp=0.0, ki=1.0, kd=0.0, windup_limit=100.0, derivative_alpha=1.0)
    dt = 0.1
    for _ in range(10):
        ctrl.update(1.0, 0.0, dt)
    ctrl.reset()
    # After reset: integral = 0, so output = ki * e * dt = 1*1*0.1 = 0.1
    sx, _ = ctrl.update(1.0, 0.0, dt)
    assert sx == pytest.approx(0.1, rel=1e-6)
