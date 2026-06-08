import pytest
from mavpilot.core.controllers import PController


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
