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


from mavpilot.core.controllers import FOPIDController


def test_fopid_reduces_to_pid_at_integer_orders():
    """GL at λ=1, μ=1 must match discrete PID (alpha=1, no derivative filter)."""
    kp, ki, kd = 0.5, 0.1, 0.05
    pid = PIDController(kp=kp, ki=ki, kd=kd, windup_limit=100.0, derivative_alpha=1.0)
    fopid = FOPIDController(kp=kp, ki=ki, kd=kd, lambda_order=1.0, mu_order=1.0, N=50)
    dt = 0.1
    errors_seq = [0.8, 0.6, 0.5, 0.4, 0.3]
    for e in errors_seq:
        px, _ = pid.update(e, 0.0, dt)
        fx, _ = fopid.update(e, 0.0, dt)
    assert px == pytest.approx(fx, rel=1e-6)


def test_fopid_reset_clears_history():
    ctrl = FOPIDController(kp=0.0, ki=1.0, kd=0.0, lambda_order=0.7, mu_order=0.7, N=20)
    dt = 0.1
    for _ in range(15):
        ctrl.update(1.0, 0.0, dt)
    ctrl.reset()
    # After reset behaves identically to a fresh instance
    fresh = FOPIDController(kp=0.0, ki=1.0, kd=0.0, lambda_order=0.7, mu_order=0.7, N=20)
    sx1, _ = ctrl.update(1.0, 0.0, dt)
    sx2, _ = fresh.update(1.0, 0.0, dt)
    assert sx1 == pytest.approx(sx2, rel=1e-9)


def test_fopid_fractional_integral_is_weaker_than_integer():
    """Fractional integral λ<1 accumulates slower than integer integral."""
    dt = 0.1
    fopid_frac = FOPIDController(kp=0.0, ki=1.0, kd=0.0, lambda_order=0.5, mu_order=1.0, N=30)
    fopid_int  = FOPIDController(kp=0.0, ki=1.0, kd=0.0, lambda_order=1.0, mu_order=1.0, N=30)
    for _ in range(20):
        fopid_frac.update(1.0, 0.0, dt)
        fopid_int.update(1.0, 0.0, dt)
    sx_frac, _ = fopid_frac.update(1.0, 0.0, dt)
    sx_int, _  = fopid_int.update(1.0, 0.0, dt)
    # λ=0.5 accumulates less than λ=1 over the same input history
    assert sx_frac < sx_int


from mavpilot.core.controllers import ADRCController


def test_adrc_reset_clears_eso_state():
    ctrl = ADRCController(b0=1.0, omega_obs=3.0, omega_ctrl=1.5)
    dt = 0.1
    for _ in range(50):
        ctrl.update(1.0, 0.0, dt)
    ctrl.reset()
    fresh = ADRCController(b0=1.0, omega_obs=3.0, omega_ctrl=1.5)
    sx1, sy1 = ctrl.update(0.5, 0.3, dt)
    sx2, sy2 = fresh.update(0.5, 0.3, dt)
    assert sx1 == pytest.approx(sx2)
    assert sy1 == pytest.approx(sy2)


def test_adrc_rejects_constant_disturbance():
    """Simulate: e[k+1] = e[k] - u[k]*dt/tau + d*dt. ESO should cancel d.

    b0 = -1/tau matches the plant sign convention: the controller output u
    reduces the error, so the effective control gain on e_dot is -1/tau.
    omega_obs=3.0 satisfies the forward-Euler stability bound (dt*omega_obs=0.3<1)
    and the rule-of-thumb omega_obs = 2×omega_ctrl.
    """
    tau = 0.4
    ctrl = ADRCController(b0=-1.0 / tau, omega_obs=3.0, omega_ctrl=1.5)
    dt = 0.1
    e = 1.0
    disturbance = 0.15  # constant wind equivalent
    for _ in range(300):
        u, _ = ctrl.update(e, 0.0, dt)
        e = e - (u / tau) * dt + disturbance * dt
    # ADRC must drive error to near-zero despite the constant disturbance
    assert abs(e) < 0.05


def test_p_controller_under_same_disturbance_has_larger_ss_error():
    """P controller should NOT fully reject a constant disturbance (baseline check)."""
    ctrl_p = PController(kp=0.7)
    dt = 0.1
    tau = 0.4
    e = 1.0
    disturbance = 0.15
    for _ in range(300):
        u, _ = ctrl_p.update(e, 0.0, dt)
        e = e - (u / tau) * dt + disturbance * dt
    # P controller will have significant steady-state error
    assert abs(e) > 0.05
