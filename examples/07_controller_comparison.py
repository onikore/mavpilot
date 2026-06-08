"""Compare P / PID / FO-PID / ADRC on a synthetic disturbed plant.

Usage:
    python3 examples/07_controller_comparison.py
    python3 examples/07_controller_comparison.py --plot   # requires matplotlib

Plant model: first-order lag  e[k+1] = e[k] - (u / TAU) * DT + d * DT + noise
where TAU is the inner-loop time constant. A step disturbance is injected at
DISTURBANCE_STEP to test steady-state rejection.
"""
from __future__ import annotations

import argparse
import math
import random
import sys

from mavpilot.core.controllers import (
    ADRCController,
    FOPIDController,
    PController,
    PIDController,
)

DT = 0.1              # control loop period (s)
STEPS = 300           # 30-second scenario
DISTURBANCE_STEP = 80 # inject step wind at k=80 (t=8 s)
DISTURBANCE_MAG = 0.15  # lateral drift (m/s equivalent)
NOISE_STD = 0.02      # sensor noise σ (m)
TAU = 0.4             # inner-loop time constant (s)


def _make_controllers() -> dict:
    return {
        "P":      PController(kp=0.7),
        "PID":    PIDController(kp=0.5, ki=0.05, kd=0.15),
        "FO-PID": FOPIDController(kp=0.5, ki=0.05, kd=0.15, lambda_order=0.8, mu_order=0.9, N=20),
        "ADRC":   ADRCController(b0=-1.0 / TAU, omega_obs=3.0, omega_ctrl=1.5),
    }


def simulate(ctrl, seed: int = 42) -> tuple[list[float], list[float]]:
    rng = random.Random(seed)
    e = 1.0
    errors = [e]
    controls = [0.0]

    for k in range(STEPS):
        dist = DISTURBANCE_MAG if k >= DISTURBANCE_STEP else 0.0
        # Current measurement (with noise, no artificial delay)
        e_meas = e + rng.gauss(0.0, NOISE_STD)

        u_x, _ = ctrl.update(e_meas, 0.0, DT)

        # Plant dynamics: e[k+1] = e[k] - (u[k]/TAU)*DT + d[k]*DT
        e = e - (u_x / TAU) * DT + dist * DT
        errors.append(e)
        controls.append(u_x)

    return errors, controls


def _metrics(name: str, errors: list[float], controls: list[float]) -> dict:
    iae = sum(abs(err) * DT for err in errors)
    effort = sum(abs(u) for u in controls)

    settling: float | None = None
    for i in range(len(errors) - 10):
        if all(abs(errors[i + j]) < 0.02 for j in range(10)):
            settling = i * DT
            break

    sign0 = math.copysign(1.0, errors[0]) if errors[0] != 0.0 else 1.0
    overshoot = max(0.0, max(-sign0 * e for e in errors) / abs(errors[0]) * 100.0)

    ss_window = errors[-30:]
    ss_error = sum(abs(e) for e in ss_window) / len(ss_window)

    return {
        "name": name,
        "IAE": iae,
        "settling_s": settling,
        "overshoot_%": overshoot,
        "ss_error_m": ss_error,
        "effort": effort,
    }


def _print_table(results: list[dict]) -> None:
    header = (
        f"{'Controller':<10}  {'IAE':>7}  {'settle(s)':>10}  "
        f"{'overshoot%':>11}  {'ss_err(m)':>10}  {'effort':>8}"
    )
    print(header)
    print("-" * len(header))
    for r in results:
        s = f"{r['settling_s']:8.1f}" if r["settling_s"] is not None else "      n/a"
        print(
            f"{r['name']:<10}  {r['IAE']:7.3f}  {s:>10}  "
            f"{r['overshoot_%']:11.1f}  {r['ss_error_m']:10.4f}  {r['effort']:8.2f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Lateral controller comparison")
    parser.add_argument("--plot", action="store_true", help="Show matplotlib trajectory plot")
    args = parser.parse_args()

    controllers = _make_controllers()
    all_errors: dict[str, list[float]] = {}
    results: list[dict] = []

    for name, ctrl in controllers.items():
        ctrl.reset()
        errors, controls = simulate(ctrl)
        all_errors[name] = errors
        results.append(_metrics(name, errors, controls))

    _print_table(results)

    if args.plot:
        try:
            import matplotlib.pyplot as plt

            t = [i * DT for i in range(STEPS + 1)]
            for name, errors in all_errors.items():
                plt.plot(t, errors, label=name)
            plt.axvline(DISTURBANCE_STEP * DT, color="gray", linestyle="--", label="disturbance")
            plt.axhline(0.02, color="lightgray", linestyle=":")
            plt.axhline(-0.02, color="lightgray", linestyle=":")
            plt.xlabel("Time (s)")
            plt.ylabel("Lateral error (m)")
            plt.title("Lateral controller comparison — first-order plant + step disturbance")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("\nmatplotlib not installed — run: pip install matplotlib", file=sys.stderr)


if __name__ == "__main__":
    main()
