#!/usr/bin/env python3
from __future__ import annotations

"""Simulate the wheel PI loop on top of an identified first-order plant."""

import matplotlib.pyplot as plt
import numpy as np


# Replace these with identified parameters from identify_wheel_first_order.py.
A_LEFT = 0.804530
B_LEFT = 0.100806

A_RIGHT = 0.774506
B_RIGHT = 0.113154

# Match the onboard controller as a starting point.
TS = 0.2
KP = 1.5
KI = 1.5
INTEGRAL_LIMIT = 8.0

WHEEL_RADIUS = 0.0318
AXLE_LENGTH = 0.10

STEP_TIME = 1.0
STEP_AMPLITUDE = 8.0
SIM_TIME = 8.0


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def build_step_reference(
    t: np.ndarray,
    step_time: float,
    amplitude: float,
    initial_value: float = 0.0,
) -> np.ndarray:
    ref = np.full_like(t, initial_value, dtype=float)
    ref[t >= step_time] = amplitude
    return ref


def body_to_wheel_refs(
    v_ref: np.ndarray,
    omega_ref: np.ndarray,
    wheel_radius: float,
    axle_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    w_left_ref = (v_ref - 0.5 * axle_length * omega_ref) / wheel_radius
    w_right_ref = (v_ref + 0.5 * axle_length * omega_ref) / wheel_radius
    return w_left_ref, w_right_ref


def simulate_pi_single_wheel(
    a: float,
    b: float,
    Ts: float,
    kp: float,
    ki: float,
    integral_limit: float,
    w_ref: np.ndarray,
    w0: float = 0.0,
) -> dict[str, np.ndarray]:
    """Simulate one wheel PI loop with integral clamping."""

    w_ref = np.asarray(w_ref, dtype=float)
    n = len(w_ref)

    w_meas = np.zeros(n, dtype=float)
    w_cmd = np.zeros(n, dtype=float)
    integral_hist = np.zeros(n, dtype=float)
    error_hist = np.zeros(n, dtype=float)

    w_meas[0] = float(w0)
    integral = 0.0

    for k in range(n - 1):
        error = w_ref[k] - w_meas[k]
        integral = clamp(integral + error * Ts, -integral_limit, integral_limit)
        w_cmd[k] = w_ref[k] + kp * error + ki * integral
        w_meas[k + 1] = a * w_meas[k] + b * w_cmd[k]

        error_hist[k] = error
        integral_hist[k + 1] = integral

    final_error = w_ref[-1] - w_meas[-1]
    error_hist[-1] = final_error
    w_cmd[-1] = w_ref[-1] + kp * final_error + ki * integral
    return {
        "w_ref": w_ref,
        "w_meas": w_meas,
        "w_cmd": w_cmd,
        "integral": integral_hist,
        "error": error_hist,
    }


def simulate_pi_two_wheels(
    left_model: tuple[float, float],
    right_model: tuple[float, float],
    Ts: float,
    kp_left: float,
    ki_left: float,
    kp_right: float,
    ki_right: float,
    integral_limit: float,
    w_left_ref: np.ndarray,
    w_right_ref: np.ndarray,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Simulate left and right wheels independently with separate models."""

    left = simulate_pi_single_wheel(
        a=left_model[0],
        b=left_model[1],
        Ts=Ts,
        kp=kp_left,
        ki=ki_left,
        integral_limit=integral_limit,
        w_ref=w_left_ref,
    )
    right = simulate_pi_single_wheel(
        a=right_model[0],
        b=right_model[1],
        Ts=Ts,
        kp=kp_right,
        ki=ki_right,
        integral_limit=integral_limit,
        w_ref=w_right_ref,
    )
    return left, right


def plot_single_wheel(t: np.ndarray, result: dict[str, np.ndarray], integral_limit: float) -> None:
    """Plot single-wheel reference tracking, command, and integral state."""

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    axes[0].plot(t, result["w_ref"], label="w_ref", linewidth=2.0)
    axes[0].plot(t, result["w_meas"], "--", label="w_meas", linewidth=2.0)
    axes[0].set_ylabel("speed [rad/s]")
    axes[0].set_title("Single-wheel PI simulation")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, result["w_cmd"], label="w_cmd", color="tab:orange", linewidth=2.0)
    axes[1].plot(t, result["error"], label="error", color="tab:red", alpha=0.8)
    axes[1].set_ylabel("command / error")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, result["integral"], label="integral", color="tab:green", linewidth=2.0)
    axes[2].axhline(integral_limit, color="k", linestyle=":", alpha=0.7)
    axes[2].axhline(-integral_limit, color="k", linestyle=":", alpha=0.7)
    axes[2].set_ylabel("integral")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True)
    axes[2].legend()

    fig.tight_layout()


def plot_two_wheels(
    t: np.ndarray,
    left: dict[str, np.ndarray],
    right: dict[str, np.ndarray],
) -> None:
    """Plot left and right wheel tracking for the two-wheel example."""

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    axes[0].plot(t, left["w_ref"], label="left ref", linewidth=2.0)
    axes[0].plot(t, left["w_meas"], "--", label="left meas", linewidth=2.0)
    axes[0].set_ylabel("left [rad/s]")
    axes[0].set_title("Independent left/right wheel PI simulation")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, right["w_ref"], label="right ref", linewidth=2.0)
    axes[1].plot(t, right["w_meas"], "--", label="right meas", linewidth=2.0)
    axes[1].set_ylabel("right [rad/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()


def main() -> None:
    t = np.arange(0.0, SIM_TIME + TS, TS)

    # First simulate a simple wheel-speed step on one wheel.
    w_ref_step = build_step_reference(t, STEP_TIME, STEP_AMPLITUDE)
    single = simulate_pi_single_wheel(
        a=A_LEFT,
        b=B_LEFT,
        Ts=TS,
        kp=KP,
        ki=KI,
        integral_limit=INTEGRAL_LIMIT,
        w_ref=w_ref_step,
    )
    plot_single_wheel(t, single, INTEGRAL_LIMIT)

    # Then show how the same wheel models can be driven from body references.
    v_ref = np.zeros_like(t)
    omega_ref = np.zeros_like(t)
    v_ref[t >= STEP_TIME] = 0.20

    w_left_ref, w_right_ref = body_to_wheel_refs(
        v_ref=v_ref,
        omega_ref=omega_ref,
        wheel_radius=WHEEL_RADIUS,
        axle_length=AXLE_LENGTH,
    )
    left, right = simulate_pi_two_wheels(
        left_model=(A_LEFT, B_LEFT),
        right_model=(A_RIGHT, B_RIGHT),
        Ts=TS,
        kp_left=KP,
        ki_left=KI,
        kp_right=KP,
        ki_right=KI,
        integral_limit=INTEGRAL_LIMIT,
        w_left_ref=w_left_ref,
        w_right_ref=w_right_ref,
    )
    plot_two_wheels(t, left, right)

    plt.show()


if __name__ == "__main__":
    main()
