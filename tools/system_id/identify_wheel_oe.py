#!/usr/bin/env python3
from __future__ import annotations

"""Identify an output-error wheel-speed model from logged Duckiebot data.

For each wheel, fit the discrete-time OE model:

    y[k] = -f1*y[k-1] - ... - fnf*y[k-nf]
           + b1*u[k-nk] + ... + bnb*u[k-nk-nb+1]

This treats the measured wheel speed as the output of a deterministic plant
driven by the commanded wheel speed. Unlike ARX, the prediction error is
computed from a free-run simulation, so the identified denominator is less
biased by output noise.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.optimize import least_squares


WHEEL_RADIUS = 0.0318
AXLE_LENGTH = 0.10

NB = 2
NF = 2
NK = 0

REQUIRED_BASE_COLUMNS = ["t", "v_cmd", "omega_cmd"]


def measurement_columns(meas_signal: str) -> tuple[str, str]:
    if meas_signal == "filtered":
        return "wL_meas", "wR_meas"
    if meas_signal == "raw":
        return "wL_raw", "wR_raw"
    raise ValueError(f"Unsupported measurement signal: {meas_signal}")


def load_data(csv_path: str, meas_cols: tuple[str, str]) -> pd.DataFrame:
    """Load, clean, and lightly normalize a wheel-ID CSV file."""

    required_cols = REQUIRED_BASE_COLUMNS + list(meas_cols)
    df = pd.read_csv(csv_path)

    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing required columns: {missing}")

    df = df.copy()
    if "enc_ready" in df.columns:
        df = df[df["enc_ready"].astype(bool)]

    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.dropna(subset=required_cols)
    df = df.sort_values("t")
    df = df.drop_duplicates(subset="t")
    df = df.reset_index(drop=True)

    if len(df) < 5:
        raise ValueError(f"{csv_path} does not contain enough usable samples.")

    df["t"] = df["t"] - float(df["t"].iloc[0])
    return df


def compute_wheel_commands(
    df: pd.DataFrame,
    wheel_radius: float,
    axle_length: float,
) -> pd.DataFrame:
    """Convert body commands into left/right wheel angular speed commands."""

    out = df.copy()
    out["wL_cmd"] = (out["v_cmd"] - 0.5 * axle_length * out["omega_cmd"]) / wheel_radius
    out["wR_cmd"] = (out["v_cmd"] + 0.5 * axle_length * out["omega_cmd"]) / wheel_radius
    return out


def estimate_sample_time(t: np.ndarray) -> tuple[float, float]:
    """Return the median sample time and a simple relative jitter metric."""

    dt = np.diff(np.asarray(t, dtype=float))
    dt = dt[dt > 0.0]
    if len(dt) == 0:
        raise ValueError("Could not estimate sample time from timestamps.")

    Ts = float(np.median(dt))
    rel_jitter = float(np.max(np.abs(dt - Ts)) / Ts) if Ts > 0.0 else 0.0
    return Ts, rel_jitter


def zero_order_hold_resample(source_t: np.ndarray, source_y: np.ndarray, target_t: np.ndarray) -> np.ndarray:
    """Resample a held command signal onto a uniform grid."""

    indices = np.searchsorted(source_t, target_t, side="right") - 1
    indices = np.clip(indices, 0, len(source_y) - 1)
    return source_y[indices]


def resample_to_uniform_grid(df: pd.DataFrame, Ts: float, meas_cols: tuple[str, str]) -> pd.DataFrame:
    """Resample commands and selected wheel-speed signals onto a uniform grid."""

    if Ts <= 0.0:
        raise ValueError("Resampling time step Ts must be positive.")

    left_meas_col, right_meas_col = meas_cols
    source_t = df["t"].to_numpy(dtype=float)
    t_end = float(source_t[-1])
    target_t = np.arange(0.0, t_end + 1e-12, Ts, dtype=float)
    if len(target_t) < 5:
        raise ValueError("Uniform resampling produced too few samples.")

    v_cmd = zero_order_hold_resample(source_t, df["v_cmd"].to_numpy(dtype=float), target_t)
    omega_cmd = zero_order_hold_resample(source_t, df["omega_cmd"].to_numpy(dtype=float), target_t)
    w_left = np.interp(target_t, source_t, df[left_meas_col].to_numpy(dtype=float))
    w_right = np.interp(target_t, source_t, df[right_meas_col].to_numpy(dtype=float))

    return pd.DataFrame(
        {
            "t": target_t,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            left_meas_col: w_left,
            right_meas_col: w_right,
        }
    )


def prepare_dataset(
    df_raw: pd.DataFrame,
    wheel_radius: float,
    axle_length: float,
    Ts: float,
    meas_cols: tuple[str, str],
) -> tuple[pd.DataFrame, float, float]:
    """Resample a dataset and compute wheel commands on the uniform grid."""

    df = resample_to_uniform_grid(df_raw, Ts, meas_cols)
    df = compute_wheel_commands(df, wheel_radius, axle_length)
    Ts_est, rel_jitter = estimate_sample_time(df["t"].to_numpy())
    return df, Ts_est, rel_jitter


def fit_metric(y_true: np.ndarray, y_sim: np.ndarray) -> dict[str, float]:
    """Return a simple engineering fit summary."""

    y_true = np.asarray(y_true, dtype=float)
    y_sim = np.asarray(y_sim, dtype=float)
    error = y_true - y_sim

    rmse = float(np.sqrt(np.mean(error**2)))
    denom = np.linalg.norm(y_true - np.mean(y_true))
    fit_percent = float("nan") if denom < 1e-12 else float(100.0 * (1.0 - np.linalg.norm(error) / denom))
    return {"rmse": rmse, "fit_percent": fit_percent}


def start_index(nb: int, nf: int, nk: int) -> int:
    """Return the first sample index with enough history for the OE model."""

    return max(nf, nk + nb - 1, 1)


def fit_arx_initial_guess(
    u: np.ndarray,
    y: np.ndarray,
    nb: int,
    nf: int,
    nk: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Build an ARX-style least-squares initial guess for the OE optimizer."""

    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    idx0 = start_index(nb, nf, nk)

    regressors: list[list[float]] = []
    targets: list[float] = []
    for k in range(idx0, len(y)):
        row = [-float(y[k - 1 - i]) for i in range(nf)]
        row.extend(float(u[k - nk - j]) for j in range(nb))
        regressors.append(row)
        targets.append(float(y[k]))

    phi = np.asarray(regressors, dtype=float)
    target = np.asarray(targets, dtype=float)
    theta, *_ = np.linalg.lstsq(phi, target, rcond=None)
    f = theta[:nf]
    b = theta[nf:]
    return np.asarray(f, dtype=float), np.asarray(b, dtype=float)


def simulate_oe_model(
    u: np.ndarray,
    y_init: np.ndarray,
    f: np.ndarray,
    b: np.ndarray,
    nk: int,
) -> np.ndarray:
    """Free-run the OE model from measured initial conditions."""

    u = np.asarray(u, dtype=float)
    y_init = np.asarray(y_init, dtype=float)
    f = np.asarray(f, dtype=float)
    b = np.asarray(b, dtype=float)

    idx0 = start_index(len(b), len(f), nk)
    y_sim = np.zeros_like(u, dtype=float)
    y_sim[:idx0] = y_init[:idx0]

    for k in range(idx0, len(u)):
        y_part = -sum(f[i] * y_sim[k - 1 - i] for i in range(len(f)))
        u_part = sum(b[j] * u[k - nk - j] for j in range(len(b)))
        y_sim[k] = y_part + u_part

    return y_sim


def oe_residuals(theta: np.ndarray, u: np.ndarray, y: np.ndarray, nb: int, nf: int, nk: int) -> np.ndarray:
    """Return free-run simulation errors for the requested OE structure."""

    f = theta[:nf]
    b = theta[nf:]
    idx0 = start_index(nb, nf, nk)
    y_sim = simulate_oe_model(u, y, f, b, nk)
    return y[idx0:] - y_sim[idx0:]


def fit_oe_model(
    u: np.ndarray,
    y: np.ndarray,
    nb: int = NB,
    nf: int = NF,
    nk: int = NK,
    max_nfev: int = 400,
) -> tuple[np.ndarray, np.ndarray, int, object]:
    """Fit an OE model by minimizing free-run simulation error."""

    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(u) != len(y):
        raise ValueError("u and y must have the same length.")
    if nb < 1:
        raise ValueError("nb must be at least 1.")
    if nf < 0:
        raise ValueError("nf must be non-negative.")
    if nk < 0:
        raise ValueError("nk must be non-negative.")

    idx0 = start_index(nb, nf, nk)
    if len(y) <= idx0:
        raise ValueError("Need more samples to fit the requested OE model.")

    f0_arx, b0_arx = fit_arx_initial_guess(u, y, nb, nf, nk)
    theta0_arx = np.concatenate([f0_arx, b0_arx])
    theta0_zero = np.zeros(nf + nb, dtype=float)
    if len(b0_arx) > 0:
        theta0_zero[nf] = b0_arx[0]

    best_result = None
    for theta0 in (theta0_arx, theta0_zero):
        result = least_squares(
            oe_residuals,
            theta0,
            args=(u, y, nb, nf, nk),
            method="trf",
            max_nfev=max_nfev,
            x_scale="jac",
        )
        if best_result is None or result.cost < best_result.cost:
            best_result = result

    assert best_result is not None
    f = best_result.x[:nf]
    b = best_result.x[nf:]
    return np.asarray(f, dtype=float), np.asarray(b, dtype=float), idx0, best_result


def oe_properties(f: np.ndarray, b: np.ndarray) -> dict[str, object]:
    """Return a few helpful OE interpretation quantities."""

    f = np.asarray(f, dtype=float)
    b = np.asarray(b, dtype=float)
    poles = np.roots(np.concatenate([[1.0], f])) if len(f) > 0 else np.asarray([])
    stable = bool(np.all(np.abs(poles) < 1.0)) if len(poles) > 0 else True

    denom = 1.0 + float(np.sum(f))
    dc_gain = None if abs(denom) < 1e-9 else float(np.sum(b) / denom)
    return {
        "poles": poles,
        "stable": stable,
        "dc_gain": dc_gain,
    }


def evaluate_wheel(
    id_df: pd.DataFrame,
    val_df: pd.DataFrame,
    cmd_col: str,
    meas_col: str,
    nb: int,
    nf: int,
    nk: int,
    max_nfev: int,
) -> dict[str, object]:
    """Fit one wheel on the ID dataset and evaluate on both ID and validation data."""

    u_id = id_df[cmd_col].to_numpy(dtype=float)
    y_id = id_df[meas_col].to_numpy(dtype=float)
    f, b, idx0, solver = fit_oe_model(u_id, y_id, nb=nb, nf=nf, nk=nk, max_nfev=max_nfev)

    y_id_sim = simulate_oe_model(u_id, y_id, f, b, nk)
    id_metrics = fit_metric(y_id[idx0:], y_id_sim[idx0:])

    u_val = val_df[cmd_col].to_numpy(dtype=float)
    y_val = val_df[meas_col].to_numpy(dtype=float)
    y_val_sim = simulate_oe_model(u_val, y_val, f, b, nk)
    val_metrics = fit_metric(y_val[idx0:], y_val_sim[idx0:])

    props = oe_properties(f, b)
    return {
        "f": f,
        "b": b,
        "start_idx": idx0,
        "id_sim": y_id_sim,
        "val_sim": y_val_sim,
        "id_metrics": id_metrics,
        "val_metrics": val_metrics,
        "poles": props["poles"],
        "stable": props["stable"],
        "dc_gain": props["dc_gain"],
        "solver_success": bool(solver.success),
        "solver_status": int(solver.status),
        "solver_nfev": int(solver.nfev),
        "solver_cost": float(solver.cost),
        "solver_message": str(solver.message),
    }


def format_complex_pair(values: np.ndarray) -> str:
    """Format poles compactly for terminal output."""

    if len(values) == 0:
        return "none"

    parts = []
    for value in values:
        if abs(value.imag) < 1e-9:
            parts.append(f"{value.real:.4f}")
        else:
            parts.append(f"{value.real:.4f}{value.imag:+.4f}j")
    return ", ".join(parts)


def print_summary(name: str, result: dict[str, object]) -> None:
    """Print a compact OE parameter and fit summary."""

    f = np.asarray(result["f"], dtype=float)
    b = np.asarray(result["b"], dtype=float)
    id_metrics = result["id_metrics"]
    val_metrics = result["val_metrics"]
    poles = np.asarray(result["poles"])
    stable = bool(result["stable"])
    dc_gain = result["dc_gain"]

    print(f"\n{name} wheel")
    for idx, value in enumerate(f, start=1):
        print(f"  f{idx:<11}= {value:.6f}")
    for idx, value in enumerate(b, start=1):
        print(f"  b{idx:<11}= {value:.6f}")
    print(f"  ID fit [%]   = {id_metrics['fit_percent']:.2f}")
    print(f"  ID RMSE      = {id_metrics['rmse']:.4f} rad/s")
    print(f"  VAL fit [%]  = {val_metrics['fit_percent']:.2f}")
    print(f"  VAL RMSE     = {val_metrics['rmse']:.4f} rad/s")
    print(f"  poles        = {format_complex_pair(poles)}")
    print(f"  stable       = {stable}")
    if dc_gain is None:
        print("  dc_gain      = not reported (denominator nearly singular)")
    else:
        print(f"  dc_gain      = {float(dc_gain):.4f} rad/s per rad/s")
    print(f"  solver ok    = {bool(result['solver_success'])}")
    print(f"  solver nfev  = {int(result['solver_nfev'])}")
    print(f"  solver cost  = {float(result['solver_cost']):.6f}")


def plot_panel(
    ax: plt.Axes,
    t: np.ndarray,
    y_meas: np.ndarray,
    y_sim: np.ndarray,
    y_cmd: np.ndarray,
    title: str,
    valid_from_idx: int,
) -> None:
    """Plot measured, simulated, and commanded wheel speeds."""

    ax.plot(t, y_meas, label="measured", linewidth=2.0)
    ax.plot(t, y_sim, "--", label="simulated", linewidth=2.0)
    ax.plot(t, y_cmd, ":", label="command", linewidth=1.5, alpha=0.8)
    ax.axvline(t[valid_from_idx], color="k", linestyle=":", alpha=0.25)
    ax.set_title(title)
    ax.set_ylabel("wheel speed [rad/s]")
    ax.grid(True)
    ax.legend()


def main() -> None:
    parser = argparse.ArgumentParser(description="Identify OE Duckiebot wheel-speed models.")
    parser.add_argument("id_csv", help="CSV used for identification")
    parser.add_argument(
        "--val-csv",
        default=None,
        help="Optional CSV used for validation. If omitted, reuse the ID file.",
    )
    parser.add_argument("--wheel-radius", type=float, default=WHEEL_RADIUS)
    parser.add_argument("--axle-length", type=float, default=AXLE_LENGTH)
    parser.add_argument(
        "--ts",
        type=float,
        default=None,
        help="Optional resampling time step [s]. Otherwise use the ID median timestamp spacing.",
    )
    parser.add_argument("--nb", type=int, default=NB, help="Number of input numerator coefficients.")
    parser.add_argument("--nf", type=int, default=NF, help="Number of output denominator coefficients.")
    parser.add_argument("--nk", type=int, default=NK, help="Input delay in samples.")
    parser.add_argument(
        "--meas-signal",
        choices=["filtered", "raw"],
        default="filtered",
        help="Use filtered encoder speed columns or raw differentiated wheel speeds.",
    )
    parser.add_argument(
        "--max-nfev",
        type=int,
        default=400,
        help="Maximum least-squares function evaluations per wheel.",
    )
    parser.add_argument("--save-plot", default=None, help="Optional path to save the generated figure")
    parser.add_argument("--no-show", action="store_true", help="Do not open the plot window")
    args = parser.parse_args()

    meas_cols = measurement_columns(args.meas_signal)
    id_raw_df = load_data(args.id_csv, meas_cols)
    Ts_id_raw, jitter_id_raw = estimate_sample_time(id_raw_df["t"].to_numpy())
    Ts = float(args.ts) if args.ts is not None else Ts_id_raw
    id_df, Ts_id_est, jitter_id = prepare_dataset(
        id_raw_df,
        args.wheel_radius,
        args.axle_length,
        Ts,
        meas_cols,
    )

    same_file = args.val_csv is None
    if same_file:
        val_df = id_df.copy()
        Ts_val_raw = Ts_id_raw
        jitter_val_raw = jitter_id_raw
        Ts_val_est = Ts_id_est
        jitter_val = jitter_id
    else:
        val_raw_df = load_data(args.val_csv, meas_cols)
        Ts_val_raw, jitter_val_raw = estimate_sample_time(val_raw_df["t"].to_numpy())
        val_df, Ts_val_est, jitter_val = prepare_dataset(
            val_raw_df,
            args.wheel_radius,
            args.axle_length,
            Ts,
            meas_cols,
        )

    print("OE model: y[k] = -f1*y[k-1] - ... - fnf*y[k-nf] + b1*u[k-nk] + ...")
    print(f"  nb={args.nb}, nf={args.nf}, nk={args.nk}")
    print(f"  measurement    = {args.meas_signal}")
    print(f"Identification CSV: {args.id_csv}")
    print(f"  raw estimated Ts = {Ts_id_raw:.6f} s")
    print(f"  raw rel jitter   = {100.0 * jitter_id_raw:.2f} %")
    print(f"  resampled Ts     = {Ts_id_est:.6f} s")
    print(f"  resampled jitter = {100.0 * jitter_id:.2f} %")
    print(f"  used Ts          = {Ts:.6f} s")

    if not same_file:
        print(f"Validation CSV: {args.val_csv}")
        print(f"  raw estimated Ts = {Ts_val_raw:.6f} s")
        print(f"  raw rel jitter   = {100.0 * jitter_val_raw:.2f} %")
        print(f"  resampled Ts     = {Ts_val_est:.6f} s")
        print(f"  resampled jitter = {100.0 * jitter_val:.2f} %")

    left_result = evaluate_wheel(
        id_df,
        val_df,
        "wL_cmd",
        meas_cols[0],
        nb=args.nb,
        nf=args.nf,
        nk=args.nk,
        max_nfev=args.max_nfev,
    )
    right_result = evaluate_wheel(
        id_df,
        val_df,
        "wR_cmd",
        meas_cols[1],
        nb=args.nb,
        nf=args.nf,
        nk=args.nk,
        max_nfev=args.max_nfev,
    )

    print_summary("Left", left_result)
    print_summary("Right", right_result)

    need_plot = (not args.no_show) or bool(args.save_plot)
    if need_plot:
        ncols = 1 if same_file else 2
        fig, axes = plt.subplots(2, ncols, figsize=(7 * ncols, 8), sharex="col")
        if ncols == 1:
            axes = np.array(axes).reshape(2, 1)

        plot_panel(
            axes[0, 0],
            id_df["t"].to_numpy(dtype=float),
            id_df[meas_cols[0]].to_numpy(dtype=float),
            np.asarray(left_result["id_sim"], dtype=float),
            id_df["wL_cmd"].to_numpy(dtype=float),
            "Left wheel: identification",
            int(left_result["start_idx"]),
        )
        plot_panel(
            axes[1, 0],
            id_df["t"].to_numpy(dtype=float),
            id_df[meas_cols[1]].to_numpy(dtype=float),
            np.asarray(right_result["id_sim"], dtype=float),
            id_df["wR_cmd"].to_numpy(dtype=float),
            "Right wheel: identification",
            int(right_result["start_idx"]),
        )

        if not same_file:
            plot_panel(
                axes[0, 1],
                val_df["t"].to_numpy(dtype=float),
                val_df[meas_cols[0]].to_numpy(dtype=float),
                np.asarray(left_result["val_sim"], dtype=float),
                val_df["wL_cmd"].to_numpy(dtype=float),
                "Left wheel: validation",
                int(left_result["start_idx"]),
            )
            plot_panel(
                axes[1, 1],
                val_df["t"].to_numpy(dtype=float),
                val_df[meas_cols[1]].to_numpy(dtype=float),
                np.asarray(right_result["val_sim"], dtype=float),
                val_df["wR_cmd"].to_numpy(dtype=float),
                "Right wheel: validation",
                int(right_result["start_idx"]),
            )

        for ax in axes[-1, :]:
            ax.set_xlabel("time [s]")

        fig.suptitle("OE wheel-speed identification", fontsize=14)
        fig.tight_layout()

        if args.save_plot:
            fig.savefig(args.save_plot, dpi=150, bbox_inches="tight")
            print(f"\nSaved plot to: {args.save_plot}")

        if not args.no_show:
            plt.show()


if __name__ == "__main__":
    main()
