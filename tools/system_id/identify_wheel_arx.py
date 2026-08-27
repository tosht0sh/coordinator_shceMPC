#!/usr/bin/env python3
from __future__ import annotations

"""Identify a second-order ARX wheel-speed model from logged Duckiebot data.

For each wheel, fit the discrete-time model:

    y[k] = a1 * y[k-1] + a2 * y[k-2] + b1 * u[k-1] + b2 * u[k-2]

This is a practical next step when the first-order model is too simple for the
measured black-box drive-stack dynamics.
"""

import argparse

import numpy as np
import pandas as pd


WHEEL_RADIUS = 0.0318
AXLE_LENGTH = 0.10

NA = 2
NB = 2
NK = 0

REQUIRED_COLUMNS = ["t", "v_cmd", "omega_cmd", "wL_meas", "wR_meas"]


def load_data(csv_path: str) -> pd.DataFrame:
    """Load, clean, and normalize a wheel-ID CSV file."""

    df = pd.read_csv(csv_path)

    missing = [col for col in REQUIRED_COLUMNS if col not in df.columns]
    if missing:
        raise ValueError(f"{csv_path} is missing required columns: {missing}")

    df = df.copy()
    if "enc_ready" in df.columns:
        df = df[df["enc_ready"].astype(bool)]

    df = df.replace([np.inf, -np.inf], np.nan)
    df = df.dropna(subset=REQUIRED_COLUMNS)
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


def resample_to_uniform_grid(df: pd.DataFrame, Ts: float) -> pd.DataFrame:
    """Resample commands and measured wheel speeds onto a uniform time grid."""

    if Ts <= 0.0:
        raise ValueError("Resampling time step Ts must be positive.")

    source_t = df["t"].to_numpy(dtype=float)
    t_end = float(source_t[-1])
    target_t = np.arange(0.0, t_end + 1e-12, Ts, dtype=float)
    if len(target_t) < 5:
        raise ValueError("Uniform resampling produced too few samples.")

    v_cmd = zero_order_hold_resample(source_t, df["v_cmd"].to_numpy(dtype=float), target_t)
    omega_cmd = zero_order_hold_resample(source_t, df["omega_cmd"].to_numpy(dtype=float), target_t)
    w_left_meas = np.interp(target_t, source_t, df["wL_meas"].to_numpy(dtype=float))
    w_right_meas = np.interp(target_t, source_t, df["wR_meas"].to_numpy(dtype=float))

    return pd.DataFrame(
        {
            "t": target_t,
            "v_cmd": v_cmd,
            "omega_cmd": omega_cmd,
            "wL_meas": w_left_meas,
            "wR_meas": w_right_meas,
        }
    )


def prepare_dataset(
    df_raw: pd.DataFrame,
    wheel_radius: float,
    axle_length: float,
    Ts: float,
) -> tuple[pd.DataFrame, float, float]:
    """Resample a dataset and compute wheel commands on the uniform grid."""

    df = resample_to_uniform_grid(df_raw, Ts)
    df = compute_wheel_commands(df, wheel_radius, axle_length)
    Ts_est, rel_jitter = estimate_sample_time(df["t"].to_numpy())
    return df, Ts_est, rel_jitter


def fit_arx_model(
    u: np.ndarray,
    y: np.ndarray,
    na: int = NA,
    nb: int = NB,
    nk: int = NK,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Fit a standard ARX model.

    Model form:
        y[k] = a1*y[k-1] + ... + ana*y[k-na] + b1*u[k-nk] + ... + bnb*u[k-nk-nb+1]
    """

    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(u) != len(y):
        raise ValueError("u and y must have the same length.")

    start_idx = max(na, nk + nb - 1)
    if len(y) <= start_idx:
        raise ValueError("Need more samples to fit the requested ARX model.")

    regressors: list[list[float]] = []
    targets: list[float] = []
    for k in range(start_idx, len(y)):
        row = [float(y[k - i]) for i in range(1, na + 1)]
        row.extend(float(u[k - nk - j]) for j in range(nb))
        regressors.append(row)
        targets.append(float(y[k]))

    phi = np.asarray(regressors, dtype=float)
    target = np.asarray(targets, dtype=float)
    theta, *_ = np.linalg.lstsq(phi, target, rcond=None)

    a = theta[:na]
    b = theta[na:]
    return a, b, start_idx


def simulate_arx_model(
    u: np.ndarray,
    y_init: np.ndarray,
    a: np.ndarray,
    b: np.ndarray,
    start_idx: int,
    nk: int = NK,
) -> np.ndarray:
    """Free-run the ARX model from measured initial conditions."""

    u = np.asarray(u, dtype=float)
    y_init = np.asarray(y_init, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    y_sim = np.zeros_like(u, dtype=float)
    y_sim[:start_idx] = y_init[:start_idx]

    for k in range(start_idx, len(u)):
        y_part = sum(a[i] * y_sim[k - 1 - i] for i in range(len(a)))
        u_part = sum(b[j] * u[k - nk - j] for j in range(len(b)))
        y_sim[k] = y_part + u_part

    return y_sim


def fit_metric(y_true: np.ndarray, y_sim: np.ndarray) -> dict[str, float]:
    """Return a simple engineering fit summary."""

    y_true = np.asarray(y_true, dtype=float)
    y_sim = np.asarray(y_sim, dtype=float)
    error = y_true - y_sim

    rmse = float(np.sqrt(np.mean(error**2)))
    denom = np.linalg.norm(y_true - np.mean(y_true))
    fit_percent = float("nan") if denom < 1e-12 else float(100.0 * (1.0 - np.linalg.norm(error) / denom))
    return {"rmse": rmse, "fit_percent": fit_percent}


def arx_properties(a: np.ndarray, b: np.ndarray) -> dict[str, object]:
    """Return a few helpful ARX interpretation quantities."""

    a1, a2 = float(a[0]), float(a[1])
    b1, b2 = float(b[0]), float(b[1])
    poles = np.roots([1.0, -a1, -a2])
    stable = bool(np.all(np.abs(poles) < 1.0))

    denom = 1.0 - a1 - a2
    dc_gain = None if abs(denom) < 1e-9 else float((b1 + b2) / denom)
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
) -> dict[str, object]:
    """Fit one wheel on the ID dataset and evaluate on both ID and validation data."""

    u_id = id_df[cmd_col].to_numpy(dtype=float)
    y_id = id_df[meas_col].to_numpy(dtype=float)
    a, b, start_idx = fit_arx_model(u_id, y_id)

    y_id_sim = simulate_arx_model(u_id, y_id, a, b, start_idx)
    id_metrics = fit_metric(y_id[start_idx:], y_id_sim[start_idx:])

    u_val = val_df[cmd_col].to_numpy(dtype=float)
    y_val = val_df[meas_col].to_numpy(dtype=float)
    y_val_sim = simulate_arx_model(u_val, y_val, a, b, start_idx)
    val_metrics = fit_metric(y_val[start_idx:], y_val_sim[start_idx:])

    props = arx_properties(a, b)
    return {
        "a": a,
        "b": b,
        "start_idx": start_idx,
        "id_sim": y_id_sim,
        "val_sim": y_val_sim,
        "id_metrics": id_metrics,
        "val_metrics": val_metrics,
        "poles": props["poles"],
        "stable": props["stable"],
        "dc_gain": props["dc_gain"],
    }


def format_complex_pair(values: np.ndarray) -> str:
    """Format poles compactly for terminal output."""

    parts = []
    for value in values:
        if abs(value.imag) < 1e-9:
            parts.append(f"{value.real:.4f}")
        else:
            parts.append(f"{value.real:.4f}{value.imag:+.4f}j")
    return ", ".join(parts)


def print_summary(name: str, result: dict[str, object]) -> None:
    """Print a compact ARX parameter and fit summary."""

    a = np.asarray(result["a"], dtype=float)
    b = np.asarray(result["b"], dtype=float)
    id_metrics = result["id_metrics"]
    val_metrics = result["val_metrics"]
    poles = np.asarray(result["poles"])
    stable = bool(result["stable"])
    dc_gain = result["dc_gain"]

    print(f"\n{name} wheel")
    print(f"  a1           = {a[0]:.6f}")
    print(f"  a2           = {a[1]:.6f}")
    print(f"  b1           = {b[0]:.6f}")
    print(f"  b2           = {b[1]:.6f}")
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


def plot_panel(
    ax,
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
    parser = argparse.ArgumentParser(description="Identify second-order ARX Duckiebot wheel-speed models.")
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
    parser.add_argument("--save-plot", default=None, help="Optional path to save the generated figure")
    parser.add_argument("--no-show", action="store_true", help="Do not open the plot window")
    args = parser.parse_args()

    id_raw_df = load_data(args.id_csv)
    Ts_id_raw, jitter_id_raw = estimate_sample_time(id_raw_df["t"].to_numpy())
    Ts = float(args.ts) if args.ts is not None else Ts_id_raw
    id_df, Ts_id_est, jitter_id = prepare_dataset(
        id_raw_df,
        args.wheel_radius,
        args.axle_length,
        Ts,
    )

    same_file = args.val_csv is None
    if same_file:
        val_df = id_df.copy()
        Ts_val_raw = Ts_id_raw
        jitter_val_raw = jitter_id_raw
        Ts_val_est = Ts_id_est
        jitter_val = jitter_id
    else:
        val_raw_df = load_data(args.val_csv)
        Ts_val_raw, jitter_val_raw = estimate_sample_time(val_raw_df["t"].to_numpy())
        val_df, Ts_val_est, jitter_val = prepare_dataset(
            val_raw_df,
            args.wheel_radius,
            args.axle_length,
            Ts,
        )

    print("ARX model: y[k] = a1*y[k-1] + a2*y[k-2] + b1*u[k-1] + b2*u[k-2]")
    print(f"  na={NA}, nb={NB}, nk={NK}")
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

    left_result = evaluate_wheel(id_df, val_df, "wL_cmd", "wL_meas")
    right_result = evaluate_wheel(id_df, val_df, "wR_cmd", "wR_meas")

    print_summary("Left", left_result)
    print_summary("Right", right_result)

    need_plot = (not args.no_show) or bool(args.save_plot)
    if need_plot:
        import matplotlib.pyplot as plt

        ncols = 1 if same_file else 2
        fig, axes = plt.subplots(2, ncols, figsize=(7 * ncols, 8), sharex="col")
        if ncols == 1:
            axes = np.array(axes).reshape(2, 1)

        plot_panel(
            axes[0, 0],
            id_df["t"].to_numpy(dtype=float),
            id_df["wL_meas"].to_numpy(dtype=float),
            np.asarray(left_result["id_sim"], dtype=float),
            id_df["wL_cmd"].to_numpy(dtype=float),
            "Left wheel: identification",
            int(left_result["start_idx"]),
        )
        plot_panel(
            axes[1, 0],
            id_df["t"].to_numpy(dtype=float),
            id_df["wR_meas"].to_numpy(dtype=float),
            np.asarray(right_result["id_sim"], dtype=float),
            id_df["wR_cmd"].to_numpy(dtype=float),
            "Right wheel: identification",
            int(right_result["start_idx"]),
        )

        if not same_file:
            plot_panel(
                axes[0, 1],
                val_df["t"].to_numpy(dtype=float),
                val_df["wL_meas"].to_numpy(dtype=float),
                np.asarray(left_result["val_sim"], dtype=float),
                val_df["wL_cmd"].to_numpy(dtype=float),
                "Left wheel: validation",
                int(left_result["start_idx"]),
            )
            plot_panel(
                axes[1, 1],
                val_df["t"].to_numpy(dtype=float),
                val_df["wR_meas"].to_numpy(dtype=float),
                np.asarray(right_result["val_sim"], dtype=float),
                val_df["wR_cmd"].to_numpy(dtype=float),
                "Right wheel: validation",
                int(right_result["start_idx"]),
            )

        for ax in axes[-1, :]:
            ax.set_xlabel("time [s]")

        fig.suptitle("Second-order ARX wheel-speed identification", fontsize=14)
        fig.tight_layout()

        if args.save_plot:
            fig.savefig(args.save_plot, dpi=150, bbox_inches="tight")
            print(f"\nSaved plot to: {args.save_plot}")

        if not args.no_show:
            plt.show()


if __name__ == "__main__":
    main()
