#!/usr/bin/env python3
from __future__ import annotations

"""Identify a first-order wheel-speed model from logged Duckiebot data.

For each wheel, fit the discrete-time model:

    y[k+1] = a * y[k] + b * u[k]

where:
- u[k] is the commanded wheel angular speed [rad/s]
- y[k] is the measured wheel angular speed [rad/s]
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


WHEEL_RADIUS = 0.0318
AXLE_LENGTH = 0.10

REQUIRED_COLUMNS = ["t", "v_cmd", "omega_cmd", "wL_meas", "wR_meas"]


def load_data(csv_path: str) -> pd.DataFrame:
    """Load, clean, and lightly normalize a wheel-ID CSV file."""

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

    if len(df) < 3:
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
    """Resample commands and measured wheel speeds onto a uniform time grid.

    Commands are treated as held between updates, while measured wheel speeds
    are linearly interpolated.
    """

    if Ts <= 0.0:
        raise ValueError("Resampling time step Ts must be positive.")

    source_t = df["t"].to_numpy(dtype=float)
    t_end = float(source_t[-1])
    target_t = np.arange(0.0, t_end + 1e-12, Ts, dtype=float)
    if len(target_t) < 3:
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


def fit_first_order_model(u: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """Fit y[k+1] = a y[k] + b u[k] by least squares."""

    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(u) != len(y):
        raise ValueError("u and y must have the same length.")
    if len(y) < 2:
        raise ValueError("Need at least 2 samples to fit a model.")

    regressors = np.column_stack([y[:-1], u[:-1]])
    target = y[1:]
    theta, *_ = np.linalg.lstsq(regressors, target, rcond=None)
    a, b = theta
    return float(a), float(b)


def simulate_model(u: np.ndarray, y0: float, a: float, b: float) -> np.ndarray:
    """Free-run the identified model from the first measured sample."""

    u = np.asarray(u, dtype=float)
    y_sim = np.zeros_like(u, dtype=float)
    y_sim[0] = float(y0)

    for k in range(len(u) - 1):
        y_sim[k + 1] = a * y_sim[k] + b * u[k]

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


def discrete_to_continuous(a: float, b: float, Ts: float) -> tuple[float | None, float | None]:
    """Convert to approximate continuous-time first-order parameters when valid."""

    if not (0.0 < a < 1.0):
        return None, None

    tau = -Ts / np.log(a)
    K = b / (1.0 - a)
    return float(tau), float(K)


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


def evaluate_wheel(
    id_df: pd.DataFrame,
    val_df: pd.DataFrame,
    cmd_col: str,
    meas_col: str,
    Ts: float,
) -> dict[str, object]:
    """Fit one wheel on the ID dataset and evaluate on both ID and validation data."""

    u_id = id_df[cmd_col].to_numpy(dtype=float)
    y_id = id_df[meas_col].to_numpy(dtype=float)
    a, b = fit_first_order_model(u_id, y_id)

    y_id_sim = simulate_model(u_id, y_id[0], a, b)
    id_metrics = fit_metric(y_id, y_id_sim)

    u_val = val_df[cmd_col].to_numpy(dtype=float)
    y_val = val_df[meas_col].to_numpy(dtype=float)
    y_val_sim = simulate_model(u_val, y_val[0], a, b)
    val_metrics = fit_metric(y_val, y_val_sim)

    tau, K = discrete_to_continuous(a, b, Ts)
    return {
        "a": a,
        "b": b,
        "tau": tau,
        "K": K,
        "id_sim": y_id_sim,
        "val_sim": y_val_sim,
        "id_metrics": id_metrics,
        "val_metrics": val_metrics,
    }


def print_summary(name: str, result: dict[str, object]) -> None:
    """Print a compact parameter and fit summary."""

    a = float(result["a"])
    b = float(result["b"])
    tau = result["tau"]
    K = result["K"]
    id_metrics = result["id_metrics"]
    val_metrics = result["val_metrics"]

    print(f"\n{name} wheel")
    print(f"  a            = {a:.6f}")
    print(f"  b            = {b:.6f}")
    print(f"  ID fit [%]   = {id_metrics['fit_percent']:.2f}")
    print(f"  ID RMSE      = {id_metrics['rmse']:.4f} rad/s")
    print(f"  VAL fit [%]  = {val_metrics['fit_percent']:.2f}")
    print(f"  VAL RMSE     = {val_metrics['rmse']:.4f} rad/s")

    if tau is None or K is None:
        print("  tau, K       = not reported (a is outside 0 < a < 1)")
    else:
        print(f"  tau          = {float(tau):.4f} s")
        print(f"  K            = {float(K):.4f} rad/s per rad/s")


def plot_panel(
    ax: plt.Axes,
    t: np.ndarray,
    y_meas: np.ndarray,
    y_sim: np.ndarray,
    y_cmd: np.ndarray,
    title: str,
) -> None:
    """Plot measured, simulated, and commanded wheel speeds."""

    ax.plot(t, y_meas, label="measured", linewidth=2.0)
    ax.plot(t, y_sim, "--", label="simulated", linewidth=2.0)
    ax.plot(t, y_cmd, ":", label="command", linewidth=1.5, alpha=0.8)
    ax.set_title(title)
    ax.set_ylabel("wheel speed [rad/s]")
    ax.grid(True)
    ax.legend()


def main() -> None:
    parser = argparse.ArgumentParser(description="Identify first-order Duckiebot wheel-speed models.")
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
        if args.ts is None and abs(Ts_val_raw - Ts_id_raw) > 0.10 * Ts:
            print(
                "[warn] Identification and validation sample times differ noticeably. "
                "Both datasets were still resampled to the same fitting Ts."
            )

    left_result = evaluate_wheel(id_df, val_df, "wL_cmd", "wL_meas", Ts)
    right_result = evaluate_wheel(id_df, val_df, "wR_cmd", "wR_meas", Ts)

    print_summary("Left", left_result)
    print_summary("Right", right_result)

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
    )
    plot_panel(
        axes[1, 0],
        id_df["t"].to_numpy(dtype=float),
        id_df["wR_meas"].to_numpy(dtype=float),
        np.asarray(right_result["id_sim"], dtype=float),
        id_df["wR_cmd"].to_numpy(dtype=float),
        "Right wheel: identification",
    )

    if not same_file:
        plot_panel(
            axes[0, 1],
            val_df["t"].to_numpy(dtype=float),
            val_df["wL_meas"].to_numpy(dtype=float),
            np.asarray(left_result["val_sim"], dtype=float),
            val_df["wL_cmd"].to_numpy(dtype=float),
            "Left wheel: validation",
        )
        plot_panel(
            axes[1, 1],
            val_df["t"].to_numpy(dtype=float),
            val_df["wR_meas"].to_numpy(dtype=float),
            np.asarray(right_result["val_sim"], dtype=float),
            val_df["wR_cmd"].to_numpy(dtype=float),
            "Right wheel: validation",
        )

    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")

    fig.suptitle("First-order wheel-speed identification", fontsize=14)
    fig.tight_layout()

    if args.save_plot:
        fig.savefig(args.save_plot, dpi=150, bbox_inches="tight")
        print(f"\nSaved plot to: {args.save_plot}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
