#!/usr/bin/env python3
"""Analyze physical Duckiebot experiment runs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Iterable

import pandas as pd


REPO_ROOT = Path(__file__).resolve().parents[2]
TRAJPLAN_ROOT = REPO_ROOT / "assets" / "TrajPlan-ScheMPC-copy"
DEFAULT_RUN_ROOT = TRAJPLAN_ROOT / "data" / "experiment_runs"
DEFAULT_GRAPH = TRAJPLAN_ROOT / "data" / "schedule_demo2_data" / "SingleRobotEnv" / "graph.json"
DEFAULT_GROUPS = ("011-015", "016-020", "021-025", "026-030")


def _parse_group(group: str) -> list[str]:
    start_raw, sep, end_raw = group.partition("-")
    if not sep:
        run_number = int(start_raw)
        return [f"{run_number:03d}"]
    start = int(start_raw)
    end = int(end_raw)
    if end < start:
        raise ValueError(f"Invalid run group {group!r}: end is before start.")
    return [f"{run_number:03d}" for run_number in range(start, end + 1)]


def _find_schedule(run_dir: Path) -> Path:
    schedules = sorted(run_dir.glob("schedule_*.csv"))
    if not schedules:
        raise FileNotFoundError(f"No schedule_*.csv file found in {run_dir}")
    if len(schedules) > 1:
        names = ", ".join(path.name for path in schedules)
        raise ValueError(f"Multiple schedule files found in {run_dir}: {names}")
    return schedules[0]


def _representative_run(run_root: Path, run_ids: Iterable[str]) -> Path:
    for run_id in run_ids:
        run_dir = run_root / run_id
        if run_dir.is_dir() and list(run_dir.glob("schedule_*.csv")):
            return run_dir
    raise FileNotFoundError(f"No usable run folder found in {run_root} for {list(run_ids)}")


def _load_node_positions(graph_path: Path) -> dict[str, tuple[float, float]]:
    with graph_path.open("r", encoding="utf-8") as handle:
        graph = json.load(handle)
    return {
        str(node_id): (float(coord[0]), float(coord[1]))
        for node_id, coord in graph["node_dict"].items()
    }


def _load_schedule(schedule_path: Path, robot_id: str | None) -> pd.DataFrame:
    schedule = pd.read_csv(schedule_path)
    schedule["robot_id"] = schedule["robot_id"].astype(str)
    schedule["node_id"] = schedule["node_id"].astype(str)
    schedule = schedule.sort_values(["robot_id", "ETA"]).reset_index(drop=True)

    robot_ids = schedule["robot_id"].dropna().unique().tolist()
    if robot_id is not None:
        schedule = schedule[schedule["robot_id"] == str(robot_id)].copy()
        if schedule.empty:
            raise ValueError(f"Robot {robot_id!r} not found in {schedule_path}")
    elif len(robot_ids) == 1:
        schedule = schedule[schedule["robot_id"] == robot_ids[0]].copy()
    else:
        raise ValueError(
            f"Multiple robots found in {schedule_path}: {robot_ids}. Pass --robot-id."
        )

    return schedule.reset_index(drop=True)


def _schedule_with_coordinates(
    *,
    case_name: str,
    run_group: str,
    run_dir: Path,
    schedule_path: Path,
    schedule: pd.DataFrame,
    node_positions: dict[str, tuple[float, float]],
) -> pd.DataFrame:
    rows = []
    missing_nodes = []
    for visit_index, row in enumerate(schedule.itertuples(index=False), start=1):
        node_id = str(row.node_id)
        if node_id not in node_positions:
            missing_nodes.append(node_id)
            continue
        x, y = node_positions[node_id]
        rows.append(
            {
                "case": case_name,
                "run_group": run_group,
                "representative_run": run_dir.name,
                "schedule_file": schedule_path.name,
                "visit_index": visit_index,
                "robot_id": str(row.robot_id),
                "node_id": node_id,
                "ETA": float(row.ETA),
                "x": x,
                "y": y,
            }
        )

    if missing_nodes:
        unique_missing = sorted(set(missing_nodes))
        raise ValueError(f"{schedule_path} contains nodes missing from graph: {unique_missing}")

    return pd.DataFrame(rows)


def _plot_case(case_df: pd.DataFrame, output_path: Path) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    import matplotlib.pyplot as plt

    xs = case_df["x"].to_numpy(dtype=float)
    ys = case_df["y"].to_numpy(dtype=float)
    data_width = max(float(xs.max() - xs.min()), 0.5)
    data_height = max(float(ys.max() - ys.min()), 0.5)
    figure_height = min(5.2, max(4.2, 6.0 * data_height / data_width))
    fig, ax = plt.subplots(figsize=(6.0, figure_height))

    ax.plot(
        xs,
        ys,
        linestyle="--",
        linewidth=1.2,
        marker="o",
        markersize=7.5,
        markerfacecolor="#222222",
        markeredgecolor="#222222",
        color="#222222",
        label="scheduled path",
    )

    for node_id, node_rows in case_df.groupby("node_id", sort=False):
        x = float(node_rows["x"].iloc[0])
        y = float(node_rows["y"].iloc[0])
        ax.text(
            x + 0.035,
            y + 0.025,
            f"{node_id}",
            ha="left",
            va="bottom",
            fontsize=8,
        )

    case_name = str(case_df["case"].iloc[0])
    map_number = case_name.removeprefix("case_")
    ax.set_title(f"Map {map_number}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left")

    margin = 0.18
    ax.set_xlim(float(xs.min()) - margin, float(xs.max()) + margin)
    ax.set_ylim(float(ys.min()) - margin, float(ys.max()) + margin)
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--graph", type=Path, default=DEFAULT_GRAPH)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_RUN_ROOT / "analysis")
    parser.add_argument("--groups", nargs="+", default=list(DEFAULT_GROUPS))
    parser.add_argument("--robot-id", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    node_positions = _load_node_positions(args.graph)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    all_case_rows = []
    for case_index, group in enumerate(args.groups, start=1):
        case_name = f"case_{case_index}"
        run_ids = _parse_group(group)
        run_dir = _representative_run(args.run_root, run_ids)
        schedule_path = _find_schedule(run_dir)
        schedule = _load_schedule(schedule_path, args.robot_id)
        case_df = _schedule_with_coordinates(
            case_name=case_name,
            run_group=group,
            run_dir=run_dir,
            schedule_path=schedule_path,
            schedule=schedule,
            node_positions=node_positions,
        )

        plot_path = args.output_dir / f"{case_name}_scheduled_nodes.png"
        _plot_case(case_df, plot_path)
        print(f"Wrote {plot_path}")
        all_case_rows.append(case_df)

    summary = pd.concat(all_case_rows, ignore_index=True)
    summary_path = args.output_dir / "scheduled_nodes_with_coordinates.csv"
    summary.to_csv(summary_path, index=False)
    print(f"Wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
