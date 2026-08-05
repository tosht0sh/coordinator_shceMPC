"""Laptop-side telemetry visualizer and optional shadow simulation.

This script listens for bot telemetry, updates the live MPC plot, can run the shadow
unicycle simulation used for motion comparison, and saves the monitored node timeline on
shutdown.

Unlike monitor_node.py, this file does not relay neighbor trajectories back to the bots.
"""

from __future__ import annotations

import json
import os
import pathlib
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd  # type: ignore


THIS_FILE = pathlib.Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[3]
TRAJPLAN_SRC = THIS_FILE.parent
MPC_RUNTIME_SRC = REPO_ROOT / "packages" / "mpc_runtime" / "src"
for path in (TRAJPLAN_SRC, MPC_RUNTIME_SRC):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from basic_motion_model.motion_model import UnicycleModel
from configs import CircularRobotSpecification, MpcConfiguration
from messages import TelemetryPacket, packet_from_json
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator
from visualizer.mpc_plot import MpcPlotInLoop
from visualizer.object import CircularVehicleVisualizer


DATA_NAME = os.getenv("MPC_DATA_NAME", "schedule_demo2_data")
ENV_FOLDER = os.getenv("MPC_ENV_FOLDER", "")
SCHEDULE_VARIANT = os.getenv("MPC_SCHEDULE_VARIANT", "SingleRobot")
TELEMETRY_BIND_IP = os.getenv("MPC_TELEMETRY_BIND_IP", "0.0.0.0")
TELEMETRY_PORT = int(os.getenv("MPC_TELEMETRY_PORT", "5008"))
MONITOR_AUTORUN = os.getenv("MPC_MONITOR_AUTORUN", "1").strip().lower() in {"1", "true", "yes", "on"}
MAP_ONLY = os.getenv("MPC_MONITOR_MAP_ONLY", "1").strip().lower() in {"1", "true", "yes", "on"}
SHADOW_SIM = os.getenv("MPC_SHADOW_SIM", "1").strip().lower() in {"1", "true", "yes", "on"}
OUTPUT_CSV = os.getenv("MPC_MONITOR_ACTUAL_CSV", "Actual_monitor.csv")
IDLE_TIMEOUT = float(os.getenv("MPC_MONITOR_IDLE_TIMEOUT", "0.1"))


def _schedule_paths(data_dir: pathlib.Path, variant: str) -> Tuple[pathlib.Path, pathlib.Path]:
    mapping = {
        "Original": ("schedule.csv", "robot_start.json"),
        "SingleRobot": ("schedule_SingleRobot.csv", "robot_start_SingleRobot.json"),
        "TwoRobots": ("schedule_TwoRobots.csv", "robot_start_TwoRobots.json"),
    }
    schedule_name, start_name = mapping.get(variant, mapping["SingleRobot"])
    return data_dir / schedule_name, data_dir / start_name


class VisualizerNode:
    """Receive telemetry, update plots, and optionally run the shadow simulator."""

    def __init__(self, env_folder: str) -> None:
        if not env_folder:
            raise ValueError("Set MPC_ENV_FOLDER to the scenario folder inside the data directory.")

        self.root_dir = THIS_FILE.parents[1]
        self.data_dir = self.root_dir / "data" / DATA_NAME
        self.config_dir = self.root_dir / "config"
        self.env_folder = env_folder

        config_mpc_path = self.config_dir / os.getenv("MPC_CFG_NAME", "mpc_fast_sim.yaml")
        config_robot_path = self.config_dir / "robot_spec.yaml"
        self.config_mpc = MpcConfiguration.from_yaml(str(config_mpc_path))
        self.config_robot = CircularRobotSpecification.from_yaml(str(config_robot_path))

        schedule_path, start_path = _schedule_paths(self.data_dir, SCHEDULE_VARIANT)
        map_path = self.data_dir / env_folder / "map.json"
        graph_path = self.data_dir / env_folder / "graph.json"
        with open(start_path, "r", encoding="utf-8") as handle:
            self.robot_starts = json.load(handle)

        self.gpc = GlobalPathCoordinator.from_csv(str(schedule_path))
        self.gpc.load_graph_from_json(str(graph_path))
        self.gpc.load_map_from_json(
            str(map_path),
            inflation_margin=self.config_robot.vehicle_width + self.config_robot.vehicle_margin,
        )
        self.robot_id_lookup = {str(rid): rid for rid in self.gpc.robot_ids}
        self.robot_ids = list(self.robot_id_lookup.keys())

        boundary_coords = self.gpc.current_map.boundary_coords
        map_width = max(np.asarray(boundary_coords)[:, 0]) - min(np.asarray(boundary_coords)[:, 0])
        map_height = max(np.asarray(boundary_coords)[:, 1]) - min(np.asarray(boundary_coords)[:, 1])
        self.plotter = MpcPlotInLoop(self.config_robot, map_only=MAP_ONLY, fig_ratio=(map_width / map_height))
        self.plotter.plot_in_loop_pre(self.gpc.current_map, graph_manager=self.gpc.current_graph)

        self.color_list = [
            "#0072B2", "#D55E00", "#009E73", "#F0E442", "#56B4E9",
            "#E69F00", "#CC79A7", "#0072B2", "#D55E00", "#009E73",
        ]
        self.visualizers: Dict[str, CircularVehicleVisualizer] = {}
        self.shadow_visualizers: Dict[str, CircularVehicleVisualizer] = {}
        self.shadow_model = UnicycleModel(sampling_time=self.config_mpc.ts)
        self.shadow_states: Dict[str, np.ndarray] = {}
        self.actual_timetable: Dict[str, List[Tuple[float, Optional[object]]]] = {rid: [] for rid in self.robot_ids}
        self.latest_packet: Dict[str, TelemetryPacket] = {}

        for index, rid in enumerate(self.robot_ids):
            source_robot_id = self.robot_id_lookup[rid]
            path_coords, _ = self.gpc.get_robot_schedule(source_robot_id)
            start_state = np.asarray(self.robot_starts[str(source_robot_id)], dtype=float)
            if len(path_coords) >= 2:
                prev_coord = path_coords[-2]
                end_coord = path_coords[-1]
                goal_heading = np.arctan2(end_coord[1] - prev_coord[1], end_coord[0] - prev_coord[0])
            else:
                end_coord = path_coords[-1]
                goal_heading = float(start_state[2])
            goal_state = np.array([end_coord[0], end_coord[1], goal_heading], dtype=float)

            self.plotter.add_object_to_pre(
                rid,
                None,
                start_state,
                goal_state,
                color=self.color_list[index % len(self.color_list)],
            )
            visualizer = CircularVehicleVisualizer(self.config_robot.vehicle_width, indicate_angle=True)
            visualizer.plot(self.plotter.map_ax, *start_state)
            self.visualizers[rid] = visualizer

            if SHADOW_SIM:
                shadow_id = f"{rid}_shadow"
                self.plotter.add_object_to_pre(shadow_id, None, start_state, goal_state, color="#999999")
                shadow_visualizer = CircularVehicleVisualizer(self.config_robot.vehicle_width, indicate_angle=True)
                shadow_visualizer.plot(self.plotter.map_ax, *start_state)
                self.shadow_visualizers[rid] = shadow_visualizer
                self.shadow_states[rid] = start_state.copy()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((TELEMETRY_BIND_IP, TELEMETRY_PORT))
        self.sock.setblocking(False)

        print(f"[Visualizer] Listening for telemetry on {TELEMETRY_BIND_IP}:{TELEMETRY_PORT}")

    def _poll_packets(self, max_packets: int = 64) -> None:
        for _ in range(max_packets):
            try:
                packet, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                print(f"[Visualizer] UDP receive failed: {exc}")
                return

            try:
                decoded = packet_from_json(packet.decode("utf-8"))
            except Exception as exc:
                print(f"[Visualizer] Invalid packet: {exc}")
                continue

            if not isinstance(decoded, TelemetryPacket):
                continue
            self._handle_telemetry(decoded)

    def _handle_telemetry(self, packet: TelemetryPacket) -> None:
        rid = packet.robot_id
        if rid not in self.visualizers:
            return

        ns = self.config_mpc.ns
        pose = np.asarray(packet.pose[:ns], dtype=float)
        if pose.shape[0] < ns or not np.all(np.isfinite(pose)):
            print(f"[Visualizer] Ignoring invalid pose for {rid}: {packet.pose}")
            return

        action = np.asarray(packet.action, dtype=float) if packet.action else np.zeros(2, dtype=float)
        pred_states = np.asarray(packet.pred_states, dtype=float) if packet.pred_states else np.empty((0, ns))
        current_refs = np.asarray(packet.current_refs, dtype=float) if packet.current_refs else np.empty((0, ns))
        cost = 0.0 if packet.cost is None else float(packet.cost)

        self.plotter.update_plot(rid, packet.t, action, pose, cost, pred_states, current_refs)
        self.visualizers[rid].update(*pose)
        self.latest_packet[rid] = packet

        if packet.current_target_node is not None:
            node_id = self.gpc.get_node_id(tuple(packet.current_target_node))
            history = self.actual_timetable[rid]
            if not history or history[-1][1] != node_id:
                history.append((packet.t, node_id))
            else:
                history[-1] = (packet.t, node_id)

        if SHADOW_SIM:
            shadow_state = self.shadow_states[rid]
            shadow_state = self.shadow_model(shadow_state, action, self.config_mpc.ts)
            self.shadow_states[rid] = shadow_state
            shadow_id = f"{rid}_shadow"
            self.plotter.update_plot(
                shadow_id,
                packet.t,
                action,
                shadow_state,
                cost,
                np.empty((0, ns)),
                np.empty((0, ns)),
            )
            self.shadow_visualizers[rid].update(*shadow_state)

    def _save_actual_schedule(self) -> None:
        rows = []
        for rid, schedule in self.actual_timetable.items():
            for eta, node_id in schedule:
                rows.append({"robot_id": rid, "node_id": node_id, "ETA": eta})
        if not rows:
            return
        df = pd.DataFrame(rows).sort_values(["robot_id", "ETA"])
        output_path = self.data_dir / OUTPUT_CSV
        df.to_csv(output_path, index=False)
        print(f"Saved monitored schedule to {output_path}")

    def run(self) -> None:
        self.plotter.show()
        last_plot_time = 0.0
        try:
            while True:
                self._poll_packets()
                now = time.monotonic()
                if now - last_plot_time >= max(IDLE_TIMEOUT, self.config_mpc.ts):
                    latest_t = 0.0
                    for packet in self.latest_packet.values():
                        latest_t = max(latest_t, packet.t)
                    self.plotter.plot_in_loop(time=latest_t, autorun=MONITOR_AUTORUN, zoom_in=None)
                    last_plot_time = now
        except KeyboardInterrupt:
            pass
        finally:
            self._save_actual_schedule()
            self.plotter.close()
            self.sock.close()


if __name__ == "__main__":
    visualizer = VisualizerNode(ENV_FOLDER)
    visualizer.run()
