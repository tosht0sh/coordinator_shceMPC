"""Laptop-side monitor, shadow simulation, and multi-bot neighbor-state relay.

This script listens for bot telemetry, updates the visualization stack, and can run a
lightweight shadow unicycle model to compare model motion to real motion.

For multi-bot MPC we also use it as a relay:
- each bot sends telemetry to the laptop
- the laptop collects the latest predicted trajectory from every bot
- the laptop forwards each bot a flattened view of all *other* robots

That keeps the MPC solve distributed on the bots while avoiding direct bot-to-bot
network plumbing.
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
from messages import NeighborStatesPacket, TelemetryPacket, CoordinatorModePacket, packet_from_json, packet_to_wire
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator
from visualizer.mpc_plot import MpcPlotInLoop
from visualizer.object import CircularVehicleVisualizer

from coordinator.coordinator import Coordinator, CROSSING_RELEASE_RADIUS


DATA_NAME = os.getenv("MPC_DATA_NAME", "schedule_demo2_data")
ENV_FOLDER = os.getenv("MPC_ENV_FOLDER", "")
SCHEDULE_VARIANT = os.getenv("MPC_SCHEDULE_VARIANT", "SingleRobot")
TELEMETRY_BIND_IP = os.getenv("MPC_TELEMETRY_BIND_IP", "0.0.0.0")
TELEMETRY_PORT = int(os.getenv("MPC_TELEMETRY_PORT", "5008"))
NEIGHBOR_RELAY = os.getenv("MPC_NEIGHBOR_RELAY", "1").strip().lower() in {"1", "true", "yes", "on"}
NEIGHBOR_PORT = int(os.getenv("MPC_NEIGHBOR_PORT", "5009"))
DEFAULT_BOT_PORT = int(os.getenv("MPC_DEFAULT_BOT_PORT", "5007"))
MONITOR_AUTORUN = os.getenv("MPC_MONITOR_AUTORUN", "1").strip().lower() in {"1", "true", "yes", "on"}
MAP_ONLY = os.getenv("MPC_MONITOR_MAP_ONLY", "1").strip().lower() in {"1", "true", "yes", "on"}
SHADOW_SIM = False #os.getenv("MPC_SHADOW_SIM", "1").strip().lower() in {"1", "true", "yes", "on"}
OUTPUT_CSV = os.getenv("MPC_MONITOR_ACTUAL_CSV", "Actual_monitor.csv")
IDLE_TIMEOUT = float(os.getenv("MPC_MONITOR_IDLE_TIMEOUT", "0.1"))

COORDINATOR_PORT = int(os.getenv("COORDINATOR_PORT", "5010"))


def _load_endpoint_map() -> Dict[str, Tuple[str, int]]:
    """Parse the same robot-id -> host mapping used by the scheduler dispatcher."""

    raw = os.getenv("MPC_BOT_ENDPOINTS", "").strip()
    endpoint_map: Dict[str, Tuple[str, int]] = {}
    if not raw:
        return endpoint_map

    loaded = json.loads(raw)
    if not isinstance(loaded, dict):
        raise ValueError("MPC_BOT_ENDPOINTS must be a JSON object.")

    for robot_id, value in loaded.items():
        rid = str(robot_id)
        if isinstance(value, str):
            host, _, port = value.partition(":")
            endpoint_map[rid] = (host, int(port or DEFAULT_BOT_PORT))
        elif isinstance(value, dict):
            host = str(value["host"])
            port = int(value.get("port", DEFAULT_BOT_PORT))
            endpoint_map[rid] = (host, port)
        else:
            raise ValueError(f"Unsupported endpoint spec for robot {rid}: {value!r}")
    return endpoint_map


def _schedule_paths(data_dir: pathlib.Path, variant: str) -> Tuple[pathlib.Path, pathlib.Path]:
    mapping = {
        "Original": ("schedule.csv", "robot_start.json"),
        "SingleRobot": ("schedule_SingleRobot.csv", "robot_start_SingleRobot.json"),
        "TwoRobots": ("schedule_TwoRobots.csv", "robot_start_TwoRobots.json"),
        "MultiRobot": ("schedule_MultiRobots.csv", "robot_start_MultiRobots.json"),
        "CoordScene1": ("schedule_CoordScene1.csv", "robot_start_CoordScene1.json"),
        "CoordScene2": ("schedule_CoordScene2.csv", "robot_start_CoordScene2.json"),
        "CoordScene3": ("schedule_CoordScene3.csv", "robot_start_CoordScene3.json"),
    }
    schedule_name, start_name = mapping.get(variant, mapping["SingleRobot"])
    return data_dir / schedule_name, data_dir / start_name


class MonitorNode:
    """Receive telemetry, update plots, and relay neighbor trajectories to the bots."""

    def __init__(self, env_folder: str) -> None:
        if not env_folder:
            raise ValueError("Set MPC_ENV_FOLDER to the scenario folder inside the data directory.")

        self.root_dir = THIS_FILE.parents[1]
        self.data_dir = self.root_dir / "data" / DATA_NAME
        self.config_dir = self.root_dir / "config"
        self.env_folder = env_folder
        self.endpoint_map = _load_endpoint_map()
        self.neighbor_relay_enabled = NEIGHBOR_RELAY and bool(self.endpoint_map)

        config_mpc_path = self.config_dir / os.getenv("MPC_CFG_NAME", "mpc_fast.yaml")
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
        self.last_packet_time: Dict[str, float] = {}
        self.actual_timetable: Dict[str, List[Tuple[float, Optional[object]]]] = {rid: [] for rid in self.robot_ids}
        self.latest_packet: Dict[str, TelemetryPacket] = {}
        self._relay_warned_missing_endpoint: set[str] = set()

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

            self.plotter.add_object_to_pre(rid, None, start_state, goal_state, color=self.color_list[index % len(self.color_list)])
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
        self.peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.neighbor_relay_enabled else None

        self.coord = Coordinator.from_csv(str(schedule_path))
        self.coord_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.coord.load_graph_from_json(str(graph_path))

        self.coord_shifted_targets = {}
        self.coord_shifted_targets_compensated = {}
        self.coord_handoff_planners = {}

        self.coord_sequence = 0

        # TODO: solve this, neeeded for coord scene 3
        # self.coord.save_jobs(jobs_list)
        # self.coord.save_initial_route(routes)

        self.coord_modes = {
            rid: "WAIT" for rid in self.robot_ids
        }

        print(f"[Monitor] Listening for telemetry on {TELEMETRY_BIND_IP}:{TELEMETRY_PORT}")
        if self.neighbor_relay_enabled:
            print(f"[Monitor] Relaying neighbor trajectories to bots on UDP port {NEIGHBOR_PORT}")
        elif NEIGHBOR_RELAY:
            print("[Monitor] Neighbor relay requested but MPC_BOT_ENDPOINTS is empty; relay disabled.")

    def _poll_packets(self, max_packets: int = 64) -> None:
        for _ in range(max_packets):
            try:
                packet, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as exc:
                print(f"[Monitor] UDP receive failed: {exc}")
                break

            try:
                decoded = packet_from_json(packet.decode("utf-8"))
            except Exception as exc:
                print(f"[Monitor] Invalid packet: {exc}")
                continue

            if not isinstance(decoded, TelemetryPacket):
                continue
            self._handle_telemetry(decoded)

    def _prediction_from_packet(self, packet: TelemetryPacket) -> np.ndarray:
        """Return one robot's predicted trajectory as a dense `(N_hor+1, ns)` array.

        If a bot has not yet published predictions, we conservatively repeat its
        current pose across the horizon so the receiving bot still sees a valid
        obstacle trajectory.
        """

        ns = self.config_mpc.ns
        horizon_len = self.config_mpc.N_hor + 1
        pose = np.asarray(packet.pose[:ns], dtype=float)
        pred_states = np.asarray(packet.pred_states, dtype=float) if packet.pred_states else np.empty((0, ns))

        if pred_states.ndim != 2 or pred_states.shape[0] == 0:
            return np.tile(pose, (horizon_len, 1))

        pred_states = pred_states[:, :ns]
        if pred_states.shape[0] >= horizon_len:
            return pred_states[:horizon_len]

        pad = np.repeat(pred_states[-1:, :], horizon_len - pred_states.shape[0], axis=0)
        return np.vstack((pred_states, pad))

    def _build_other_robot_states(self, target_robot_id: str) -> tuple[list[float], list[str]]:
        """Flatten every *other* robot trajectory into the solver's expected layout."""

        ns = self.config_mpc.ns
        horizon_len = self.config_mpc.N_hor + 1
        slot_len = ns * horizon_len
        max_other = self.config_mpc.Nother

        if max_other <= 0:
            return [], []

        flattened: list[float] = []
        source_robot_ids: list[str] = []
        for rid in sorted(self.robot_ids):
            if rid == target_robot_id:
                continue
            packet = self.latest_packet.get(rid)
            if packet is None:
                continue
            if len(source_robot_ids) >= max_other:
                break
            pred = self._prediction_from_packet(packet)
            flattened.extend(pred.reshape(-1).tolist())
            source_robot_ids.append(rid)

        while len(flattened) < slot_len * max_other:
            flattened.extend([-10.0] * slot_len)

        return flattened[: slot_len * max_other], source_robot_ids

    def _relay_neighbor_states(self) -> None:
        if not self.neighbor_relay_enabled or self.peer_sock is None:
            return

        for rid in self.robot_ids:
            endpoint = self.endpoint_map.get(rid)
            if endpoint is None:
                if rid not in self._relay_warned_missing_endpoint:
                    print(f"[Monitor] No endpoint configured for {rid}; neighbor relay skipped for that robot.")
                    self._relay_warned_missing_endpoint.add(rid)
                continue

            host, _ = endpoint
            vector, source_ids = self._build_other_robot_states(rid)
            schedule_id = self.latest_packet[rid].schedule_id if rid in self.latest_packet else "schedule-default"
            packet = NeighborStatesPacket(
                robot_id=rid,
                schedule_id=schedule_id,
                t=time.time(),
                other_robot_states=vector,
                source_robot_ids=source_ids,
            )
            try:
                self.peer_sock.sendto(packet_to_wire(packet), (host, NEIGHBOR_PORT))
            except OSError as exc:
                print(f"[Monitor] Failed to relay neighbor states to {rid} at {host}:{NEIGHBOR_PORT}: {exc}")

    def _handle_telemetry(self, packet: TelemetryPacket) -> None:
        rid = packet.robot_id
        if rid not in self.visualizers:
            return

        pose = np.asarray(packet.pose, dtype=float)
        action = np.asarray(packet.action, dtype=float) if packet.action else np.zeros(2, dtype=float)
        pred_states = np.asarray(packet.pred_states, dtype=float) if packet.pred_states else np.empty((0, 3))
        current_refs = np.asarray(packet.current_refs, dtype=float) if packet.current_refs else np.empty((0, 3))
        cost = 0.0 if packet.cost is None else float(packet.cost)

        target_node_id = None

        self.plotter.update_plot(rid, packet.t, action, pose, cost, pred_states, current_refs)
        self.visualizers[rid].update(*pose)
        self.latest_packet[rid] = packet
        self.last_packet_time[rid] = time.monotonic()

        if packet.current_target_node is not None:
            node_id = self.gpc.get_node_id(tuple(packet.current_target_node))
            history = self.actual_timetable[rid]
            if not history or history[-1][1] != node_id:
                history.append((packet.t, node_id))
            else:
                history[-1] = (packet.t, node_id)

            target_coord = tuple(packet.current_target_node)
            # print(f"Target coord: {packet.current_target_node}")
            target_node_id = self.gpc.get_node_id(packet.current_target_node)
            # print(f"Node_id: {target_node_id}")

        self.coord.update_horizon(rid, self._prediction_from_packet(packet))
        self.coord.update_curr_pose(rid, pose)

        if rid in self.coord_shifted_targets_compensated:
            compensation = self.coord_shifted_targets_compensated[rid]
            shifted_target = compensation["new_target"]
            shifted_xy = np.asarray(shifted_target, dtype=float)

            if np.linalg.norm(pose[:2] - shifted_xy) <= CROSSING_RELEASE_RADIUS:
                self.coord.release_crossing_robot(rid)
                self.coord_shifted_targets_compensated.pop(rid, None)
                self.coord_shifted_targets.pop(rid, None)
                print(f"[Monitor] Released crossing conflict for {rid} at shifted target {list(shifted_target)}")
            elif target_node_id is not None and target_node_id != compensation["parent_node"]:
                self.coord_shifted_targets_compensated.pop(rid, None)
                self.coord_shifted_targets.pop(rid, None)
                self.coord.update_target_nodes(rid, target_node_id)
            else:
                self.coord.update_target_nodes(rid, compensation["parent_node"])
        elif target_node_id is not None:
            self.coord.update_target_nodes(rid, target_node_id)

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
                np.empty((0, 3)),
                np.empty((0, 3)),
            )
            self.shadow_visualizers[rid].update(*shadow_state)

        self._relay_neighbor_states()

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

    def _run_coordinator(self) -> None:
        if not all(rid in self.last_packet_time for rid in self.robot_ids):
            return

        coord_time = max(packet.t for packet in self.latest_packet.values())
        self.coord.update_time(coord_time)

        decisions = self.coord.validate() or {}
        released_crossing = False
        for source_rid, decision in decisions.items():
            if decision["mode"] != "crossing" or decision.get("target_coord") is None:
                continue

            rid = str(source_rid)
            packet = self.latest_packet.get(rid)
            if packet is None:
                continue

            pose_xy = np.asarray(packet.pose[:2], dtype=float)
            target_xy = np.asarray(decision["target_coord"], dtype=float)
            if np.linalg.norm(pose_xy - target_xy) <= CROSSING_RELEASE_RADIUS:
                self.coord.release_crossing_robot(rid)
                self.coord_shifted_targets_compensated.pop(rid, None)
                self.coord_shifted_targets.pop(rid, None)
                released_crossing = True
                print(f"[Monitor] Released crossing conflict for {rid} at shifted target {target_xy.tolist()}")

        if released_crossing:
            decisions = self.coord.validate() or {}

        requested_modes = {
            rid: {"mode": "WORK", "target_coord": None}
            for rid in self.robot_ids
        }

        # change mode to WAIT for the robot wherever needed
        for source_rid, decicsion in decisions.items():
            rid = str(source_rid)

            if decicsion["mode"] == "stopped":
                requested_modes[rid] = {"mode": "WAIT", "target_coord": None}
            elif decicsion["mode"] == "crossing":
                target_coord = tuple(decicsion["target_coord"])
                parent_node = self.coord._current_target_node_ids.get(rid)
                compensation = self.coord_shifted_targets_compensated.get(rid)

                if compensation is None:
                    self.coord_shifted_targets_compensated[rid] = {
                        "parent_node": parent_node,
                        "new_target": target_coord,
                    }
                else:
                    compensation["new_target"] = target_coord

                self.coord_shifted_targets[rid] = target_coord
                requested_modes[rid] = {
                    "mode": "CROSSING",
                    "target_coord": list(target_coord),
                }
            else:
                self.coord_shifted_targets_compensated.pop(rid, None)
                self.coord_shifted_targets.pop(rid, None)
                requested_modes[rid] = {"mode": "WORK", "target_coord": None}

        self._send_coordinator_modes(requested_modes)

    def _send_coordinator_modes(self, requested_modes) -> None:
        if self.coord_sock is None:
            return

        for rid, request in requested_modes.items():
            endpoint = self.endpoint_map.get(rid)

            if endpoint is None:
                if rid not in self._relay_warned_missing_endpoint:
                    print(f"[Monitor] No coordinator endpoint for {rid}. Mode command skipped")

                    self._relay_warned_missing_endpoint.add(rid)
                continue

            host, _ = endpoint

            packet = CoordinatorModePacket(robot_id=rid,
                        mode=request["mode"],
                        sequence=self.coord_sequence,
                        sent_at=time.time(),
                        target_coord=request["target_coord"],
                    )

            self.coord_sock.sendto(packet_to_wire(packet), (host, COORDINATOR_PORT))

        self.coord_sequence += 1

    def run(self) -> None:
        self.plotter.show()
        last_plot_time = 0.0
        try:
            while True:
                self._poll_packets()
                self._run_coordinator()
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
            self.coord_sock.close()
            if self.peer_sock is not None:
                self.peer_sock.close()


if __name__ == "__main__":
    monitor = MonitorNode(ENV_FOLDER)
    monitor.run()
