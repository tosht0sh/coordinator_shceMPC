"""Laptop-side scheduler dispatcher.

This script loads the scenario map/graph/schedule, extracts one schedule per robot,
and sends the required map plus schedule payloads to each bot runtime node.
"""

from __future__ import annotations

import json
import os
import pathlib
import socket
import sys
from typing import Any, Dict, Optional, Tuple


THIS_FILE = pathlib.Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[3]
TRAJPLAN_SRC = THIS_FILE.parent
MPC_RUNTIME_SRC = REPO_ROOT / "packages" / "mpc_runtime" / "src"
for path in (TRAJPLAN_SRC, MPC_RUNTIME_SRC):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from configs import CircularRobotSpecification
from messages import MapPacket, SchedulePacket, packet_to_wire
from pkg_motion_plan.global_path_coordinate import GlobalPathCoordinator


DATA_NAME = os.getenv("MPC_DATA_NAME", "schedule_demo2_data")
CFG_FNAME = os.getenv("MPC_CFG_NAME", "mpc_fast.yaml")
ENV_FOLDER = os.getenv("MPC_ENV_FOLDER", "")
SCHEDULE_VARIANT = os.getenv("MPC_SCHEDULE_VARIANT", "SingleRobot")
SEND_MAP = os.getenv("MPC_SEND_MAP", "1").strip().lower() in {"1", "true", "yes", "on"}
DISPATCH_TIMEOUT = float(os.getenv("MPC_DISPATCH_TIMEOUT", "5.0"))
DEFAULT_BOT_PORT = int(os.getenv("MPC_DEFAULT_BOT_PORT", "5007"))
DEFAULT_BOT_HOST = os.getenv("MPC_DEFAULT_BOT_HOST", "192.168.1.12")



def _load_endpoint_map() -> Dict[str, Tuple[str, int]]:
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
    }
    schedule_name, start_name = mapping.get(variant, mapping["SingleRobot"])
    return data_dir / schedule_name, data_dir / start_name


class SchedulerDispatcher:
    """Build and send robot-specific startup packets from the laptop."""

    def __init__(self, env_folder: str) -> None:
        if not env_folder:
            raise ValueError("Set MPC_ENV_FOLDER to the scenario folder inside the data directory.")

        self.root_dir = THIS_FILE.parents[1]
        self.data_dir = self.root_dir / "data" / DATA_NAME
        self.config_dir = self.root_dir / "config"
        self.env_folder = env_folder
        self.endpoint_map = _load_endpoint_map()

        config_robot_path = self.config_dir / "robot_spec.yaml"
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
        self.robot_ids = self.gpc.robot_ids

    def _endpoint_for(self, robot_id: str) -> Tuple[str, int]:
        if robot_id in self.endpoint_map:
            return self.endpoint_map[robot_id]
        if DEFAULT_BOT_HOST:
            return DEFAULT_BOT_HOST, DEFAULT_BOT_PORT
        raise ValueError(
            f"No endpoint configured for robot {robot_id}. Set MPC_BOT_ENDPOINTS or MPC_DEFAULT_BOT_HOST."
        )

    def build_map_packet(self) -> MapPacket:
        return MapPacket(
            map_id=self.env_folder,
            boundary_coords=[list(point) for point in self.gpc.inflated_map.boundary_coords],
            static_obstacles=[[list(point) for point in polygon] for polygon in self.gpc.inflated_map.obstacle_coords_list],
        )

    def build_schedule_packet(self, robot_id: Any) -> SchedulePacket:
        path_coords, path_times = self.gpc.get_robot_schedule(robot_id)
        start_state = self.robot_starts[str(robot_id)]
        return SchedulePacket(
            robot_id=str(robot_id),
            schedule_id=f"{self.env_folder}:{SCHEDULE_VARIANT}",
            start_state=[float(v) for v in start_state],
            path_coords=[[float(x), float(y)] for x, y in path_coords],
            path_times=None if path_times is None else [float(t) for t in path_times],
            effective_from=0.0,
        )

    def dispatch(self) -> None:
        map_packet = self.build_map_packet()
        for robot_id in self.robot_ids:
            rid = str(robot_id)
            host = "192.168.1.11" # Robots ip
            port = 5007
            # host, port = self._endpoint_for(rid)

            schedule_packet = self.build_schedule_packet(robot_id)
            with socket.create_connection((host, port), timeout=DISPATCH_TIMEOUT) as sock:
                if SEND_MAP:
                    sock.sendall(packet_to_wire(map_packet))
                sock.sendall(packet_to_wire(schedule_packet))
            print(f"Dispatched {schedule_packet.schedule_id} to {rid} at {host}:{port}")


if __name__ == "__main__":
    dispatcher = SchedulerDispatcher(ENV_FOLDER)
    dispatcher.dispatch()
