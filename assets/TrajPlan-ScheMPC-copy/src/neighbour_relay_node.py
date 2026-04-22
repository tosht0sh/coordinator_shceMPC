"""Laptop-side neighbour-state relay for distributed multi-bot MPC.

This script listens for bot telemetry(current states + horizon), 
keeps the latest predicted trajectory for each configured robot, 
flattens the other robots into the solver's expected `other_robot_states` layout, 
and forwards the result back to each bot.
"""

from __future__ import annotations

import os
import pathlib
import socket
import sys
import time
from typing import Dict

import numpy as np

# TODO: maybe not create and entire list but only use find path of the 2 files we need now+
THIS_FILE = pathlib.Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[3]
TRAJPLAN_SRC = THIS_FILE.parent
MPC_RUNTIME_SRC = REPO_ROOT / "packages" / "mpc_runtime" / "src"
for path in (TRAJPLAN_SRC, MPC_RUNTIME_SRC):
    path_str = str(path)
    if path_str not in sys.path:
        sys.path.insert(0, path_str)

from configs import MpcConfiguration
from messages import NeighborStatesPacket, TelemetryPacket, packet_from_json, packet_to_wire

# IP where horizons are received from the bots. This is just the laptop ip adress.
HORIZON_RECEIVER_IP = "192.168.1.9"         # tosh
# HORIZON_RECEIVER_IP = "192.168.1.10"        # kim

RECEIVER_PORT = 5008        # receive on this port(bots send to <laptop_ip>:<this_port>)
NEIGHBOR_PORT = 5009        # send other robot states on this port
IDLE_SLEEP = 0.01

# Mapping from robot ids in the schedule (A1, A2, ...) to the bot IPs.
BOT_HOSTS: Dict[str, str] = {
    "A1": "192.168.1.11",       # duck1
    "A2": "192.168.1.12",       # duck2
    # "A3": "192.168.1.13",       # duck3
    # "A4": "192.168.1.14",       # duck4
}


class NeighbourRelayNode:
    """Receive telemetry and forward the latest neighbour horizons to every bot"""

    def __init__(self) -> None:
        # doing this to learn horizon length, number of other robots, and number of states from the config file.
        self.root_dir = THIS_FILE.parents[1]
        self.config_dir = self.root_dir / "config"
        config_mpc_path = self.config_dir / os.getenv("MPC_CFG_NAME", "mpc_fast.yaml")
        self.config_mpc = MpcConfiguration.from_yaml(str(config_mpc_path))

        # set disctionary of hosts
        self.bot_hosts = dict(BOT_HOSTS)
        if not self.bot_hosts:
            raise ValueError("Set BOT_HOSTS in neighbour_relay_node.py before running the relay.")
        self.robot_ids = sorted(self.bot_hosts.keys())

        # latest data received
        self.latest_packet: Dict[str, TelemetryPacket] = {}
        self._warned_unknown_robot: set[str] = set()

        # connect to socket to receive data
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.recv_sock.bind((HORIZON_RECEIVER_IP, RECEIVER_PORT))
        self.recv_sock.setblocking(False)

        # socket connection to send neighbour data to bots
        self.neighbour_send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.neighbour_send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        print(f"[NeighbourRelay] Listening for telemetry on {HORIZON_RECEIVER_IP}:{RECEIVER_PORT}")
        print(f"[NeighbourRelay] Forwarding neighbor trajectories on UDP port {NEIGHBOR_PORT}")

    def _poll_packets(self, max_packets: int = 64) -> None:
        """ Handle received JSON from bot. """
        for _ in range(max_packets):
            try:
                packet, _ = self.recv_sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                print(f"[NeighbourRelay] UDP receive failed: {exc}")
                return

            try:
                decoded = packet_from_json(packet.decode("utf-8"))      # decode JSON packet to object
            except Exception as exc:
                print(f"[NeighbourRelay] Invalid packet: {exc}")
                continue

            if not isinstance(decoded, TelemetryPacket):
                continue
            self._handle_telemetry(decoded)     # store and send updated telemtry

    def _prediction_from_packet(self, packet: TelemetryPacket) -> np.ndarray:
        """Return one robot's predicted trajectory(horizon) as a dense `(N_hor+1, ns)` array."""
        """" Cleans the array length as there are problems due to no horizon
            - adds the current state as a repeated value,
            - adds the final state from horizon as a repeated value if horizon ends prematurely
            - truncates horizon if longer than expected  """

        ns = self.config_mpc.ns                         # number of states
        horizon_len = self.config_mpc.N_hor + 1         # horizon length + 1

        pose = np.asarray(packet.pose[:ns], dtype=float)
        if pose.shape[0] < ns:
            pose = np.pad(pose, (0, ns - pose.shape[0]), constant_values=0.0)

        pred_states = np.asarray(packet.pred_states, dtype=float) if packet.pred_states else np.empty((0, ns))
        if pred_states.ndim != 2 or pred_states.shape[0] == 0 or pred_states.shape[1] < ns:
            return np.tile(pose, (horizon_len, 1))

        pred_states = pred_states[:, :ns]
        if pred_states.shape[0] >= horizon_len:
            return pred_states[:horizon_len]

        pad = np.repeat(pred_states[-1:, :], horizon_len - pred_states.shape[0], axis=0)    # padding if necessary
        return np.vstack((pred_states, pad))

    def _build_other_robot_states(self, target_robot_id: str) -> tuple[list[float], list[str]]:
        """Flatten every other robot trajectory into the solver's expected layout."""

        ns = self.config_mpc.ns                         # number of states
        horizon_len = self.config_mpc.N_hor + 1         # horizon length
        slot_len = ns * horizon_len                     # length of slot for 1 robot
        max_other = self.config_mpc.Nother              # number of other robots(total robotos -1)

        if max_other <= 0:
            return [], []

        flattened: list[float] = []     # list of all horizons of the neighbour bots
        source_robot_ids: list[str] = []
        
        for rid in self.robot_ids:
            if rid == target_robot_id:
                continue

            neighbor_telemetry = self.latest_packet.get(rid)    # get latest telemetry for robot we do not send to
            if neighbor_telemetry is None:
                continue
            if len(source_robot_ids) >= max_other:
                break
            pred = self._prediction_from_packet(neighbor_telemetry)     
            flattened.extend(pred.reshape(-1).tolist())
            source_robot_ids.append(rid)

        while len(flattened) < slot_len * max_other:
            flattened.extend([-10.0] * slot_len)        # safety to prevent blocking due to less bots than config

        return flattened[: slot_len * max_other], source_robot_ids

    def _relay_neighbor_states(self) -> None:
        """ Create other robot states and send to bots(neighbour state update) """
        for rid in self.robot_ids:
            host = self.bot_hosts[rid]
            vector, source_ids = self._build_other_robot_states(rid)        # build vector of robots other than current one
            schedule_id = self.latest_packet[rid].schedule_id if rid in self.latest_packet else "schedule-default"
            packet = NeighborStatesPacket(
                robot_id=rid,
                schedule_id=schedule_id,
                t=time.time(),
                other_robot_states=vector,
                source_robot_ids=source_ids,
            )
            try:
                self.neighbour_send_sock.sendto(packet_to_wire(packet), (host, NEIGHBOR_PORT))
            except OSError as exc:
                print(f"[NeighbourRelay] Failed to relay neighbor states to {rid} at {host}:{NEIGHBOR_PORT}: {exc}")

    def _handle_telemetry(self, packet: TelemetryPacket) -> None:
        """ Update and send new telemetry data to other robots """

        rid = packet.robot_id       # robot id of new data
        if rid not in self.bot_hosts:
            if rid not in self._warned_unknown_robot:
                print(f"[NeighbourRelay] Ignoring telemetry for unknown robot id {rid}.")
                self._warned_unknown_robot.add(rid)
            return

        self.latest_packet[rid] = packet        # store this new telemetry into dict
        self._relay_neighbor_states()           # send updated neighbor data to other bots

    def run(self) -> None:
        try:
            while True:
                self._poll_packets()
                time.sleep(max(0.0, IDLE_SLEEP))
        except KeyboardInterrupt:
            pass
        finally:
            self.recv_sock.close()
            self.neighbour_send_sock.close()


if __name__ == "__main__":
    relay = NeighbourRelayNode()
    relay.run() 