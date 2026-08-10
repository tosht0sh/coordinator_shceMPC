#!/usr/bin/env python3
"""

Bot-side ROS entrypoint for onboard local planner + MPC execution.

This node:
- receives map/schedule updates from the laptop over TCP
- reads the robot pose from local ROS topics
- receives the latest predicted states of the other robots over UDP
- runs the shared `MpcAgent`
- publishes control commands locally on the Duckiebot
- streams telemetry back to the laptop over UDP

This is a ROS/runtime wrapper around the MPC logic found in mpc_agent.py.
"""

from __future__ import annotations

import os
import socket
from typing import Optional

import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64MultiArray

try:
    from .PI_control_motors import PI
    from ._repo_paths import TRAJPLAN_CONFIG
    from .messages import (
        CoordinatorModePacket,
        MapPacket,
        NeighborStatesPacket,
        SchedulePacket,
        StatusPacket,
        TelemetryPacket,
        packet_from_json,
        packet_to_wire,
    )
    from .mpc_agent import MpcAgent
except ImportError:
    from PI_control_motors import PI
    from _repo_paths import TRAJPLAN_CONFIG
    from messages import (
        CoordinatorModePacket,
        MapPacket,
        NeighborStatesPacket,
        SchedulePacket,
        StatusPacket,
        TelemetryPacket,
        packet_from_json,
        packet_to_wire,
    )
    from mpc_agent import MpcAgent


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _wrap_to_pi(theta: float) -> float:
    return ((theta + np.pi) % (2.0 * np.pi)) - np.pi

# Scheduler: Bot listens to laptop 

BOT_SCHEDULE_LISTEN_IP = "0.0.0.0" # Bot/Bots Ip
BOT_SCHEDULE_LISTENER_PORT = 5007

# Telemetry: Bot sends telemetry to laptop
LAPTOP_TELEMETRY_IP = "192.168.1.10" # Tosh ip
# LAPTOP_TELEMETRY_IP = "192.168.1.10" # Kim ip
LAPTOP_TELEMETRY_PORT = 5008

# Coordinator: bot receives WAIT/WORK commands from the laptop.
COORDINATOR_BIND_IP = os.getenv("MPC_COORDINATOR_BIND_IP", "0.0.0.0")
COORDINATOR_PORT = int(os.getenv("MPC_COORDINATOR_PORT", "5010"))
COORDINATOR_TIMEOUT = float(os.getenv("MPC_COORDINATOR_TIMEOUT", "1.0"))


# # Neighbor trajectories: bot listens for neighbor-state packets from laptop
BOT_NEIGHBOUR_LISTEN_IP = "0.0.0.0" # Bot/Bots IP
BOT_NEIGHBOUR_LISTEN_PORT = 5009


class BotMpcNode(DTROS):
    """ROS wrapper that connects the shared MPC core to Duckietown topics."""

    def __init__(self, node_name: str):
        super(BotMpcNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        pose_topic_template = os.getenv("MPC_POSE_TOPIC", f"/{self.vehicle_name}/mocap_reader")
        self.pose_topic = (
            pose_topic_template.replace("{vehicle}", self.vehicle_name)
            .replace("{VEHICLE_NAME}", self.vehicle_name)
        )
        self.cmd_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"

        self.pose_timeout = float(os.getenv("MPC_POSE_TIMEOUT", "0.5"))
        self.neighbor_timeout = float(os.getenv("MPC_NEIGHBOR_TIMEOUT", "0.5"))
        self.ignore_speed_ref = _env_bool("MPC_IGNORE_SPEED_REF", False)
        self.report_cost = _env_bool("MPC_REPORT_COST", False)
        self.omega_scale = float(os.getenv("MPC_OMEGA_SCALE", "1.0"))
        
        cfg_name = os.getenv("MPC_CFG_NAME")
        #cfg_name = os.getenv("MPC_CFG_NAME", "mpc_fast.yaml")
        robot_cfg_name = os.getenv("MPC_ROBOT_CFG_NAME", "robot_spec.yaml")
        

        if cfg_name is None:
            if self.vehicle_name == "duck1":
                cfg_name = "mpc_duck1.yaml"
            elif self.vehicle_name == "duck2":
                cfg_name = "mpc_duck2.yaml"
            elif self.vehicle_name == "duck4":
                cfg_name = "mpc_duck4.yaml"
            elif self.vehicle_name == "duck6":
                cfg_name = "mpc_duck6.yaml"


        from configs import CircularRobotSpecification, MpcConfiguration

        config_mpc_path = str(TRAJPLAN_CONFIG / cfg_name)
        config_robot_path = str(TRAJPLAN_CONFIG / robot_cfg_name)
        self.config_mpc = MpcConfiguration.from_yaml(config_mpc_path)
        self.config_robot = CircularRobotSpecification.from_yaml(config_robot_path)
        self.agent = MpcAgent(
            robot_id=self.vehicle_name,
            config_mpc=self.config_mpc,
            config_robot=self.config_robot,
            monitor_cost=False,
            verbose=True,
        )

        

        #self.pi_controller = PI()

        self.pose_sub = rospy.Subscriber(self.pose_topic, Float64MultiArray, self._on_pose, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)

        self._latest_pose: Optional[np.ndarray] = None
        self._latest_pose_rx_time: Optional[float] = None
        self._latest_other_robot_states: Optional[list[float]] = None
        self._latest_neighbor_rx_time: Optional[float] = None
        self._latest_neighbor_sources: list[str] = []
        self._last_action = self.agent.stop_action()
        self._schedule_id = "schedule-default"
        self._logical_robot_id = self.vehicle_name
        self._schedule_epoch = rospy.get_time()
        self._schedule_loaded = False
        self._idle_stop_sent = False
        self._pose_stale_stop_sent = False
        self._buffer = ""
        self._conn = None
        self._server = self._create_schedule_server()
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._neighbor_sock = self._create_neighbor_socket()
        self._coordinator_sock = self._create_coordinator_socket()

        # Fail safe: motion is disabled until a fresh WORK command is received.
        self.coord_mode = "WAIT"
        self._last_coordinator_rx_time: Optional[float] = None
        self._last_coordinator_sequence = -1

        self.loginfo(f"Listening for schedule updates on {BOT_SCHEDULE_LISTEN_IP}:{BOT_SCHEDULE_LISTENER_PORT}")
        self.loginfo(f"Reading pose from {self.pose_topic} (timeout={self.pose_timeout:.2f}s)")
        self.loginfo(
            f"Listening for neighbor state updates on {BOT_NEIGHBOUR_LISTEN_IP}:{BOT_NEIGHBOUR_LISTEN_PORT} "
            f"(timeout={self.neighbor_timeout:.2f}s)"
        )
        self.loginfo(f"Sending telemetry to {LAPTOP_TELEMETRY_IP}:{LAPTOP_TELEMETRY_PORT }")
        self.loginfo(
            f"Listening for coordinator commands on {COORDINATOR_BIND_IP}:{COORDINATOR_PORT} "
            f"(timeout={COORDINATOR_TIMEOUT:.2f}s)"
        )


    def _create_schedule_server(self) -> socket.socket:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((BOT_SCHEDULE_LISTEN_IP, BOT_SCHEDULE_LISTENER_PORT))
        server.listen(1)
        server.setblocking(False)
        return server

    def _create_neighbor_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((BOT_NEIGHBOUR_LISTEN_IP, BOT_NEIGHBOUR_LISTEN_PORT))
        sock.setblocking(False)
        return sock

    def _create_coordinator_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((COORDINATOR_BIND_IP, COORDINATOR_PORT))
        sock.setblocking(False)
        return sock

    def _expected_other_state_len(self) -> int:
        return self.config_mpc.ns * (self.config_mpc.N_hor + 1) * self.config_mpc.Nother

    def _empty_other_robot_states(self) -> list[float]:
        """Return the original solver's placeholder vector for 'no nearby robots'.

        The original controller used `-10` value to indicate that an
        unused other-robot slot should be treated as 'far away / irrelevant'. 

        """

        return [-10.0] * self._expected_other_state_len()

    def _normalize_other_robot_states(self, values: list[float]) -> list[float]:
        """Pad or truncate a received vector to exactly match the solver shape."""

        expected_len = self._expected_other_state_len()
        normalized = [float(value) for value in values]
        if len(normalized) >= expected_len:
            return normalized[:expected_len]
        return normalized + ([-10.0] * (expected_len - len(normalized)))

    def _other_robot_states_for_step(self) -> list[float]:
        """Return the freshest neighbor-state vector, or a safe empty placeholder.

        Multi-robot MPC should keep running even if peer updates arrive late. When
        that happens we intentionally fall back to the 'no other robots nearby'
        sentinel rather than blocking the control loop entirely.
        """

        if self.config_mpc.Nother <= 0:
            return []

        if self._latest_other_robot_states is None or self._latest_neighbor_rx_time is None:
            return self._empty_other_robot_states()

        age = rospy.get_time() - self._latest_neighbor_rx_time
        if age > self.neighbor_timeout:
            rospy.logwarn_throttle(
                2.0,
                "Neighbor state stream stale for %s: age=%.3fs timeout=%.3fs. Falling back to empty slots.",
                self._logical_robot_id,
                age,
                self.neighbor_timeout,
            )
            return self._empty_other_robot_states()

        return self._normalize_other_robot_states(self._latest_other_robot_states)

    def _on_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            return

        pose = np.asarray(msg.data[:3], dtype=float)
        # Reject non-finite samples here as a final safety net. The mocap relay
        # and receiver should already drop bad packets, but keeping this guard in
        # the control node prevents any upstream regression from breaking the MPC
        # state with NaNs.
        if not np.all(np.isfinite(pose)):
            rospy.logwarn_throttle(2.0, "Discarding non-finite pose sample on %s", self.pose_topic)
            return

        pose[2] = _wrap_to_pi(float(pose[2]))
        self._latest_pose = pose
        self._latest_pose_rx_time = rospy.get_time()
        self._pose_stale_stop_sent = False

    def _accept_connection(self) -> None:
        if self._conn is not None:
            return
        try:
            self._conn, addr = self._server.accept()
            self._conn.setblocking(False)
            self._buffer = ""
            self.loginfo(f"Schedule client connected: {addr[0]}:{addr[1]}")
        except BlockingIOError:
            return
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "Schedule accept failed: %s", exc)

    def _poll_schedule_socket(self) -> None:
        self._accept_connection()
        if self._conn is None:
            return

        try:
            chunk = self._conn.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            self._close_connection()
            return

        if not chunk:
            self._close_connection()
            return

        self._buffer += chunk.decode("utf-8")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            self._handle_packet(line)

    def _handle_packet(self, raw_packet: str) -> None:
        try:
            packet = packet_from_json(raw_packet)
        except Exception as exc:
            rospy.logwarn("Invalid schedule/map packet: %s", exc)
            return

        if isinstance(packet, MapPacket):
            self.agent.load_map(
                [tuple(point) for point in packet.boundary_coords],
                [[tuple(point) for point in polygon] for polygon in packet.static_obstacles],
            )
            self.loginfo(f"Loaded map {packet.map_id}")
            self._send_status("info", f"Loaded map {packet.map_id}")
            return

        if isinstance(packet, SchedulePacket):
            self._schedule_id = packet.schedule_id
            self._logical_robot_id = packet.robot_id
            self._schedule_epoch = rospy.get_time() + max(0.0, packet.effective_from)
            self._idle_stop_sent = False
            self.agent.load_schedule(
                start_state=np.asarray(packet.start_state, dtype=float),
                path_coords=[tuple(point) for point in packet.path_coords],
                path_times=packet.path_times,
            )
            self._schedule_loaded = True
            self.loginfo(f"Loaded schedule {packet.schedule_id} for {packet.robot_id}")
            self._send_status("info", f"Loaded schedule {packet.schedule_id}")
            return

    def _handle_neighbor_packet(self, raw_packet: bytes, sender: tuple[str, int]) -> None:
        # Decodes raw UDP data as utf-8 and parses the JSON, checks
        # that is of type NeighborStatesPacket and stored the data.
        try:
            packet = packet_from_json(raw_packet.decode("utf-8"))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Invalid neighbor packet from %s:%s: %s", sender[0], sender[1], exc)
            return

        if not isinstance(packet, NeighborStatesPacket):
            return

        # The laptop relay addresses bots by logical schedule id (A1, A2, ...).
        # Before the first schedule arrives, the runtime still uses VEHICLE_NAME as
        # its local identity, so we accept either name during the transition.
        if packet.robot_id not in {self._logical_robot_id, self.vehicle_name}:
            return

        if self._schedule_loaded and packet.schedule_id not in {self._schedule_id, "", "schedule-default"}:
            return

        self._latest_other_robot_states = self._normalize_other_robot_states(packet.other_robot_states)
        self._latest_neighbor_rx_time = rospy.get_time()
        self._latest_neighbor_sources = list(packet.source_robot_ids)
        rospy.loginfo_throttle(
            2.0,
            "Neighbor trajectories updated for %s from %s",
            self._logical_robot_id,
            ", ".join(self._latest_neighbor_sources) if self._latest_neighbor_sources else "no peers",
        )

    def _poll_neighbor_socket(self, max_packets: int = 32) -> None:
        # Reads raw UDP data from socket
        for _ in range(max_packets):
            try:
                packet, sender = self._neighbor_sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                rospy.logwarn_throttle(2.0, "Neighbor UDP receive failed: %s", exc)
                return
            self._handle_neighbor_packet(packet, sender)

    def _handle_coordinator_packet(self, raw_packet: bytes, sender: tuple[str, int]) -> None:
        try:
            packet = packet_from_json(raw_packet.decode("utf-8").strip())
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Invalid coordinator packet from %s:%s: %s",
                sender[0],
                sender[1],
                exc,
            )
            return

        if not isinstance(packet, CoordinatorModePacket):
            return

        if packet.robot_id not in {self._logical_robot_id, self.vehicle_name}:
            return

        mode = packet.mode.strip().upper()
        if mode not in {"WAIT", "WORK"}:
            rospy.logwarn_throttle(2.0, "Unsupported coordinator mode: %s", packet.mode)
            return

        if packet.sequence < self._last_coordinator_sequence:
            return

        self._last_coordinator_sequence = packet.sequence
        self._last_coordinator_rx_time = rospy.get_time()
        self.coord_mode = mode
        rospy.loginfo_throttle(
            1.0,
            "Coordinator mode for %s: %s",
            self._logical_robot_id,
            self.coord_mode,
        )

    def _poll_coordinator_socket(self, max_packets: int = 32) -> None:
        for _ in range(max_packets):
            try:
                packet, sender = self._coordinator_sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as exc:
                rospy.logwarn_throttle(2.0, "Coordinator UDP receive failed: %s", exc)
                return
            self._handle_coordinator_packet(packet, sender)

    def _clip_action(self, action: np.ndarray) -> np.ndarray:
        clipped = np.asarray(action, dtype=float).copy()
        clipped[0] = np.clip(clipped[0], self.config_robot.lin_vel_min, self.config_robot.lin_vel_max)
        clipped[1] = np.clip(clipped[1], -self.config_robot.ang_vel_max, self.config_robot.ang_vel_max)
        return clipped

    def _publish_action(self, action: np.ndarray) -> None:
        published_action = self._clip_action(np.asarray(
            [float(action[0]), float(action[1]) * self.omega_scale],
            dtype=float,
        ))

        # if self.pi_controller.is_ready():
        #     corrected_v, corrected_omega = self.pi_controller.pi_controller(
        #         float(published_action[0]),
        #         float(published_action[1]),
        #         self.config_mpc.ts,
        #     )
        #     published_action = self._clip_action(np.asarray([corrected_v, corrected_omega], dtype=float))
        # else:
        #     rospy.loginfo_throttle(2.0, "Wheel PI waiting for valid encoder updates; publishing raw MPC command.")

        coordinator_stale = (
            self._last_coordinator_rx_time is None
            or rospy.get_time() - self._last_coordinator_rx_time > COORDINATOR_TIMEOUT
        )
        if self.coord_mode == "WAIT" or coordinator_stale:
            published_action = np.zeros(2, dtype=float)

        msg = Twist2DStamped(v=float(published_action[0]), omega=float(published_action[1]))

        self.cmd_pub.publish(msg)
        self._last_action = published_action


    def _send_status(self, level: str, message: str) -> None:
        packet = StatusPacket(
            robot_id=self._logical_robot_id,
            level=level,
            message=message,
            t=rospy.get_time(),
            schedule_id=self._schedule_id,
        )
        try:
            self._telemetry_sock.sendto(packet_to_wire(packet), (LAPTOP_TELEMETRY_IP, LAPTOP_TELEMETRY_PORT ))
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "Status UDP send failed: %s", exc)

    def _send_telemetry(self, step_result) -> None:
        target_node = step_result["current_target_node"]
        packet = TelemetryPacket(
            robot_id=self._logical_robot_id,
            schedule_id=self._schedule_id,
            t=max(0.0, rospy.get_time() - self._schedule_epoch),
            pose=self.agent.state.tolist(),
            action=self._last_action.tolist(),
            pred_states=np.asarray(step_result["pred_states"], dtype=float).tolist(),
            current_refs=np.asarray(step_result["current_refs"], dtype=float).tolist(),
            current_target_node=None if target_node is None else [float(target_node[0]), float(target_node[1])],
            cost=float(step_result["debug_info"].get("cost", 0.0)),
            solver_time=step_result["solver_time"],
            status="idle" if step_result["controller_idle"] else "running",
        )
        try:
            self._telemetry_sock.sendto(packet_to_wire(packet), (LAPTOP_TELEMETRY_IP, LAPTOP_TELEMETRY_PORT ))
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "Telemetry UDP send failed: %s", exc)

    def _close_connection(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            finally:
                self._conn = None
                self._buffer = ""

    def run(self) -> None:
        rate = rospy.Rate(max(1.0, 1.0 / self.config_mpc.ts))
        while not rospy.is_shutdown():
            self._poll_schedule_socket()
            self._poll_neighbor_socket()
            self._poll_coordinator_socket()

            if not self._schedule_loaded or self._latest_pose is None:
                rate.sleep()
                continue

            if rospy.get_time() < self._schedule_epoch:
                rate.sleep()
                continue

            pose_age = None if self._latest_pose_rx_time is None else rospy.get_time() - self._latest_pose_rx_time
            if pose_age is None or pose_age > self.pose_timeout:
                rospy.logwarn_throttle(
                    2.0,
                    "Pose stream stale on %s: age=%s timeout=%.3fs",
                    self.pose_topic,
                    "NA" if pose_age is None else f"{pose_age:.3f}s",
                    self.pose_timeout,
                )
                if not self._pose_stale_stop_sent:
                    self._publish_action(self.agent.stop_action())
                    self._pose_stale_stop_sent = True
                rate.sleep()
                continue

            other_robot_states = self._other_robot_states_for_step()
            self.agent.set_state(self._latest_pose)
            step_result = self.agent.step(
                t_now=max(0.0, rospy.get_time() - self._schedule_epoch),
                other_robot_states=other_robot_states,
                ignore_speed_ref=self.ignore_speed_ref,
                report_cost=self.report_cost,
            )

            if step_result["controller_idle"]:
                if not self._idle_stop_sent:
                    self._publish_action(self.agent.stop_action())
                    self._idle_stop_sent = True
            else:
                self._publish_action(step_result["action"])
                self._idle_stop_sent = False

            pose = self._latest_pose
            cmd = self._last_action
            rospy.loginfo_throttle(
                self.config_mpc.ts,
                f"[mpc_runtime data] pose=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}) "
                f"cmd=({cmd[0]:.3f}, {cmd[1]:.3f}) "
                f"peer_sources={self._latest_neighbor_sources or []} idle={step_result['controller_idle']}"
            )

            self._send_telemetry(step_result)
            rate.sleep()

    def on_shutdown(self) -> None:
        self._publish_action(self.agent.stop_action())
        self._close_connection()
        if self._server is not None:
            self._server.close()
        self._neighbor_sock.close()
        self._coordinator_sock.close()
        self._telemetry_sock.close()


if __name__ == "__main__":
    node = BotMpcNode(node_name="bot_mpc_node")
    node.run()
