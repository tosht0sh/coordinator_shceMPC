#!/usr/bin/env python3
"""Bot-side ROS entrypoint for onboard local planner + MPC execution.

This node:
- receives map/schedule updates from the laptop over TCP
- reads the robot pose from local ROS topics
- runs the shared `MpcAgent`
- publishes control commands locally on the Duckiebot
- streams telemetry back to the laptop over UDP

This is a ROS/runtime wrapper around the MPC logic found in mpc_agent.py

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
    from ._repo_paths import TRAJPLAN_CONFIG
    from .messages import MapPacket, SchedulePacket, StatusPacket, TelemetryPacket, packet_from_json, packet_to_wire
    from .mpc_agent import MpcAgent
except ImportError:
    from _repo_paths import TRAJPLAN_CONFIG
    from messages import MapPacket, SchedulePacket, StatusPacket, TelemetryPacket, packet_from_json, packet_to_wire
    from mpc_agent import MpcAgent


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


class BotMpcNode(DTROS):
    """ROS wrapper that connects the shared MPC core to Duckietown topics."""

    def __init__(self, node_name: str):
        super(BotMpcNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.vehicle_name = os.environ["VEHICLE_NAME"]
        self.pose_topic = f"/{self.vehicle_name}/pose_reader"
        self.cmd_topic = f"/{self.vehicle_name}/car_cmd_switch_node/cmd"

        self.schedule_bind_ip = os.getenv("MPC_SCHEDULE_BIND_IP", "0.0.0.0")
        self.schedule_port = int(os.getenv("MPC_SCHEDULE_PORT", "5007"))
        self.telemetry_ip = os.getenv("MPC_TELEMETRY_IP", os.getenv("LAPTOP_IP", "192.168.1.197"))
        self.telemetry_port = int(os.getenv("MPC_TELEMETRY_PORT", "5008"))
        self.ignore_speed_ref = _env_bool("MPC_IGNORE_SPEED_REF", False)
        self.report_cost = _env_bool("MPC_REPORT_COST", False)
        self.omega_scale = float(os.getenv("MPC_OMEGA_SCALE", "1.5"))

        cfg_name = os.getenv("MPC_CFG_NAME", "mpc_fast.yaml")
        robot_cfg_name = os.getenv("MPC_ROBOT_CFG_NAME", "robot_spec.yaml")

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

        self.pose_sub = rospy.Subscriber(self.pose_topic, Float64MultiArray, self._on_pose, queue_size=10)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)

        self._latest_pose: Optional[np.ndarray] = None
        self._last_action = self.agent.stop_action()
        self._schedule_id = "schedule-default"
        self._logical_robot_id = self.vehicle_name
        self._schedule_epoch = rospy.get_time()
        self._schedule_loaded = False
        self._buffer = ""
        self._conn = None
        self._server = self._create_schedule_server()
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.loginfo(f"Listening for schedule updates on {self.schedule_bind_ip}:{self.schedule_port}")
        self.loginfo(f"Sending telemetry to {self.telemetry_ip}:{self.telemetry_port}")

    def _create_schedule_server(self) -> socket.socket:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.schedule_bind_ip, self.schedule_port))
        server.listen(1)
        server.setblocking(False)
        return server

    def _on_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 3:
            return
        self._latest_pose = np.asarray(msg.data[:3], dtype=float)

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
            self._send_status("info", f"Loaded map {packet.map_id}")
            return

        if isinstance(packet, SchedulePacket):
            self._schedule_id = packet.schedule_id
            self._logical_robot_id = packet.robot_id
            self._schedule_epoch = rospy.get_time() + max(0.0, packet.effective_from)
            self.agent.load_schedule(
                start_state=np.asarray(packet.start_state, dtype=float),
                path_coords=[tuple(point) for point in packet.path_coords],
                path_times=packet.path_times,
            )
            self._schedule_loaded = True
            self._send_status("info", f"Loaded schedule {packet.schedule_id}")
            return

    def _publish_action(self, action: np.ndarray) -> None:
        msg = Twist2DStamped(v=float(action[0]), omega=float(action[1]) * self.omega_scale)
        self.cmd_pub.publish(msg)
        self._last_action = np.asarray(action, dtype=float)

    def _reported_robot_id(self) -> str:
        return self._logical_robot_id

    def _send_status(self, level: str, message: str) -> None:
        packet = StatusPacket(
            robot_id=self._reported_robot_id(),
            level=level,
            message=message,
            t=rospy.get_time(),
            schedule_id=self._schedule_id,
        )
        try:
            self._telemetry_sock.sendto(packet_to_wire(packet), (self.telemetry_ip, self.telemetry_port))
        except OSError as exc:
            rospy.logwarn_throttle(2.0, "Status UDP send failed: %s", exc)

    def _send_telemetry(self, step_result) -> None:
        target_node = step_result["current_target_node"]
        packet = TelemetryPacket(
            robot_id=self._reported_robot_id(),
            schedule_id=self._schedule_id,
            t=max(0.0, rospy.get_time() - self._schedule_epoch),
            pose=self.agent.state.tolist(),
            action=np.asarray(step_result["action"], dtype=float).tolist(),
            pred_states=np.asarray(step_result["pred_states"], dtype=float).tolist(),
            current_refs=np.asarray(step_result["current_refs"], dtype=float).tolist(),
            current_target_node=None if target_node is None else [float(target_node[0]), float(target_node[1])],
            cost=float(step_result["debug_info"].get("cost", 0.0)),
            solver_time=step_result["solver_time"],
            status="idle" if step_result["controller_idle"] else "running",
        )
        try:
            self._telemetry_sock.sendto(packet_to_wire(packet), (self.telemetry_ip, self.telemetry_port))
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

            if not self._schedule_loaded or self._latest_pose is None:
                rate.sleep()
                continue

            if rospy.get_time() < self._schedule_epoch:
                rate.sleep()
                continue

            self.agent.set_state(self._latest_pose)
            step_result = self.agent.step(
                t_now=max(0.0, rospy.get_time() - self._schedule_epoch),
                other_robot_states=None,
                ignore_speed_ref=self.ignore_speed_ref,
                report_cost=self.report_cost,
            )
            self._publish_action(step_result["action"])
            self._send_telemetry(step_result)
            rate.sleep()

    def on_shutdown(self) -> None:
        self._publish_action(self.agent.stop_action())
        self._close_connection()
        self._server.close()
        self._telemetry_sock.close()


if __name__ == "__main__":
    node = BotMpcNode(node_name="bot_mpc_node")
    node.run()
