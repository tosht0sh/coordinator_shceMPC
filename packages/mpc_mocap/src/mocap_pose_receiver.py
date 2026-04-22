#!/usr/bin/env python3

import json
import os
import socket

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float64MultiArray


class MocapPoseReceiverNode(DTROS):
    """Receive planar mocap pose over UDP and publish it on /<vehicle>/mocap_reader."""

    def __init__(self, node_name):
        super(MocapPoseReceiverNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ["VEHICLE_NAME"]
        self._bind_ip = os.getenv("MOCAP_BIND_IP", "192.168.1.12") # Bot IP
        self._port = int(os.getenv("MOCAP_PORT", "22223"))
        self._stale_timeout = float(os.getenv("MOCAP_STALE_TIMEOUT", "0.5"))
        self._source_ip = os.getenv("MOCAP_SOURCE_IP", "192.168.1.9").strip() or None # Laptop IP

        self._topic = f"/{self._vehicle_name}/mocap_reader"
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._bind_ip, self._port))
        self._sock.setblocking(False)

        self._pose_pub = rospy.Publisher(self._topic, Float64MultiArray, queue_size=10)
        self._latest_rx_time = None
        self._last_warn_time = 0.0

        self.loginfo(f"Listening for mocap UDP on {self._bind_ip}:{self._port}")
        self.loginfo(f"Publishing mocap pose on {self._topic}")
        if self._source_ip is not None:
            self.loginfo(f"Accepting mocap packets only from {self._source_ip}")

    def _handle_packet(self, packet, sender):
        sender_ip = sender[0]
        if self._source_ip is not None and sender_ip != self._source_ip:
            return

        try:
            data = json.loads(packet.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            rospy.logwarn_throttle(2.0, "Invalid mocap UDP packet received")
            return

        if data.get("vehicle") != self._vehicle_name:
            return

        try:
            x = float(data["x"])
            y = float(data["y"])
            theta = float(data["theta"])
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(2.0, "Mocap packet missing x/y/theta fields")
            return

        self._latest_rx_time = rospy.get_time()
        self._pose_pub.publish(Float64MultiArray(data=[x, y, theta]))

    def _poll_packets(self, max_packets=32):
        for _ in range(max_packets):
            try:
                packet, sender = self._sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError as exc:
                rospy.logwarn_throttle(2.0, "Mocap UDP receive failed: %s", exc)
                return

            self._handle_packet(packet, sender)

    def _warn_if_stale(self):
        if self._latest_rx_time is None:
            return

        age = rospy.get_time() - self._latest_rx_time
        if age > self._stale_timeout:
            rospy.logwarn_throttle(
                2.0,
                "Mocap pose stream stale for %s: age=%.3fs timeout=%.3fs",
                self._vehicle_name,
                age,
                self._stale_timeout,
            )

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self._poll_packets()
            self._warn_if_stale()
            rate.sleep()

    def on_shutdown(self):
        self._sock.close()


if __name__ == "__main__":
    node = MocapPoseReceiverNode(node_name="mocap_pose_receiver")
    node.run()
