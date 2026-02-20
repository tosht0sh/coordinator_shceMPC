#!/usr/bin/env python3

import json
import os
import socket
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float64MultiArray


class HandshakeSubscriberNode(DTROS):
    def __init__(self, node_name):
        super(HandshakeSubscriberNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC,
        )

        vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        self._listen_ip = os.getenv("ROBOT_TCP_IP", "0.0.0.0")
        self._listen_port = int(os.getenv("ROBOT_TCP_PORT", "5006"))
        self._publisher = rospy.Publisher(
            f"/{vehicle}/laptop_pose_cmd", Float64MultiArray, queue_size=10
        )

    def _parse_payload(self, raw):
        # Supports either JSON ({"x":..,"y":..,"theta":..}) or "x,y,theta".
        try:
            data = json.loads(raw)
            return float(data["x"]), float(data["y"]), float(data["theta"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            parts = [p.strip() for p in raw.split(",")]
            if len(parts) != 3:
                raise ValueError("Payload must be JSON or x,y,theta")
            return float(parts[0]), float(parts[1]), float(parts[2])

    def run(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_ip, self._listen_port))
        server.listen(1)
        server.settimeout(1.0)
        rospy.loginfo("Listening for TCP pose on %s:%d", self._listen_ip, self._listen_port)

        while not rospy.is_shutdown():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError as e:
                rospy.logwarn_throttle(2.0, "TCP accept failed: %s", e)
                continue

            rospy.loginfo("TCP client connected: %s:%s", addr[0], addr[1])
            with conn:
                conn.settimeout(1.0)
                buffer = ""
                while not rospy.is_shutdown():
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break

                    if not chunk:
                        break

                    buffer += chunk.decode("utf-8")
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            x, y, theta = self._parse_payload(line)
                        except ValueError as e:
                            rospy.logwarn_throttle(2.0, "Invalid payload: %s", e)
                            continue

                        self._publisher.publish(Float64MultiArray(data=[x, y, theta]))
                        rospy.loginfo_throttle(
                            1.0,
                            "Received command pose: %.3f, %.3f, %.3f",
                            x,
                            y,
                            theta,
                        )

if __name__ == '__main__':
    node = HandshakeSubscriberNode(node_name='handshake_subscriber_node')
    node.run()
