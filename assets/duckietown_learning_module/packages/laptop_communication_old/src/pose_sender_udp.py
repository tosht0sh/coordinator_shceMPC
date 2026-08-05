#!/usr/bin/env python3

import os
import json
import socket
import rospy
from std_msgs.msg import Float64MultiArray
from duckietown.dtros import DTROS, NodeType


class HandshakePublisherNode(DTROS):
    """
    Purpose:
      Read robot pose from local ROS and send it to the laptop via UDP.

    ROS topics used:
    - publishers:
      - none
    - subscribers:
      - /<vehicle>/pose_reader:
        Pose estimated from wheel encoders on the robot.
    """
    def __init__(self, node_name):
        super(HandshakePublisherNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC,
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._laptop_ip = os.getenv("LAPTOP_IP", "192.168.1.192")
        self._laptop_port = int(os.getenv("LAPTOP_UDP_PORT", "5005"))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.0)

        # Receives pose from wheel encoder reader node running on the bot.
        self.sub_enc_pose = rospy.Subscriber(
            f"/{self._vehicle_name}/pose_reader", Float64MultiArray, self.callback_pose, queue_size=10
        )
        self.loginfo("Sending UDP pose packets")

    def callback_pose(self, msg):
        if len(msg.data) < 3:
            return

        payload = {
            "vehicle": self._vehicle_name,
            "x": float(msg.data[0]),
            "y": float(msg.data[1]),
            "theta": float(msg.data[2]),
            "stamp": rospy.get_time(),
        }

        try:
            packet = json.dumps(payload).encode("utf-8")
            self._sock.sendto(packet, (self._laptop_ip, self._laptop_port))
            # rospy.loginfo_throttle(2.0, "Forwarding pose via UDP to laptop")
        except OSError as e:
            rospy.logwarn_throttle(2.0, "UDP send failed: %s", e)

        self.clock_receiver()

    def clock_receiver(self):
        while not rospy.is_shutdown():
            try:
                packet, sender = self._sock.recvfrom(4096)
            except socket.timeout:
                break
            except BlockingIOError:
                break
            except OSError as e:
                rospy.logwarn_throttle(2.0, "UDP receive failed: %s", e)
                break

            try:
                data = json.loads(packet.decode("utf-8"))
            except (json.JSONDecodeError, UnicodeDecodeError):
                rospy.logwarn_throttle(2.0, "Received incorrect UDP packet")
                continue

            if data.get("type") != "clock":
                continue

            try:
                laptop_time = float(data["laptop_time"])
            except (KeyError, TypeError, ValueError):
                rospy.logwarn_throttle(2.0, "Clock packet missing/invalid laptop_time")
                continue

            if data.get("vehicle") not in (None, self._vehicle_name):
                continue

            self._last_laptop_time = laptop_time
            self._last_clock_rx_time = rospy.get_time()
            rospy.loginfo("Clock sync packet from %s: %.6f", sender, laptop_time)


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = HandshakePublisherNode(node_name='handshake_publisher')
    node.run()
