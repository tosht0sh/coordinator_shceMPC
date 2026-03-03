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
        self._laptop_ip = os.getenv("LAPTOP_IP", "192.168.1.197")
        self._laptop_port = int(os.getenv("LAPTOP_UDP_PORT", "5005"))
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
            rospy.loginfo_throttle(2.0, "Forwarding pose via UDP to laptop")
        except OSError as e:
            rospy.logwarn_throttle(2.0, "UDP send failed: %s", e)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = HandshakePublisherNode(node_name='handshake_publisher')
    node.run()
