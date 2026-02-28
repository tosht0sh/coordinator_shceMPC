#!/usr/bin/env python3

import os
import json
import socket
import rospy
from std_msgs.msg import Float64MultiArray


class LaptopPoseReader:
    """
    Purpose:
      Receive robot pose packets over UDP on the laptop and republish them as ROS topics.

    ROS topics used:
    - publishers:
      - /laptop_pose_reader:
        Shared topic with all received robot poses (useful for quick monitoring).
      - /<vehicle>/laptop_pose_reader:
        Per-vehicle namespaced pose topic for multi-robot use.
    - subscribers:
      - none
    """
    def __init__(self):
        rospy.init_node("pose_reader", anonymous=True)
        self._udp_ip = rospy.get_param("~udp_ip", "192.168.1.197")
        self._udp_port = int(rospy.get_param("~udp_port", 5005))
        self._default_topic = rospy.get_param("~pose_topic", "/laptop_pose_reader")
        self._pub_default = rospy.Publisher(self._default_topic, Float64MultiArray, queue_size=10)
        self._per_bot_publishers = {}

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._udp_ip, self._udp_port))
        self._sock.settimeout(0.5)
        rospy.loginfo("Listening for UDP pose packets on %s:%d", self._udp_ip, self._udp_port)

    def _get_bot_publisher(self, vehicle):
        if vehicle not in self._per_bot_publishers:
            topic = f"/{vehicle}/laptop_pose_reader"
            self._per_bot_publishers[vehicle] = rospy.Publisher(topic, Float64MultiArray, queue_size=10)
        return self._per_bot_publishers[vehicle]

    def run(self):
        while not rospy.is_shutdown():
            try:
                packet, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError as e:
                rospy.logwarn_throttle(2.0, "UDP receive failed: %s", e)
                continue

            try:
                data = json.loads(packet.decode("utf-8"))
                vehicle = data["vehicle"]
                x = float(data["x"])
                y = float(data["y"])
                theta = float(data["theta"])
            except (KeyError, ValueError, TypeError, json.JSONDecodeError):
                rospy.logwarn_throttle(2.0, "Received malformed UDP payload")
                continue

            msg = Float64MultiArray(data=[x, y, theta])
            self._pub_default.publish(msg)
            self._get_bot_publisher(vehicle).publish(msg)
            rospy.loginfo_throttle(1.0, "[%s] pose: %.3f, %.3f, %.3f", vehicle, x, y, theta)

if __name__ == '__main__':
    node = LaptopPoseReader()
    node.run()
