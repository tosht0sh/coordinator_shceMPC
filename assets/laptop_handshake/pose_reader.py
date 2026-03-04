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
        self._clock_hz = float(rospy.get_param("~clock_hz", 20.0))
        self._clock_enabled = bool(rospy.get_param("~clock_enabled", True))
        self._clock_start_time = rospy.get_time()
        self._pub_default = rospy.Publisher(self._default_topic, Float64MultiArray, queue_size=10)
        self._per_bot_publishers = {}
        self._bot_udp_peers = {}

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._udp_ip, self._udp_port))
        self._sock.settimeout(0.5)
        rospy.loginfo("Listening for UDP pose packets on %s:%d", self._udp_ip, self._udp_port)

    def _get_bot_publisher(self, vehicle):
        if vehicle not in self._per_bot_publishers:
            topic = f"/{vehicle}/laptop_pose_reader"
            self._per_bot_publishers[vehicle] = rospy.Publisher(topic, Float64MultiArray, queue_size=10)
        return self._per_bot_publishers[vehicle]
    
    def _send_clock_sync(self):
        if not self._clock_enabled:
            return
        
        now = rospy.get_time()
        elapsed = now - self._clock_start_time
        for vehicle, peer in self._bot_udp_peers.items():
            payload = {
            "type": "clock",
            "vehicle": vehicle,
            "laptop_time": elapsed
        }
            try:
                    self._sock.sendto(json.dumps(payload).encode("utf-8"), peer)
            except OSError as e:
                rospy.logwarn_throttle(2.0, "UDP clock send failed for %s: %s", vehicle, e)
        rospy.loginfo_throttle(10.0, "Sent clock packets to %d bot(s)", len(self._bot_udp_peers))

    def run(self):
        next_clock_send = rospy.get_time()
        clock_period = (1.0 / self._clock_hz) 
        
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self._clock_enabled and now >= next_clock_send:
                self._send_clock_sync()
                next_clock_send = now + clock_period
                
            try:
                packet, sender = self._sock.recvfrom(4096)
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

            self._bot_udp_peers[vehicle] = sender

            msg = Float64MultiArray(data=[x, y, theta])
            self._pub_default.publish(msg)
            self._get_bot_publisher(vehicle).publish(msg)
            rospy.loginfo("[%s] pose: %.3f, %.3f, %.3f", vehicle, x, y, theta)

if __name__ == '__main__':
    node = LaptopPoseReader()
    node.run()
