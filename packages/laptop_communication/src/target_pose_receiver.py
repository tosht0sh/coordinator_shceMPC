#!/usr/bin/env python3

import json
import os
import socket
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float64MultiArray
from duckietown_msgs.msg import WheelsCmdStamped

THROTTLE_LEFT = 0.1
THROTTLE_RIGHT = 0.1
CONTROL_HZ = 20
TARGET_EPS = 1e-3


class TargetPoseTcpReceiverNode(DTROS):
    """
    Purpose:
      Receive target pose commands over TCP and drive the robot forward until
      the current x-position reaches the target x-position.

    ROS topics used:
    - publishers:
      - /<vehicle>/laptop_pose_cmd:
        Republishes the latest received target pose [x, y, theta].
      - /<vehicle>/wheels_driver_node/wheels_cmd:
        Wheel throttle command used for move/stop control.
    - subscribers:
      - /<vehicle>/pose_reader:
        Current robot pose estimated from wheel encoders.
    """
    def __init__(self, node_name):
        super(TargetPoseTcpReceiverNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC,
        )

        vehicle = os.environ.get("VEHICLE_NAME", "duckiebot")
        self._listen_ip = os.getenv("ROBOT_TCP_IP", "192.168.1.197")
        self._listen_port = int(os.getenv("ROBOT_TCP_PORT", "5006"))
        self._target_pose = None
        self._current_x = None
        self._moving = False

        self.pose_sub = rospy.Subscriber(
            f"/{vehicle}/pose_reader", Float64MultiArray, self.callback_pose_reader, queue_size=10
        )

        # TODO: maybe this publisher is not sensible
        self._publisher = rospy.Publisher(
            f"/{vehicle}/laptop_pose_cmd", Float64MultiArray, queue_size=10
        )

        wheels_topic = f"/{vehicle}/wheels_driver_node/wheels_cmd"
        self.wheel_publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def callback_pose_reader(self, msg):
        if len(msg.data) < 1:
            return
        self._current_x = float(msg.data[0])

    def _publish_motion(self, left, right):
        self.wheel_publisher.publish(WheelsCmdStamped(vel_left=left, vel_right=right))

    def _update_control(self):
        if self._target_pose is None or self._current_x is None:
            if self._moving:
                self._publish_motion(0.0, 0.0)
                self._moving = False
            return

        target_x = self._target_pose[0]
        if self._current_x < (target_x - TARGET_EPS):
            self._publish_motion(THROTTLE_LEFT, THROTTLE_RIGHT)
            self._moving = True
        else:
            if self._moving:
                self._publish_motion(0.0, 0.0)
                self._moving = False

    def _parse_payload(self, raw):
        """
        Parse target pose from either JSON ({"x":..,"y":..,"theta":..}) or
        a CSV string "x,y,theta".
        """
        
        try:
            data = json.loads(raw)
            return float(data["x"]), float(data["y"]), float(data["theta"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            parts = [p.strip() for p in raw.split(",")]
            if len(parts) != 3:
                raise ValueError("Payload must be JSON or x,y,theta")
            return float(parts[0]), float(parts[1]), float(parts[2])

    def run(self):
        # TCP connection to receive target coordinate form laptop.
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_ip, self._listen_port))
        server.listen(1)
        server.setblocking(False)
        rospy.loginfo("Listening for TCP pose on %s:%d", self._listen_ip, self._listen_port)

        
        conn = None         # to check connection on or not
        buffer = ""         # input stream buffer
        rate = rospy.Rate(CONTROL_HZ)

        while not rospy.is_shutdown():
            # setup connection
            if conn is None:
                try:
                    conn, addr = server.accept()
                    conn.setblocking(False)
                    buffer = ""
                    rospy.loginfo("TCP client connected: %s:%s", addr[0], addr[1])
                except BlockingIOError:
                    pass
                except OSError as e:
                    rospy.logwarn_throttle(2.0, "TCP accept failed: %s", e)

            if conn is not None:
                try:
                    chunk = conn.recv(1024)
                    if not chunk:
                        conn.close()
                        conn = None
                    else:
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
                            self._target_pose = (x, y, theta)
                            self._publisher.publish(Float64MultiArray(data=[x, y, theta]))
                            rospy.loginfo("New target pose: %.3f, %.3f, %.3f", x, y, theta)
                except BlockingIOError:
                    pass
                except OSError:
                    conn = None

            self._update_control()
            rate.sleep()

    def on_shutdown(self):
        self._publish_motion(0.0, 0.0)


if __name__ == '__main__':
    node = TargetPoseTcpReceiverNode(node_name='target_pose_receiver_node')
    node.run()
