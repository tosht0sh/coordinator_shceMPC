#!/usr/bin/env python3

import json
import os
import socket
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float64MultiArray
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

# THROTTLE_LEFT = 0.1
# THROTTLE_RIGHT = 0.1
# CONTROL_HZ = 20
# TARGET_EPS = 1e-3

V_MAX = 1.5
W_MAX = 0.5


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

        self._rate = 10 # Hz
        vehicle_name = os.environ.get("VEHICLE_NAME", "duckiebot")
        self._axis_length = rospy.get_param("axis_length", 0.09716)

        # TODO: maybe this publisher is not sensible
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=10)

        # kinematic nodes subscriber
        twist_topic  = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self._twist_publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self._twist_subscriber = rospy.Subscriber("/duckiebot/kinematics_node/velocity", Twist2DStamped, cb)

        # communication nodes
        self._listen_ip = os.getenv("ROBOT_TCP_IP", "192.168.1.197") # for CASELAB wifi
        # self._listen_ip = "10.42.0.129" # for laptop hotspot
        self._listen_port = int(os.getenv("ROBOT_TCP_PORT", "5006"))
        
        # velocities to be published
        self.target_v = 0.0
        self.target_w = 0.0

        self.first_command = True
        self.last_msg_time = 0.0
        

    def _parse_payload(self, raw):
        """
        Parse target pose from either JSON ({"v":..,"w":..,}) 
        """
        
        # try:
        data = json.loads(raw)
        return float(data["v"]), float(data["w"])
        # except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        #     parts = [p.strip() for p in raw.split(",")]
        #     if len(parts) != 3:
        #         raise ValueError("Payload must be JSON or x,y,theta")
        #     return float(parts[0]), float(parts[1]), float(parts[2])

    def _wheel_control(self):
        # vel_left = self.target_v - ((self._axis_length / 2) * self.target_w)
        # vel_right = self.target_v + ((self._axis_length / 2) * self.target_w)

        # vel_left_cmd = vel_left / V_MAX
        # vel_right_cmd = vel_right / V_MAX

        message = Twist2DStamped(
            v=self.target_v,
            omega=self.target_w * 1.5
            )
        self._twist_publisher.publish(message)



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
        rate = rospy.Rate(self._rate)

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
                        if self.first_command:
                            self.first_command = False
                        else:
                            while "\n" in buffer:
                                line, buffer = buffer.split("\n", 1)
                                line = line.strip()
                                
                                if not line:
                                    continue

                                try:
                                    self.target_v, self.target_w = self._parse_payload(line)
                                    self.last_msg_time = rospy.get_time()
                                    rospy.loginfo(f"Linear velocity: {self.target_v}, Angular Velocity: {self.target_w}")
                                except (json.JSONDecodeError, KeyError, TypeError, ValueError) as e:
                                    rospy.logwarn(f"Could not parse data.")

                except BlockingIOError:
                    pass
                except OSError:
                    conn = None

            now = rospy.get_time()
            if self.last_msg_time > 0.0 and (now - self.last_msg_time) > 5.0:
                rospy.logwarn_throttle(
                    1.0,
                    "Time exceeded: last control received %.2f s ago",
                    now - self.last_msg_time,
                )
                self.target_v = 0.0
                self.target_w = 0.0
                self.first_command = True
                self.last_msg_time = 0.0

            self._wheel_control()
            rate.sleep()

    def on_shutdown(self):
        self.target_v = 0.0
        self.target_w = 0.0
        self._wheel_control()


if __name__ == '__main__':
    node = TargetPoseTcpReceiverNode(node_name='target_pose_receiver_node')
    node.run()
