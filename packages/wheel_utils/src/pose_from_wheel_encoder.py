#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
from std_msgs.msg import Float64MultiArray, String

# laptop - 192.168.1.197


class WheelEncoderReaderNode(DTROS):
    """
    Purpose:
      Estimate robot pose (x, y, theta) from left and right wheel encoder ticks.

    ROS topics used:
    - publishers:
      - /<vehicle>/pose_reader:
        Estimated robot pose as Float64MultiArray [x, y, theta].
    - subscribers:
      - /<vehicle>/left_wheel_encoder_node/tick:
        Left wheel encoder ticks.
      - /<vehicle>/right_wheel_encoder_node/tick:
        Right wheel encoder ticks.
    """

    def __init__(self, node_name):
        super(WheelEncoderReaderNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        self._ticks_left = None
        self._ticks_right = None
        self.sub_left = rospy.Subscriber(
            self._left_encoder_topic,
            WheelEncoderStamped,
            self.callback_left
        )
        self.sub_right = rospy.Subscriber(
            self._right_encoder_topic,
            WheelEncoderStamped,
            self.callback_right
        )
        self.pose_pub = rospy.Publisher(f"/{self._vehicle_name}/pose_reader", Float64MultiArray, queue_size=10)

        # Geometry/calibration (override via ROS params if needed).
        self._wheel_radius = rospy.get_param("~wheel_radius", 0.0318)
        self._axis_length = rospy.get_param("~axis_length", 0.09716)
        # self.first_callback = True
        self.left_enc_resolution = None
        self.left_meters_per_tick = None

        self.right_enc_resolution = None
        self.right_meters_per_tick = None


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_l = None
        self.prev_r = None

        # Wheel scale factors; use to remove heading drift from wheel mismatch.
        # self.k_l = 0.5
        # self.k_r = 2.4
        self.k_l = 1.0
        self.k_r = 1.0

    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        self._ticks_left = data.data
        if self.left_enc_resolution is None:
            self.left_enc_resolution = data.resolution
            self.left_meters_per_tick = (2 * math.pi * self._wheel_radius) / self.left_enc_resolution

    def callback_right(self, data):
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        self._ticks_right = data.data
        if self.right_enc_resolution is None:
            self.right_enc_resolution = data.resolution
            self.right_meters_per_tick = (2 * math.pi * self._wheel_radius) / self.right_enc_resolution

    def position_calc(self):

        if self._ticks_left is None or self._ticks_right is None:
            return

        # previous tick 
        if self.prev_l is None:
            self.prev_l = self._ticks_left
            self.prev_r = self._ticks_right
            return

        # update delta for current ticks
        dl = (self._ticks_left - self.prev_l) * self.left_meters_per_tick * self.k_l
        dr = (self._ticks_right - self.prev_r) * self.right_meters_per_tick * self.k_r

        # new kinematics
        d = (dr + dl) / 2
        dtheta = (dr - dl) / self._axis_length

        self.x += d * math.cos(self.theta + dtheta / 2)
        self.y += d * math.sin(self.theta + dtheta / 2)
        self.theta += (dtheta * 2.05)
        # Keep heading bounded to [0, 2*pi).
        self.theta = self.theta % (2 * math.pi)

        # update previous ticks values
        self.prev_l = self._ticks_left
        self.prev_r = self._ticks_right

        
    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self._ticks_left is not None and self._ticks_right is not None:

                self.position_calc()

                msg = (
                    # f"Curr ticks: "
                    # f"{self._ticks_left}, {self._ticks_right} \n"
                    # f"Prev ticks"
                    # f"{self.prev_l}, {self.prev_r} \n"
                    f"Position [x, y, theta]: "
                    f"{self.x:.3f}, {self.y:.3f}, {self.theta:.3f}"
                )
                rospy.loginfo(msg)
                pose_msg = Float64MultiArray(data=[round(self.x, 3), round(self.y, 3), round(self.theta, 3)])
                self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
   node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
   node.run()
   rospy.spin()
