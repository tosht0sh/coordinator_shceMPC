#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, Pose2DStamped
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
        self._wheel_radius = 0.0318
        self._axis_length = 0.1
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
        self.k_l = 1.0
        self.k_r = 1.0

        # node for pose from kinematics node
        self._vel_to_pose_topic = f"/{self._vehicle_name}/velocity_to_pose_node/pose"
        self.vel_to_pose_node = rospy.Subscriber(
            self._vel_to_pose_topic,
            Pose2DStamped,
            self.callback_vel_to_pose
        )

        self.vtp_x = 0.0
        self.vtp_y = 0.0
        self.vtp_theta = 0.0

        self.vtp_x_actual = 0.0
        self.vtp_y_actual = 0.0
        self.vtp_theta_actual = 0.0

        self.vtp_x_origin = None
        self.vtp_y_origin = None
        self.vtp_theta_origin = None

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

    def callback_vel_to_pose(self, data):
        self.vtp_x_actual = data.x
        self.vtp_y_actual = data.y
        self.vtp_theta_actual = data.theta

        if self.vtp_x_origin is None:
            self.vtp_x_origin = data.x
            self.vtp_y_origin = data.y
            self.vtp_theta_origin = data.theta

        self.vtp_x = self.vtp_x_actual - self.vtp_x_origin
        self.vtp_y = self.vtp_y_actual - self.vtp_y_origin
        self.vtp_theta = self.vtp_theta_actual - self.vtp_theta_origin
        self.vtp_theta = (self.vtp_theta + math.pi) % (2 * math.pi) - math.pi

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

        # dl = (self._ticks_left - self.prev_l) * self.left_meters_per_tick
        # dr = (self._ticks_right - self.prev_r) * self.right_meters_per_tick 

        # new kinematics
        d = (dr + dl) / 2
        dtheta = (dr - dl) / self._axis_length

        self.x += d * math.cos(self.theta + dtheta / 2)
        self.y += d * math.sin(self.theta + dtheta / 2)
        self.theta += (dtheta * 2.0)
        # Keep heading bounded to [-pi, pi] for symmetric CW/CCW comparison.
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # update previous ticks values
        self.prev_l = self._ticks_left
        self.prev_r = self._ticks_right

        
    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            
            if self._ticks_left is not None and self._ticks_right is not None:

                self.position_calc()

                msg = (
                    f"Encoder pose [x, y, theta]: "
                    f"{self.x:.3f}, {self.y:.3f}, {self.theta:.3f} | "
                    f"Velocity pose (relative) [x, y, theta]: "
                    f"{self.vtp_x:.3f}, {self.vtp_y:.3f}, {self.vtp_theta:.3f}"
                )
                rospy.loginfo(msg)
                pose_msg = Float64MultiArray(data=[round(self.x, 3), round(self.y, 3), round(self.theta, 3)])
                self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
   node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
   node.run()
   rospy.spin()
