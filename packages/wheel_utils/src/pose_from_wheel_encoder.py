#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, Pose2DStamped, Twist2DStamped
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import Imu
import numpy as np

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

        self.rate = 30
        self.dt = 1 / self.rate

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
        # self._vel_to_pose_topic = f"/{self._vehicle_name}/velocity_to_pose_node/pose"
        # self.vel_to_pose_node = rospy.Subscriber(
        #     self._vel_to_pose_topic,
        #     Pose2DStamped,
        #     self.callback_vel_to_pose
        # )

        # imu settings
        self._imu_reader_topic = f"/{self._vehicle_name}/imu_node/data"

        self._ang_vel = None

        self._imu_subscirber = rospy.Subscriber(self._imu_reader_topic, Imu, self.callback_imu)
        # self._imu_rate = 30

        # kinematics node pose 
        self._vel_reader_topic = f"/{self._vehicle_name}/kinematics_node/velocity"

        self._lin_vel_kn = 0.0
        self._ang_vel_kn = 0.0

        self._kin_node_subscirber = rospy.Subscriber(self._vel_reader_topic, Twist2DStamped, self.callback_kn)

        # complementary filter setup for yaw
        self._yaw_alpha = 0.97
        self._yaw_bias = 0.0                                                            # TODO: make this dynamic
        self._bias_samples = []
        self._bias_ready = False


    def callback_kn(self, msg):
        self._lin_vel_kn = msg.v
        self._ang_vel_kn = msg.omega

    # def callback_imu(self, msg):
    #     self._ang_vel = msg.angular_velocity.z
    #     self._ang_vel = self._ang_vel - self._yaw_bias
    #     # TODO: maybe add a low psss filter here?

    #     self.gyro_z = self.gyro_z_prev + (self._ang_vel / self._imu_rate)
    #     self.gyro_z_prev = self.gyro_z

    def callback_imu(self, msg):
        wz = msg.angular_velocity.z

        if not self._bias_ready:    
            self._bias_samples.append(wz)
            # rospy.loginfo(self._bias_samples)

            if len(self._bias_samples) >= 100:
                self._yaw_bias = sum(self._bias_samples) / len(self._bias_samples)
                self._bias_ready = True
                rospy.loginfo(f"Estimated gyro bias: {self._yaw_bias:.6f} rad/s")
        
        self._ang_vel = wz - self._yaw_bias

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

        if self._ticks_left is None or self._ticks_right is None or self._ang_vel is None or self._ang_vel is None or not self._bias_ready:
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

        theta_enc = self.theta + dtheta
        # wrap to bound to [-pi, pi]
        theta_enc = (theta_enc + math.pi) % (2 * math.pi) - math.pi

        theta_gyro = self.theta + self._ang_vel * self.dt
        # wrap to bound to [-pi, pi]
        theta_gyro = (theta_gyro + math.pi) % (2 * math.pi) - math.pi

        theta_new = self._yaw_alpha * theta_gyro + (1.0 - self._yaw_alpha) * theta_enc
        # wrap to bound to [-pi, pi]
        theta_new = (theta_new + math.pi) % (2 * math.pi) - math.pi

        # midpoint position update
        theta_mid = self.theta + 0.5 * (theta_new - self.theta)
        theta_mid = (theta_mid + math.pi) % (2 * math.pi) - math.pi

        # update x and y based on complemntary filter theta
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        # update theta
        self.theta = theta_new

        # update previous ticks values
        self.prev_l = self._ticks_left
        self.prev_r = self._ticks_right

        
    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():

            # print(f"Ticks left, Ticks Right: {self._ticks_left}, {self._ticks_right}")            
            if self._ticks_left is not None and self._ticks_right is not None:

                self.position_calc()

                if self._bias_ready:
                    msg = (
                        f"Encoder pose [x, y, theta]: "
                        f"{self.x:.3f}, {self.y:.3f}, {self.theta:.3f} | "
                        # f"Angular Velocity (x,y,z): "
                        # f"({self._ang_vel.x:.3f}, {self._ang_vel.y:.3f}, {self._ang_vel.z:.3f}) | "
                        # f"Linear Acceleration (x,y,z): "
                        # f"({self._lin_acc.x:.3f}, {self._lin_acc.y:.3f}, {self._lin_acc.z:.3f})"
                        f"Imu (angular velocity): "
                        f"({self._ang_vel:.3f}) |"
                        f"Kinematics Node (v, omega): "
                        f"[{self._lin_vel_kn:.3f}, {self._ang_vel_kn:.3f}]"
                        
                    )
                    rospy.loginfo(msg)
                    pose_msg = Float64MultiArray(data=[round(self.x, 3), round(self.y, 3), round(self.theta, 3)])
                    self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
   node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
   node.run()
   rospy.spin()
