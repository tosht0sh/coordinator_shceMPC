#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped

class WheelEncoderReaderNode(DTROS):
    '''
    Currently prints number of ticks
    TODO: output position x,y,theta
    '''

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

        # for 
        self._wheel_radius = 0.0325
        self._axis_length = 0.1
        # self.first_callback = True
        self.enc_resolution = None
        self.meters_per_tick = None


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_l = None
        self.prev_r = None

    def callback_left(self, data):
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        self._ticks_left = data.data
        if self.enc_resolution is None:
            self.enc_resolution = data.resolution
            self.meters_per_tick = (2 * math.pi * self._wheel_radius) / self.enc_resolution

    def callback_right(self, data):
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        self._ticks_right = data.data

    def position_calc(self):

        if self._ticks_left is None and self._ticks_right is None:
            return

        # previous tick 
        if self.prev_l is None:
            self.prev_l = self._ticks_left
            self.prev_r = self._ticks_right
            return

        # update delta for current ticks
        dl = (self._ticks_left - self.prev_l) * self.meters_per_tick
        dr = (self._ticks_right -self.prev_r) * self.meters_per_tick

        # new kinematics
        d = (dr + dl) / 2
        dtheta = (dr - dl) / self._axis_length   

        self.x += d * math.cos(self.theta + dtheta / 2)
        self.y += d * math.sin(self.theta + dtheta / 2)
        self.theta += dtheta

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
            rate.sleep()

if __name__ == '__main__':
   node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
   node.run()
   rospy.spin()