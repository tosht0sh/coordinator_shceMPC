#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Imu

class ImuReaderNode(DTROS):

    def __init__(self, node_name):
        super(ImuReaderNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        self.vehicle_name = os.environ['VEHICLE_NAME']
        self._imu_reader_topic = f"/duck1/imu_node/data"

        self._ang_vel = None
        self._lin_acc = None

        self._imu_subscirber = rospy.Subscriber(self._imu_reader_topic, Imu, self.callback)
        self.loginfo(f"Subscribing to IMU: {self._imu_reader_topic}")

    
    def callback(self, msg):
        self._ang_vel = msg.angular_velocity
        self._lin_acc = msg.linear_acceleration
        self.msg_data = msg.header
        # self.loginfo(f"{self._ang_vel}")

        # self.loginfo("IMU callback triggered (receiving messages)")

    def run(self):
       rate = rospy.Rate(20)
       while not rospy.is_shutdown():
            if self._ang_vel is not None or self._lin_acc is not None:
                # self.loginfo("Waiting for first IMU message...")
                # rate.sleep()
                # continue
                
                log = (
                    f"Angular Velocity (x,y,z): "
                    f"({self._ang_vel.x:.3f}, {self._ang_vel.y:.3f}, {self._ang_vel.z:.3f}) | "
                    f"Linear Acceleration (x,y,z): "
                    f"({self._lin_acc.x:.3f}, {self._lin_acc.y:.3f}, {self._lin_acc.z:.3f})"
                )
                rospy.loginfo(log)
            rate.sleep()

            

if __name__ == '__main__':
    node = ImuReaderNode(node_name='imu_reader_node')
    node.run()
    rospy.spin()
