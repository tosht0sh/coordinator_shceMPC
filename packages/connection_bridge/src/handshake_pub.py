#!/usr/bin/env python3

import os
import time
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType

class HanshakePublisherNode(DTROS):
    def __init__(self, node_name):
        super(HanshakePublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.publisher = rospy.Publisher('/hanshake_sender', String, queue_size=5)

    def run(self):
        rate = rospy.Rate(1)
        message = f"Hello from {self._vehicle_name}....."
        while not rospy.is_shutdown():
            self.loginfo(f'Publishing greetings to roscore')
            self.publisher.publish(message)
            rate.sleep()

if __name__ == '__main__':
    node = HanshakePublisherNode(node_name='hanshake_publisher')
    node.run()
    rospy.spin()