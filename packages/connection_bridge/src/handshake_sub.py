#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class HanshakeSubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(HanshakeSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.sub = rospy.Subscriber('/laptop_sender', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard '%s'", data.data)

if __name__ == '__main__':
    # create the node
    node = HanshakeSubscriberNode(node_name='hs_subscriber_node')
    # keep spinning
    rospy.spin()