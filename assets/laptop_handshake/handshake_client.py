#!/usr/bin/env python3
import os
import time
import rospy
from std_msgs.msg import String

class LaptopSubscriber:
    def __init__(self):
        rospy.init_node('hanshake_sub')

        self.topic = rospy.get_param("~topic", "/hanshake_sender")

        self.sub = rospy.Subscriber(
            self.topic,
            String,
            self.callback,
            queue_size=5
        )

        rospy.loginfo("Listening on %s", self.topic)


    def callback(self, msg: String):
        rospy.loginfo('Heard: %s', msg.data)

if __name__=='__main__':
    node=LaptopSubscriber()
    rospy.spin()