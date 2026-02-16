#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class LaptopPublisher:

    def __init__(self):
        rospy.init_node('hanshake_publisher', anonymous=True)

        self.pub = rospy.Publisher(
            '/laptop_sender',
            String,
            queue_size=10
        )

        self.rate = rospy.Rate(1)  # 1 Hz

    def run(self):
        while not rospy.is_shutdown():
            msg = String()
            msg.data = "Hello from laptop"
            self.pub.publish(msg)

            rospy.loginfo("Sent: %s", msg.data)
            self.rate.sleep()


if __name__ == '__main__':
    node = LaptopPublisher()
    node.run()
