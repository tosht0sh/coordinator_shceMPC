#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

# Command magnitudes
THROTTLE_LEFT = 0.1
THROTTLE_RIGHT = 0.1
PHASE_DURATION_SEC = 2.0

class WheelControlNode(DTROS):

   def __init__(self, node_name):
       super(WheelControlNode, self).__init__(
           node_name=node_name,
           node_type=NodeType.GENERIC
       )
       vehicle_name = os.environ['VEHICLE_NAME']
       wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
       self._speed_left = THROTTLE_LEFT
       self._speed_right = THROTTLE_RIGHT
       self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

   def run(self):
       rate = rospy.Rate(10)
       start_time = rospy.Time.now().to_sec()
       while not rospy.is_shutdown():
           elapsed = rospy.Time.now().to_sec() - start_time
           cycle_time = elapsed % (2.0 * PHASE_DURATION_SEC)
           direction = 1.0 if cycle_time < PHASE_DURATION_SEC else -1.0
           message = WheelsCmdStamped(
               vel_left=direction * self._speed_left,
               vel_right=direction * self._speed_right
           )
           self._publisher.publish(message)
           rate.sleep()

   def on_shutdown(self):
       stop = WheelsCmdStamped(vel_left=0, vel_right=0)
       self._publisher.publish(stop)

if __name__ == '__main__':
   node = WheelControlNode(node_name='wheel_control_node')
   node.run()
   rospy.spin()
