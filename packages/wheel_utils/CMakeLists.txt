cmake_minimum_required(VERSION 2.8.3)
project(wheel_utils)

find_package(catkin REQUIRED COMPONENTS
	rospy
)

catkin_package(
  CATKIN_DEPENDS rospy duckietown_msgs
)

