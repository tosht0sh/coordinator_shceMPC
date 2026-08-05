cmake_minimum_required(VERSION 2.8.3)
project(mpc_mocap)

find_package(catkin REQUIRED COMPONENTS
	rospy
	duckietown_msgs
	std_msgs
) 

catkin_package()