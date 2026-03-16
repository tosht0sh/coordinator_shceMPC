# Catkin package definition for the onboard MPC runtime ROS package.
# The executable Python nodes live under `src/` and are launched from Duckietown launchers.
cmake_minimum_required(VERSION 2.8.3)
project(mpc_runtime)

find_package(catkin REQUIRED COMPONENTS
    rospy
    duckietown_msgs
    std_msgs
)

catkin_package()
