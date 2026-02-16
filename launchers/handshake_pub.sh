#!/bin/bash
set -e

source /environment.sh

unset ROS_HOSTNAME
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.129

# Optional but helpful
# export ROS_LOG_DIR="/data/ros/log"

dt-launchfile-init
rosrun connection_bridge handshake_pub.py
dt-launchfile-join