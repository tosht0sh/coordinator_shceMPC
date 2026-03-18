#!/bin/bash
set -e

source /environment.sh

# Keep bot on its local ROS master.
# UDP destination is configured with LAPTOP_IP / LAPTOP_UDP_PORT env vars.
export LAPTOP_IP=192.168.1.192
export LAPTOP_UDP_PORT=5005
unset ROS_HOSTNAME

# Optional but helpful
# export ROS_LOG_DIR="/data/ros/log"

dt-launchfile-init
rosrun laptop_communication pose_sender_udp.py
dt-launchfile-join
