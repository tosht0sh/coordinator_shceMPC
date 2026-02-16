#!/bin/bash
set -e

source /environment.sh

unset ROS_HOSTNAME
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_IP=10.42.0.129

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun connection_bridge handshake_sub.py

# wait for app to end
dt-launchfile-join