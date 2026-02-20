#!/bin/bash
set -e

source /environment.sh

# Keep bot on local ROS master.
unset ROS_HOSTNAME
export ROBOT_TCP_PORT=5006

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun connection_bridge handshake_sub.py

# wait for app to end
dt-launchfile-join
