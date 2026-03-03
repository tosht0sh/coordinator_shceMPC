#!/bin/bash

source /environment.sh

# Keep bot on its local ROS master so encoder topics are available.
unset ROS_HOSTNAME

dt-launchfile-init
rosrun wheel_utils pose_from_wheel_encoder.py
dt-launchfile-join
