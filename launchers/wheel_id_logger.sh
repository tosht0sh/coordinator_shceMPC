#!/bin/bash
# Duckietown launcher for wheel identification logging on the bot.
set -e

source /environment.sh

# Keep the logger on the bot's local ROS master so encoder topics are available.
unset ROS_HOSTNAME

dt-launchfile-init
rosrun mpc_runtime wheel_id_logger.py
dt-launchfile-join
