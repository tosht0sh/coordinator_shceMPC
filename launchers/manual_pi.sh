#!/bin/bash
# Duckietown launcher for manual wheel-PI tuning on the bot.
set -e

source /environment.sh

unset ROS_HOSTNAME

dt-launchfile-init

rosrun mpc_runtime manual_pi_node.py

dt-launchfile-join
