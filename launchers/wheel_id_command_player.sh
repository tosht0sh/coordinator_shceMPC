#!/bin/bash
# Duckietown launcher for scripted wheel-identification command playback.
set -e

source /environment.sh

# Keep the command player on the bot's local ROS master.
unset ROS_HOSTNAME

dt-launchfile-init
rosrun wheel_utils wheel_id_command_player.py
dt-launchfile-join
