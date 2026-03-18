#!/bin/bash
# Duckietown launcher for the onboard MPC runtime node.
# Use this on the bot after the package has been built into the image.
set -e

source /environment.sh

unset ROS_HOSTNAME
export MPC_SCHEDULE_BIND_IP=${MPC_SCHEDULE_BIND_IP:-0.0.0.0}
export MPC_SCHEDULE_PORT=${MPC_SCHEDULE_PORT:-5007}
export MPC_TELEMETRY_PORT=${MPC_TELEMETRY_PORT:-5008}

# initialize launch file
dt-launchfile-init

rosrun mpc_runtime bot_mpc_node.py

dt-launchfile-join
