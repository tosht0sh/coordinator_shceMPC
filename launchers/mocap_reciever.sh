#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun laptop_communication mocap_pose_receiver.py
dt-launchfile-join