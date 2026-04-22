#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun wheel_utils wheel_control_node.py
dt-launchfile-join