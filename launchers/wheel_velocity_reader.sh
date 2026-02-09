#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun wheel_utils wheel_speed.py
dt-launchfile-join