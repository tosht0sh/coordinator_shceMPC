#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun first_pkg subscriber.py

# wait for app to end
dt-launchfile-join