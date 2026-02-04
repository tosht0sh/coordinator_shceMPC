#!/bin/bash
source /environment.sh
dt-launchfile-init
rosrun first_pkg publisher.py
dt-launchfile-join