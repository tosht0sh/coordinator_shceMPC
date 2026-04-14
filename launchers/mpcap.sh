#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun mpc_mocap main.py
dt-launchfile-join