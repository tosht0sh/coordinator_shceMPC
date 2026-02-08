#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun imu_reader imu_reader.py
dt-launchfile-join