#!/bin/bash

source /environment.sh

dt-launchfile-init
rosrun wheel_utils wheel_encoder_reader_node.py
dt-launchfile-join