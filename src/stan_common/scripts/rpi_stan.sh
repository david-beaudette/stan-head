#!/bin/sh
source /home/ubuntu/ros1_ws/install/setup.sh

roslaunch stan_common stan_head_rpi.launch &
PID=$!
wait "$PID"