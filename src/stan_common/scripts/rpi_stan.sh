#!/bin/sh
source /home/ubuntu/ros1_ws/install/setup.sh
source /home/ubuntu/ros1_ws/install/share/stan_common/scripts/rpi_env.sh

roslaunch stan_common stan_head_rpi.launch &
PID=$!
wait "$PID"