#!/bin/sh

. ~/ros1_ws/install/setup.sh
export ROS_IP="192.168.0.80"
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=stanhead

exec "$@"