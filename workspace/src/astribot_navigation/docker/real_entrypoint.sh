#!/usr/bin/env bash
set -e
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /slam_overlay/install/setup.bash
source /real_ws/install/setup.bash
exec "$@"
