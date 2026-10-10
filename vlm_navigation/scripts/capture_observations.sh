#!/usr/bin/env bash
set -e
source /opt/astribot_ros/middle_ware/setup.bash
source /opt/astribot_ros/software/astribot_camera/share/astribot_camera/local_setup.bash
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 "$(dirname "$0")/capture_observations.py" "$@"
