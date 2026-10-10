#!/usr/bin/env bash
set -e
source /home/astribot/astribot_sdk_aarch64/env.sh >/dev/null 2>&1
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
exec /usr/bin/python3 "$(dirname "$0")/probe_sensor_sdk.py" "$@"
