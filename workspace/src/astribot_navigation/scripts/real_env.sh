#!/usr/bin/env bash
# Source from a fresh S1 bash shell. Never constructs an SDK or starts nodes.
TOWERGO_REAL_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TOWERGO_PROJECT=$(cd "$TOWERGO_REAL_REPO/../../.." && pwd)
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
source /opt/astribot_ros/middle_ware/setup.bash
source /opt/astribot_ros/software/setup.bash
if [ -f "$TOWERGO_PROJECT/runtime_ws/install/setup.bash" ]; then
  source "$TOWERGO_PROJECT/runtime_ws/install/setup.bash"
fi
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export TOWERGO_WS="$TOWERGO_PROJECT/workspace"
export TOWERGO_REPO="$TOWERGO_REAL_REPO"
