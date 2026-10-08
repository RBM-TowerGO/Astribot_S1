#!/bin/bash
# Source only local setup: SDK env.sh rewrites DDS XML and must not be used here.
set -e
TELEOP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for arg in "$@"; do
  case "$arg" in
    --dry-run|--help|-h) exec /usr/bin/python3 "$TELEOP_DIR/tg_teleop.py" "$@" ;;
  esac
done
if [ ! -t 0 ]; then
  echo '需要交互终端：请使用 ssh -t S1' >&2
  exit 2
fi
source /opt/astribot_ros/middle_ware/local_setup.bash
source /opt/astribot_ros/software/local_setup.bash
if [ -f /opt/astribot_ros/third_pkg/local_setup.bash ]; then
  source /opt/astribot_ros/third_pkg/local_setup.bash
fi
TELEOP_SDK=/home/astribot/astribot_sdk_aarch64
export PYTHONPATH="$TELEOP_SDK/third_party/astribot_ros_middleware_py:$TELEOP_SDK:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$TELEOP_SDK/astribot_sdk/core/common/robotics_library_py:$TELEOP_SDK/astribot_sdk/core/common/whole_body_control/third_party:$TELEOP_SDK/third_party/drake/lib:$TELEOP_SDK/third_party/third_pkg:/opt/drake/lib:${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=S1 ASTRIBOT_SDK_ROOT="$TELEOP_SDK"
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
exec /usr/bin/python3 "$TELEOP_DIR/tg_teleop.py" "$@"
