#!/usr/bin/env bash
set -e
NAV_SCRIPTS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
for arg in "$@"; do
  case "$arg" in -h|--help) PYTHONPATH="$NAV_SCRIPTS${PYTHONPATH:+:$PYTHONPATH}" exec /usr/bin/python3 -m real_navigation.session "$@" ;; esac
done
source "$NAV_SCRIPTS/real_env.sh"
NAV_WORKSPACE=$(cd "$NAV_SCRIPTS/../../.." && pwd)
for setup in "$NAV_WORKSPACE/dual_livox_ws/install/local_setup.bash" "$NAV_WORKSPACE/offline_ws/install/local_setup.bash" "$NAV_WORKSPACE/navigation_ws/install/local_setup.bash"; do
  if [ -f "$setup" ]; then source "$setup"; fi
done
export PYTHONPATH="$NAV_SCRIPTS:$NAV_SCRIPTS/../towergo_real:$NAV_SCRIPTS/../astribot_nav_sim${PYTHONPATH:+:$PYTHONPATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
for arg in "$@"; do
  if [ "$arg" = '--replay' ]; then
    export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
    unset FASTRTPS_DEFAULT_PROFILES_FILE
  fi
done
NAV_SDK=/home/astribot/astribot_sdk_aarch64
export PYTHONPATH="$NAV_SDK/third_party/astribot_ros_middleware_py:$NAV_SDK:$PYTHONPATH"
export LD_LIBRARY_PATH="$NAV_SDK/astribot_sdk/core/common/robotics_library_py:$NAV_SDK/astribot_sdk/core/common/whole_body_control/third_party:$NAV_SDK/third_party/drake/lib:$NAV_SDK/third_party/third_pkg:/opt/drake/lib:${LD_LIBRARY_PATH:-}"
export ROBOT_TYPE=S1 ASTRIBOT_SDK_ROOT="$NAV_SDK"
exec /usr/bin/python3 -m real_navigation.session "$@"
