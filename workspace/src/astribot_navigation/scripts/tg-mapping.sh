#!/bin/bash
set -e
MAPPING_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for arg in "$@"; do
  case "$arg" in -h|--help) exec /usr/bin/python3 "$MAPPING_DIR/tg_mapping.py" "$@" ;; esac
done
MAPPING_OFFLINE=0
for arg in "$@"; do
  case "$arg" in finalize|--replay) MAPPING_OFFLINE=1 ;; esac
done
if [ "$MAPPING_OFFLINE" = 0 ] && [ ! -t 0 ]; then
  echo '在线遥操需要 ssh -t 的交互终端' >&2
  exit 2
fi
source /opt/astribot_ros/middle_ware/local_setup.bash
source /opt/astribot_ros/software/local_setup.bash
source /home/astribot/TowerGO/runtime_ws/install/local_setup.bash
# Finalization is compatible with old sessions and does not need the fusion overlay.
if [ "${1:-}" != "finalize" ]; then
  DUAL_SETUP="$MAPPING_DIR/../../../dual_livox_ws/install/local_setup.bash"
  if [ ! -f "$DUAL_SETUP" ]; then
    echo '缺少双雷达适配器，请先运行 scripts/build_dual_livox.sh' >&2
    exit 2
  fi
  source "$DUAL_SETUP"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if [ "$MAPPING_OFFLINE" = 1 ]; then
  export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
  unset FASTRTPS_DEFAULT_PROFILES_FILE
else
  export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
fi
# Bound numerical work so preview/finalization cannot consume all Orin cores.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec /usr/bin/python3 "$MAPPING_DIR/tg_mapping.py" "$@"
