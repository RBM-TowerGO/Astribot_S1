#!/bin/bash
set -e
source /opt/astribot_ros/middle_ware/local_setup.bash
source /opt/astribot_ros/software/local_setup.bash
export PYTHONNOUSERSITE=1
exec /usr/bin/python3 "$(dirname "$0")/export_collection.py" "$@"
