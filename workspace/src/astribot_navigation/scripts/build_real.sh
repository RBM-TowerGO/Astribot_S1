#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PROJECT=$(cd "$REPO/../../.." && pwd)
REAL_WS=${TOWERGO_RUNTIME_WS:-$PROJECT/runtime_ws}
# ROS setup scripts are not consistently nounset-clean.
set +u
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
source /opt/astribot_ros/middle_ware/setup.bash
source /opt/astribot_ros/software/setup.bash
set -u
python3 "$REPO/scripts/fetch_real_dependencies.py" --directory "$PROJECT/workspace/data/cache/real_dependencies"
mkdir -p "$REAL_WS"
if [ ! -f "$REAL_WS/build_python/em.py" ]; then
  python3 -m pip install --no-deps --target "$REAL_WS/build_python" 'empy==3.3.4'
fi
# Vendor host has EmPy 4 in its user site; Humble generators require EmPy 3.
export PYTHONPATH="$REAL_WS/build_python:${PYTHONPATH:-}"
cd "$REAL_WS"
MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --parallel-workers 1 --cmake-clean-cache \
  --base-paths "$REPO/towergo_real" "$REPO/towergo_livox_adapter" "$PROJECT/workspace/data/cache/real_dependencies" \
  --allow-overriding livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
dpkg-query -W > "$REAL_WS/dependency-versions.txt"
cp "$PROJECT/workspace/data/cache/real_dependencies/source_versions.json" "$REAL_WS/source_versions.json"
echo "Built observation/control overlay at $REAL_WS. Hardware remains disabled until verified."
