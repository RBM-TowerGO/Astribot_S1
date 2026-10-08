#!/bin/bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../../.." && pwd)"
# Isolated overlay: never rebuild or overwrite the vendor/runtime workspaces.
set +u
source /opt/astribot_ros/middle_ware/local_setup.bash
source /home/astribot/TowerGO/runtime_ws/install/local_setup.bash
set -u
mkdir -p "$WS/dual_livox_ws"
cd "$WS/dual_livox_ws"
colcon build --base-paths "$HERE/../towergo_dual_livox" --packages-select towergo_dual_livox --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --base-paths "$HERE/../towergo_dual_livox" --packages-select towergo_dual_livox --event-handlers console_direct+
colcon test-result --verbose
