#!/bin/bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$REPO/astribot_nav_bridge:$REPO/astribot_nav_sim:$REPO/astribot_nav_api:$REPO/towergo_real:${PYTHONPATH:-}"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest "$REPO/tests" -q -p no:cacheprovider "$@"
