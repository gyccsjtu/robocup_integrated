#!/usr/bin/env bash
set -Eeo pipefail
exec 9>/tmp/robocup_single_uav_demo.lock
flock -n 9 || { echo 'CONTROL_CONFLICT: demo is already running' >&2; exit 3; }
source /workspace/scripts/container/single_uav_env.sh
if [[ "${1:-}" == "--skip-build" ]]; then
  shift
else
  bash /workspace/scripts/container/check_astar_package.sh
fi
source /workspace/build/navigation_catkin_ws/devel/setup.bash
exec python3 /workspace/tests/sitl_single_wall.py "$@"
