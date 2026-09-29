#!/usr/bin/env bash
set -Eeo pipefail
exec 9>/tmp/robocup_single_uav_demo.lock
flock -n 9 || { echo 'CONTROL_CONFLICT: demo is already running' >&2; exit 3; }
source /workspace/scripts/container/single_uav_env.sh
# Separate incremental workspace: no PX4/plugin rebuild or teammate packages.
nav_ws=/workspace/build/navigation_catkin_ws
mkdir -p "${nav_ws}/src"
if [[ ! -e "${nav_ws}/src/robocup_navigation" ]]; then
  ln -s /workspace/src/robocup_navigation "${nav_ws}/src/robocup_navigation"
fi
timeout 120 catkin config --workspace "${nav_ws}" --extend /workspace/build/xtdrone_catkin_ws/devel
timeout 120 catkin build --workspace "${nav_ws}" --no-status --jobs 2 --parallel-packages 1 robocup_navigation
source "${nav_ws}/devel/setup.bash"
config_file="${1:-/workspace/src/robocup_navigation/config/single_uav_waypoints.yaml}"
# rosrun preserves node exit status; roslaunch may hide a child's failure.
exec rosrun robocup_navigation uav_navigation_node.py "_config_file:=${config_file}"
