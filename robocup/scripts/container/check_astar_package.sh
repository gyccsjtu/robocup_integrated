#!/usr/bin/env bash
# Build and smoke-test the ROS package without launching any flight controller.
set -Eeo pipefail
source /workspace/scripts/container/single_uav_env.sh
nav_ws=/workspace/build/navigation_catkin_ws
mkdir -p "${nav_ws}/src"
if [[ ! -e "${nav_ws}/src/robocup_navigation" ]]; then
  ln -s /workspace/src/robocup_navigation "${nav_ws}/src/robocup_navigation"
fi
timeout 120 catkin config --workspace "${nav_ws}" --extend /workspace/build/xtdrone_catkin_ws/devel
timeout 120 catkin build --workspace "${nav_ws}" --no-status --jobs 2 --parallel-packages 1 robocup_navigation
source "${nav_ws}/devel/setup.bash"
timeout 15 rosrun robocup_navigation plan_training_route.py --help
