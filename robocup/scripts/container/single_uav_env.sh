#!/usr/bin/env bash

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
px4="${PX4_SOURCE_DIR:-${workspace}/third_party/PX4-Autopilot}"
catkin_ws="${XTD_CATKIN_WS:-${workspace}/build/xtdrone_catkin_ws}"

source /opt/ros/noetic/setup.bash
if [[ -f "${catkin_ws}/devel/setup.bash" ]]; then
  source "${catkin_ws}/devel/setup.bash"
fi
# PX4's setup script appends to these variables without defining defaults.
# Define empty values so callers may safely enable `set -u`.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "${px4}/Tools/setup_gazebo.bash" "${px4}" "${px4}/build/px4_sitl_default"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${px4}:${px4}/Tools/sitl_gazebo"
