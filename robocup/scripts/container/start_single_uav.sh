#!/usr/bin/env bash
# NOTE: no `set -u` on purpose. ROS's own profile scripts (e.g.
# /opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh) reference variables that
# are not yet defined while the setup files are being sourced, so nounset makes
# the launch abort with `ROS_DISTRO: unbound variable`.
set -Eeo pipefail

# The ROS setup files expect ROS_DISTRO to exist even before they are sourced.
export ROS_DISTRO="${ROS_DISTRO:-noetic}"

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
px4="${PX4_SOURCE_DIR:-${workspace}/third_party/PX4-Autopilot}"
gui="${ROBOCUP_GAZEBO_GUI:-false}"

log() { printf '[start_single_uav] %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

[[ -x "${px4}/build/px4_sitl_default/bin/px4" ]] || fail "PX4 SITL is not built; run scripts/build_single_uav.ps1"
[[ -f "${workspace}/build/xtdrone_catkin_ws/devel/setup.bash" ]] || fail "XTDrone gazebo_ros_pkgs is not built; run scripts/build_single_uav.ps1"

source "${workspace}/scripts/container/single_uav_env.sh"
cd "${px4}"
log "starting PX4 SITL + Gazebo (iris) + MAVROS; gui=${gui}"
# PX4's interactive shell reads stdin. Detached containers have no terminal,
# therefore interactive must stay false for reliable compose restart behavior.
exec roslaunch px4 mavros_posix_sitl.launch \
  gui:="${gui}" \
  verbose:=false \
  interactive:=false
