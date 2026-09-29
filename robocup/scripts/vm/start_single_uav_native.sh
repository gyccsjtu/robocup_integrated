#!/usr/bin/env bash
# Start the existing single-UAV PX4/Gazebo/MAVROS scene without Docker.
set -Eeuo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
px4_dir="${repo_root}/third_party/PX4-Autopilot"
xtdrone_ws="${repo_root}/build/xtdrone_catkin_ws"

[[ -x "${px4_dir}/build/px4_sitl_default/bin/px4" ]] || {
  echo 'PX4 SITL is not built. Run scripts/vm/bootstrap_ubuntu20.sh first.' >&2
  exit 2
}
[[ -f "${xtdrone_ws}/devel/setup.bash" ]] || {
  echo 'XTDrone Gazebo ROS packages are not built. Run scripts/vm/bootstrap_ubuntu20.sh first.' >&2
  exit 2
}

export ROBOCUP_WORKSPACE="${repo_root}"
export PX4_SOURCE_DIR="${px4_dir}"
export XTD_CATKIN_WS="${xtdrone_ws}"
export ROBOCUP_GAZEBO_GUI="${ROBOCUP_GAZEBO_GUI:-true}"
exec bash "${repo_root}/scripts/container/start_single_uav.sh"
