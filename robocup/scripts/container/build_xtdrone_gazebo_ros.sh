#!/usr/bin/env bash
set -Eeo pipefail

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
source_tree="${XTD_SOURCE_DIR:-${workspace}/third_party/XTDrone}/sitl_config/gazebo_ros_pkgs"
catkin_ws="${XTD_CATKIN_WS:-${workspace}/build/xtdrone_catkin_ws}"

log() { printf '[build_xtdrone_gazebo_ros] %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

[[ -d "${source_tree}/gazebo_ros" ]] || fail "XTDrone gazebo_ros_pkgs source is missing: ${source_tree}"
source /opt/ros/noetic/setup.bash

mkdir -p "${catkin_ws}/src"
rsync -a --delete "${source_tree}/" "${catkin_ws}/src/gazebo_ros_pkgs/"

# A Windows checkout may contain CRLF shebangs. Normalize only the generated
# catkin source copy; the pinned XTDrone checkout remains untouched.
find "${catkin_ws}/src/gazebo_ros_pkgs" -type f \( \
  -name '*.py' -o -name '*.sh' -o -path '*/scripts/*' \
\) -exec sed -i 's/\r$//' {} +

log "building XTDrone gazebo_ros_pkgs in ${catkin_ws}"
catkin config --workspace "${catkin_ws}" \
  --source-space "${catkin_ws}/src" \
  --build-space "${catkin_ws}/build" \
  --devel-space "${catkin_ws}/devel" \
  --install-space "${catkin_ws}/install" \
  --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
# Gazebo plugin translation units are memory-heavy. Keep this deterministic
# and safe on Docker Desktop instead of inheriting the host CPU count.
catkin build --workspace "${catkin_ws}" --no-status --jobs 2 --parallel-packages 1

test -f "${catkin_ws}/devel/setup.bash" || fail "catkin devel setup was not generated"
log "gazebo_ros_pkgs build complete"
