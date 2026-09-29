#!/usr/bin/env bash
# Start PX4 SITL + Gazebo + MAVROS with a generated training world.
#
# The container command is swapped by docker-compose.training-city.yml; the
# empty-field baseline keeps using scripts/container/start_single_uav.sh and
# is otherwise untouched.
#
# Nothing is launched until the world and its metadata have been re-validated
# on disk, so a failed generation can never end up flying a previous map.
set -Eeuo pipefail

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
px4="${PX4_SOURCE_DIR:-${workspace}/third_party/PX4-Autopilot}"
gui="${ROBOCUP_GAZEBO_GUI:-false}"
state="${workspace}/build/training_city"
pkg="${workspace}/src/robocup_training_worlds"

log() { printf '[start_training_city] %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

[[ -x "${px4}/build/px4_sitl_default/bin/px4" ]] || fail "PX4 SITL is not built; run scripts/build_single_uav.ps1"
[[ -f "${workspace}/build/xtdrone_catkin_ws/devel/setup.bash" ]] || fail "XTDrone gazebo_ros_pkgs is not built; run scripts/build_single_uav.ps1"

world_file="$(cat "${state}/current_world.txt" 2>/dev/null || true)"
metadata_file="$(cat "${state}/current_metadata.txt" 2>/dev/null || true)"
[[ -n "$world_file" ]] || fail "no training world recorded; run scripts/generate_training_city.ps1 first"
[[ -f "$world_file" ]] || fail "recorded world is missing: $world_file"
[[ -f "$metadata_file" ]] || fail "recorded metadata is missing: $metadata_file"

source "${workspace}/scripts/container/single_uav_env.sh"
# The generator package is used straight from the source tree; no rebuild of
# PX4 or the XTDrone workspace is required.
export ROS_PACKAGE_PATH="${workspace}/src:${ROS_PACKAGE_PATH:-}"
rospack find robocup_training_worlds >/dev/null 2>&1 \
    || fail "package robocup_training_worlds is not visible on ROS_PACKAGE_PATH"

log "validating ${world_file}"
python3 "${pkg}/scripts/validate_training_city.py" \
    --world "$world_file" --metadata "$metadata_file" \
    || fail "refusing to start with an invalid training world"

ROBOCUP_SPAWN_X=0
ROBOCUP_SPAWN_Y=0
ROBOCUP_SPAWN_Z=0
ROBOCUP_SPAWN_YAW=0
if [[ -f "${state}/spawn.env" ]]; then
    # shellcheck disable=SC1091
    source "${state}/spawn.env"
fi

cd "${px4}"
log "starting PX4 SITL + Gazebo with the generated world; gui=${gui}"
log "spawn x=${ROBOCUP_SPAWN_X} y=${ROBOCUP_SPAWN_Y} yaw=${ROBOCUP_SPAWN_YAW}"
# PX4's interactive shell reads stdin. Detached containers have no terminal,
# therefore interactive must stay false for reliable compose restart behavior.
exec roslaunch robocup_training_worlds training_city_single_uav.launch \
  world:="${world_file}" \
  gui:="${gui}" \
  verbose:=false \
  interactive:=false \
  x:="${ROBOCUP_SPAWN_X}" \
  y:="${ROBOCUP_SPAWN_Y}" \
  z:="${ROBOCUP_SPAWN_Z}"
