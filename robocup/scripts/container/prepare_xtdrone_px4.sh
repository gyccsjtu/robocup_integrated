#!/usr/bin/env bash
set -Eeuo pipefail

workspace="${ROBOCUP_WORKSPACE:-/workspace}"
xtdrone="${XTD_SOURCE_DIR:-${workspace}/third_party/XTDrone}"
px4="${PX4_SOURCE_DIR:-${workspace}/third_party/PX4-Autopilot}"
expected_xtd="${XTD_EXPECTED_COMMIT:-62339a816ef815113a0366a62e8aca4be3000f80}"
expected_px4="${PX4_EXPECTED_COMMIT:-46a12a09bf11c8cbafc5ad905996645b4fe1a9df}"

log() { printf '[prepare_xtdrone_px4] %s\n' "$*"; }
fail() { log "ERROR: $*" >&2; exit 1; }

[[ -d "${xtdrone}/.git" ]] || fail "XTDrone source is missing: ${xtdrone}"
[[ -d "${px4}/.git" ]] || fail "PX4 source is missing: ${px4}"

actual_xtd="$(git -c safe.directory="${xtdrone}" -C "${xtdrone}" rev-parse HEAD)"
actual_px4="$(git -c safe.directory="${px4}" -C "${px4}" rev-parse HEAD)"
[[ "${actual_xtd}" == "${expected_xtd}" ]] || fail "XTDrone commit ${actual_xtd} != ${expected_xtd}"
[[ "${actual_px4}" == "${expected_px4}" ]] || fail "PX4 commit ${actual_px4} != ${expected_px4}"

log "applying the official XTDrone 1_13_2 overlay"
cp -a "${xtdrone}/sitl_config/init.d-posix/." "${px4}/ROMFS/px4fmu_common/init.d-posix/"
cp -a "${xtdrone}/sitl_config/launch/." "${px4}/launch/"
cp -a "${xtdrone}/sitl_config/worlds/." "${px4}/Tools/sitl_gazebo/worlds/"
cp -a "${xtdrone}/sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.cpp" "${px4}/Tools/sitl_gazebo/src/"
cp -a "${xtdrone}/sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.hh" "${px4}/Tools/sitl_gazebo/include/"
cp -a "${xtdrone}/sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.cpp" "${px4}/Tools/sitl_gazebo/src/"
cp -a "${xtdrone}/sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.h" "${px4}/Tools/sitl_gazebo/include/"
cp -a "${xtdrone}/sitl_config/CMakeLists.txt" "${px4}/Tools/sitl_gazebo/CMakeLists.txt"
cp -a "${xtdrone}/sitl_config/models/." "${px4}/Tools/sitl_gazebo/models/"

# The sources may be checked out on Windows with core.autocrlf=true. PX4 runs
# these files through /bin/sh, so normalize only the generated PX4 overlay.
find "${px4}/ROMFS/px4fmu_common/init.d-posix" -type f -exec sed -i 's/\r$//' {} +
find "${px4}/launch" -type f -name '*.launch' -exec sed -i 's/\r$//' {} +

test -f "${px4}/launch/indoor1.launch" || fail "indoor1.launch was not installed"
test -f "${px4}/Tools/sitl_gazebo/models/typhoon_h480/model.config" || fail "typhoon_h480 model was not installed"
test -f "${px4}/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink" || fail "PX4 startup overlay was not installed"
log "overlay ready: XTDrone=${actual_xtd} PX4=${actual_px4}"
