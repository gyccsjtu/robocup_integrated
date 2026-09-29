#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[robocup-init] %s\n' "$*" >&2; }
root=/workspace

[[ -f "${ROBOCUP_ROS_SETUP:?ROBOCUP_ROS_SETUP is required}" ]] || { log "ROS setup missing"; exit 64; }
source "${ROBOCUP_ROS_SETUP}"
command -v catkin_make >/dev/null || { log "catkin_make is not available in the official image"; exit 69; }
[[ -f "$root/src/robocup_environment_baseline/package.xml" ]] || { log "baseline package missing"; exit 66; }

log "clean catkin build starting"
rm -rf "$root/build" "$root/devel"
catkin_make -C "$root"
log "clean catkin build passed"
