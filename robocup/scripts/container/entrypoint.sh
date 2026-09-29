#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[robocup-entrypoint] %s\n' "$*" >&2; }

if [[ ! -f "${ROBOCUP_ROS_SETUP:?ROBOCUP_ROS_SETUP is required}" ]]; then
  log "ROS setup file does not exist: ${ROBOCUP_ROS_SETUP}"
  exit 64
fi

source "${ROBOCUP_ROS_SETUP}"
if [[ -n "${ROBOCUP_CATKIN_SETUP:-}" ]]; then
  if [[ ! -f "${ROBOCUP_CATKIN_SETUP}" ]]; then
    log "Configured catkin setup file does not exist: ${ROBOCUP_CATKIN_SETUP}"
    exit 64
  fi
  source "${ROBOCUP_CATKIN_SETUP}"
fi

log "container ready; official simulation is started explicitly by scripts/start.ps1"
exec tail -f /dev/null
