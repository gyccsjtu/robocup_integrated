#!/usr/bin/env bash
set -Eeuo pipefail

log() { printf '[robocup-start] %s\n' "$*" >&2; }
assets="${ROBOCUP_OFFICIAL_ASSETS_DIR:?ROBOCUP_OFFICIAL_ASSETS_DIR is required}"
cmd="${ROBOCUP_OFFICIAL_START_COMMAND:?ROBOCUP_OFFICIAL_START_COMMAND is required}"

[[ -d "$assets" ]] || { log "official assets mount is unavailable: $assets"; exit 66; }
[[ -n "$(find "$assets" -mindepth 1 -maxdepth 1 -print -quit)" ]] || { log "official assets mount is empty"; exit 66; }
[[ "$cmd" != PENDING_* ]] || { log "official start command is still a placeholder"; exit 64; }
[[ -f "${ROBOCUP_ROS_SETUP:?ROBOCUP_ROS_SETUP is required}" ]] || { log "ROS setup missing"; exit 64; }

source "${ROBOCUP_ROS_SETUP}"
log "starting organizer-provided command"
exec bash -lc "$cmd"
