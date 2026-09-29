#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

require_docker
output_dir="$workspace_root/logs/host/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$output_dir"
log "collecting Docker logs into $output_dir"
compose logs --no-color --timestamps >"$output_dir/docker-compose.log"
compose exec --no-TTY sim bash -lc 'source "$ROBOCUP_ROS_SETUP" && { rosnode list; rostopic list; }' >"$output_dir/ros-graph.txt"
log 'log collection complete'
