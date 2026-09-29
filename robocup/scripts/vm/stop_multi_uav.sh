#!/usr/bin/env bash
# Stop the whole six-UAV fleet (sim + all namespaced navigation nodes).
set -uo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/multi_uav_common.sh"

mu_log "stopping fleet"
mu_kill_all
sleep 2
if mu_any_alive; then
    mu_log "WARN: processes still alive:"
    pgrep -af "uav_navigation_nod[e]|multi_uav_mavros_sit[l]|gzserve[r]|px[4]" || true
    exit 1
fi
mu_log "all fleet processes stopped"
