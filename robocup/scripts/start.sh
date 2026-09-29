#!/usr/bin/env bash
set -Eeuo pipefail
source "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)/common.sh"

assert_official_config
log 'starting the idle organizer container'
compose up --detach sim
log 'starting organizer-provided simulation command'
compose exec --detach sim bash /workspace/scripts/container/start_sim.sh
sleep 2
"$script_dir/health_check.sh"
