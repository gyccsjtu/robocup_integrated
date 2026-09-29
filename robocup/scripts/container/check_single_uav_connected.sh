#!/usr/bin/env bash
# Kept in a file to avoid PowerShell 5.1 -> Docker -> Bash quote loss.
set -eo pipefail
source /workspace/scripts/container/single_uav_env.sh >/dev/null 2>&1
timeout 6s rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'
