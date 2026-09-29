#!/usr/bin/env bash
set -Eeo pipefail
source /workspace/scripts/container/single_uav_env.sh
export PYTHONPATH="/workspace/src/robocup_navigation/src:${PYTHONPATH:-}"
exec timeout 35 python3 /workspace/scripts/container/single_wall_tools.py "$@"
