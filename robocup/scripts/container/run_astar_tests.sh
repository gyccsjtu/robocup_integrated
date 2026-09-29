#!/usr/bin/env bash
set -Eeuo pipefail

export PYTHONPATH="/workspace/src/robocup_navigation/src:${PYTHONPATH:-}"
exec python3 /workspace/tests/test_astar_planner.py
