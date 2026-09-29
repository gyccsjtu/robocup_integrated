#!/usr/bin/env bash
set -Eeo pipefail
source /workspace/scripts/container/single_uav_env.sh
export PYTHONPATH="/workspace/src/robocup_navigation/src:${PYTHONPATH:-}"
for suite in test_astar_planner.py test_route_safety.py test_route_runtime.py test_uav_motion.py; do
  timeout 60 python3 -m unittest discover -s /workspace/tests -p "$suite" -v
done
python3 -m py_compile /workspace/scripts/container/single_wall_tools.py /workspace/tests/sitl_single_wall.py
