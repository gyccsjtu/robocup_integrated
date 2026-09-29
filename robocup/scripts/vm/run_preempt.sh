#!/usr/bin/env bash
# Boundary case: goal preemption mid-flight (empty map, seed 42).
set -Eeo pipefail
WS="${ROBOCUP_WORKSPACE:-$HOME/robocup/robocup_ws}"
export ROBOCUP_WORKSPACE="$WS"

# 1. start the standard scenario cycle in the background
bash "$WS/scripts/vm/run_vm_scenario.sh" empty 42 > ~/preempt_scenario.log 2>&1 &
RUNNER=$!

# 2. start the goal injector (it waits for cruise on its own)
source /opt/ros/noetic/setup.bash
python3 "$WS/scripts/vm/inject_goal.py" > ~/preempt_inject.log 2>&1 &
INJ=$!

# 3. wait for the runner verdict (up to 360 s)
wait $RUNNER || true
kill $INJ 2>/dev/null || true

echo "--- injector ---"
cat ~/preempt_inject.log
echo "--- verdict ---"
grep '^OUTCOME=empty' ~/preempt_scenario.log || true
LOGDIR=$(grep '^OUTCOME=empty' ~/preempt_scenario.log | sed 's/.*LOGDIR=//')
echo "--- event chain ---"
grep -E '"event": "(REPLAN_STARTED|GOAL_ACCEPTED|GOAL_PREEMPTED|GOAL_REJECTED|ARRIVED|RESULT)"' \
    "$LOGDIR/events.jsonl" 2>/dev/null | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        row = json.loads(line)
    except ValueError:
        continue
    keys = ['event', 'goal_seq', 'map_version', 'detail', 'reason', 'code', 'elapsed_s']
    print({k: row[k] for k in keys if k in row})
"
