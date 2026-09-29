#!/usr/bin/env bash
# Boundary case: /clock stall mid-flight (empty map, seed 42).
# Pauses Gazebo physics for PAUSE_S wall seconds; the node must raise
# CLOCK_STALLED within clock_timeout_s and fail-safe land; after unpause
# the landing completes.  Verdict: RESULT reason=CLOCK_STALLED + LANDED.
# usage: pause_clock.sh [pause_s] [seed]
set -Eeo pipefail
PAUSE_S="${1:-8}"
SEED="${2:-42}"
WS="${ROBOCUP_WORKSPACE:-$HOME/robocup/robocup_ws}"
export ROBOCUP_WORKSPACE="$WS"
export PX4_SOURCE_DIR="$WS/third_party/PX4-Autopilot"
export XTD_CATKIN_WS="$WS/build/xtdrone_catkin_ws"
STATE="$WS/build/training_city"
CAT=empty

echo "=================== CLOCK PAUSE E2E pause=${PAUSE_S}s seed=$SEED ==================="

pkill gzserver 2>/dev/null || true
pkill gzclient 2>/dev/null || true
pkill mavros 2>/dev/null || true
pkill rosmaster 2>/dev/null || true
pkill -f "roslaunch robocup_trainin[g]" 2>/dev/null || true
pkill -f "px4 $HOME/robocu[p]" 2>/dev/null || true
pkill -f "uav_navigation_nod[e]" 2>/dev/null || true
pkill -f "clock_pause_watcher" 2>/dev/null || true
sleep 3

bash "$WS/scripts/container/generate_training_city.sh" \
    --preset unit --category "$CAT" --seed "$SEED" >/dev/null
python3 "$WS/scripts/vm/vm_scenario_tools.py" prepare "$CAT" "$SEED" "$WS"

export DISPLAY=:0 ROBOCUP_GAZEBO_GUI=false
unset ROBOCUP_LAUNCH
nohup bash "$WS/scripts/vm/start_training_city_native.sh" > "$HOME/sim_pause.log" 2>&1 &

source /opt/ros/noetic/setup.bash
CONNECTED=""
for _ in $(seq 1 45); do
    sleep 2
    CONNECTED=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep connected || true)
    if echo "$CONNECTED" | grep -q "True"; then break; fi
done
if ! echo "$CONNECTED" | grep -q "True"; then
    echo "OUTCOME=CLOCK_PAUSE ERROR=MAVROS_NEVER_CONNECTED"; exit 3
fi
sleep 2
PARAM_OK=""
for _ in $(seq 1 20); do
    V=$(timeout 5 rosrun mavros mavparam get COM_RCL_EXCEPT 2>/dev/null | tail -1 || true)
    if [ -n "$V" ]; then PARAM_OK="$V"; break; fi
    sleep 2
done
if [ -z "$PARAM_OK" ]; then
    echo "OUTCOME=CLOCK_PAUSE ERROR=PARAMS_NEVER_SYNCED"; exit 5
fi
echo "PARAM_SYNC=$PARAM_OK"

python3 "$WS/scripts/vm/vm_scenario_tools.py" receipt "$WS" >/dev/null

nohup python3 "$WS/scripts/vm/pause_clock.py" "$PAUSE_S" > "$HOME/clock_pause.log" 2>&1 &
WATCHER=$!

source "$WS/scripts/container/single_uav_env.sh" 2>/dev/null || true
source "$WS/devel_isolated/setup.bash"
export ROS_PACKAGE_PATH="$WS/src:$WS/devel_isolated:${ROS_PACKAGE_PATH:-}"
LOGD="$WS/logs/scenario_matrix/$(date -u +%Y%m%dT%H%M%S.%NZ)_clockpause"
mkdir -p "$LOGD"
nohup rosrun robocup_navigation uav_navigation_node.py \
    _config_file:="$STATE/scenario_$CAT.yaml" \
    _log_dir:="$LOGD" > "$HOME/nav_pause.log" 2>&1 &
NODE_PID=$!

RESULT=""
for _ in $(seq 1 240); do
    sleep 2
    RESULT=$(grep '"event": "RESULT"' "$LOGD/events.jsonl" 2>/dev/null | tail -1 || true)
    if [ -n "$RESULT" ]; then break; fi
    if ! kill -0 "$NODE_PID" 2>/dev/null && ! pgrep -f "uav_navigation_nod[e]" >/dev/null; then
        RESULT=$(grep '"event": "RESULT"' "$LOGD/events.jsonl" 2>/dev/null | tail -1 || true)
        break
    fi
done
kill $WATCHER 2>/dev/null || true

echo "--- pause watcher ---"; cat "$HOME/clock_pause.log" 2>/dev/null || true
if [ -z "$RESULT" ]; then
    echo "OUTCOME=CLOCK_PAUSE ERROR=NO_RESULT_TIMEOUT LOGDIR=$LOGD"; exit 4
fi
REASON=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('reason',''))")
CODE=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('code',''))")
echo "OUTCOME=CLOCK_PAUSE REASON=$REASON CODE=$CODE LOGDIR=$LOGD"

echo "--- event chain ---"
grep -E '"event": "(FAILURE|LANDED|LANDING_START|RESULT)"' \
    "$LOGD/events.jsonl" 2>/dev/null | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        row = json.loads(line)
    except ValueError:
        continue
    keys = ['event', 'detail', 'reason', 'code', 'elapsed_s']
    print({k: (round(row[k],1) if isinstance(row.get(k), float) else row[k]) for k in keys if k in row})
"
