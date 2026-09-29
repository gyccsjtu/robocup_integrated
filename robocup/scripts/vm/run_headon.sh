#!/usr/bin/env bash
# Dual-UAV head-on corridor case: drone A (nav stack, perception mode) flies
# the empty-map line west->east; intruder drone B (kinematic gazebo model,
# no flight stack) launches from the goal side and flies head-on at the same
# speed class.  Expected: A's depth perception marks B, the corridor probe
# triggers a replan, A dodges and still completes the mission — or, at the
# very least, never loses control (bounded fail-safe, no endless pinning).
# usage: run_headon.sh [seed] [mode: headon|cross_north|cross_south] [speed]
set -Eeo pipefail
SEED="${1:-42}"
MODE="${2:-headon}"
SPEED="${3:-0.30}"
BLOG="$HOME/headon_b_traj_${MODE}_${SPEED}.log"
WS="${ROBOCUP_WORKSPACE:-$HOME/robocup/robocup_ws}"
export ROBOCUP_WORKSPACE="$WS"
export PX4_SOURCE_DIR="$WS/third_party/PX4-Autopilot"
export XTD_CATKIN_WS="$WS/build/xtdrone_catkin_ws"
STATE="$WS/build/training_city"
CAT=empty

echo "=================== HEADON E2E seed=$SEED mode=$MODE speed=$SPEED ==================="

pkill gzserver 2>/dev/null || true
pkill gzclient 2>/dev/null || true
pkill mavros 2>/dev/null || true
pkill rosmaster 2>/dev/null || true
pkill -f "roslaunch robocup_trainin[g]" 2>/dev/null || true
pkill -f "px4 $HOME/robocu[p]" 2>/dev/null || true
pkill -f "uav_navigation_nod[e]" 2>/dev/null || true
pkill -f "headon_intruder_[b]" 2>/dev/null || true
rm -f "$HOME/headon_b_traj.log" "$BLOG"
sleep 3

bash "$WS/scripts/container/generate_training_city.sh" \
    --preset unit --category "$CAT" --seed "$SEED" >/dev/null
PREP=$(python3 "$WS/scripts/vm/vm_scenario_tools.py" prepare "$CAT" "$SEED" "$WS")
echo "$PREP"
START_XY=$(printf '%s' "$PREP" | sed -n 's/^START=\([-0-9.]*\),\([-0-9.]*\).*/\1,\2/p')

# perception config on the empty world: same safety envelope as dynamic block
python3 - "$STATE/scenario_$CAT.yaml" "$STATE/scenario_dynamic_block.yaml" <<'PYEOF'
import sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src).read()
text += """
# --- head-on intruder acceptance (scene_mode=perception) ---
scene_mode: perception
perception_grid_resolution_m: 0.25
perception_grid_side_m: 12.0
perception_z_min_m: 0.8
perception_z_max_m: 3.5
perception_update_hz: 5.0
perception_check_hz: 2.0
perception_lookahead_m: 5.0
perception_corridor_half_m: 0.5
perception_trigger_frames: 3
perception_blocked_min_samples: 3
# Runtime clearance contract: vehicle_radius 0.40 + horizontal_margin 0.20.
perception_inflation_m: 0.6
perception_depth_max_age_s: 5.0
perception_depth_stale_action: warn
"""
open(dst, "w").write(text)
print("PERCEPTION_CONFIG=" + dst)
PYEOF

export DISPLAY=:0 ROBOCUP_GAZEBO_GUI=false
export ROBOCUP_LAUNCH=training_city_single_uav_depth.launch
nohup bash "$WS/scripts/vm/start_training_city_native.sh" > "$HOME/sim_headon.log" 2>&1 &

source /opt/ros/noetic/setup.bash
CONNECTED=""
for _ in $(seq 1 45); do
    sleep 2
    CONNECTED=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep connected || true)
    if echo "$CONNECTED" | grep -q "True"; then break; fi
done
if ! echo "$CONNECTED" | grep -q "True"; then
    echo "OUTCOME=HEADON ERROR=MAVROS_NEVER_CONNECTED"; exit 3
fi
sleep 2
PARAM_OK=""
for _ in $(seq 1 20); do
    V=$(timeout 5 rosrun mavros mavparam get COM_RCL_EXCEPT 2>/dev/null | tail -1 || true)
    if [ -n "$V" ]; then PARAM_OK="$V"; break; fi
    sleep 2
done
if [ -z "$PARAM_OK" ]; then
    echo "OUTCOME=HEADON ERROR=PARAMS_NEVER_SYNCED"; exit 5
fi
echo "PARAM_SYNC=$PARAM_OK"

python3 "$WS/scripts/vm/vm_scenario_tools.py" receipt "$WS" >/dev/null

# intruder B waits for A's cruise on its own
nohup python3 "$WS/scripts/vm/fly_headon_b.py" "${START_XY%%,*}" "$MODE" "$SPEED" "$BLOG" \
    > "$HOME/headon_b.log" 2>&1 &

source "$WS/scripts/container/single_uav_env.sh" 2>/dev/null || true
source "$WS/devel_isolated/setup.bash"
export ROS_PACKAGE_PATH="$WS/src:$WS/devel_isolated:${ROS_PACKAGE_PATH:-}"
LOGD="$WS/logs/scenario_matrix/$(date -u +%Y%m%dT%H%M%S.%NZ)_headon"
mkdir -p "$LOGD"
nohup rosrun robocup_navigation uav_navigation_node.py \
    _config_file:="$STATE/scenario_dynamic_block.yaml" \
    _log_dir:="$LOGD" > "$HOME/nav_headon.log" 2>&1 &
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

echo "--- intruder B ---"; cat "$HOME/headon_b.log" 2>/dev/null | tail -4 || true
if [ -z "$RESULT" ]; then
    echo "OUTCOME=HEADON ERROR=NO_RESULT_TIMEOUT LOGDIR=$LOGD"; exit 4
fi
REASON=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('reason',''))")
CODE=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('code',''))")
echo "OUTCOME=HEADON REASON=$REASON CODE=$CODE LOGDIR=$LOGD"

echo "--- event chain ---"
grep -E '"event": "(PERCEPTION_REPLAN_STARTED|PERCEPTION_REPLAN_FAILED|PERCEPTION_DEPTH_STALE|GOAL_ACCEPTED|GOAL_PREEMPTED|FAILURE|RESULT)"' \
    "$LOGD/events.jsonl" 2>/dev/null | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        row = json.loads(line)
    except ValueError:
        continue
    keys = ['event', 'blocked_samples', 'map_version', 'detail', 'reason', 'code', 'elapsed_s']
    print({k: (round(row[k],1) if isinstance(row.get(k), float) else row[k]) for k in keys if k in row})
"
