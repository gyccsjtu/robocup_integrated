#!/usr/bin/env bash
# Perception-failure injection acceptance (dynamic wall, scene_mode=perception).
# usage: run_depth_cut.sh <mode> [delay_s] [seed]
#   mode = cut_warn  | depth stream killed right after the mid-flight replan;
#          expect PERCEPTION_DEPTH_STALE (warn) and MISSION_COMPLETE anyway.
#   mode = cut_abort | same cut with action=abort; expect fail-safe landing
#          and RESULT reason=PERCEPTION_DEPTH_STALE.
#   mode = delay     | stream stays alive but every frame is delayed by
#          delay_s (sim stamps preserved); age == injected latency. delay
#          above the 5 s budget must fire the watchdog, below must not.
#   mode = never     | no camera at all (relay not started, no wall spawner);
#          expect PERCEPTION_DEPTH_STALE from MOVE begin, mission completes
#          on the metadata map (static single_wall is fully known).
set -Eeo pipefail

MODE="${1:?mode required: cut_warn|cut_abort|delay|never}"
DELAY_S="${2:-0}"
SEED="${3:-42}"
WS="${ROBOCUP_WORKSPACE:-$HOME/robocup/robocup_ws}"
export ROBOCUP_WORKSPACE="$WS"
export PX4_SOURCE_DIR="$WS/third_party/PX4-Autopilot"
export XTD_CATKIN_WS="$WS/build/xtdrone_catkin_ws"
STATE="$WS/build/training_city"
CAT=single_wall

case "$MODE" in
    cut_warn)  ACTION=warn;  CUT=1; SPAWN=1; RELAY=1 ;;
    cut_abort) ACTION=abort; CUT=1; SPAWN=1; RELAY=1 ;;
    delay)     ACTION=warn;  CUT=0; SPAWN=1; RELAY=1 ;;
    never)     ACTION=warn;  CUT=0; SPAWN=0; RELAY=0 ;;
    *) echo "bad mode $MODE"; exit 2 ;;
esac
echo "=================== DEPTH-CUT E2E mode=$MODE delay=$DELAY_S seed=$SEED ==================="

# 1. stop any running stack
pkill gzserver 2>/dev/null || true
pkill gzclient 2>/dev/null || true
pkill mavros 2>/dev/null || true
pkill rosmaster 2>/dev/null || true
pkill -f "roslaunch robocup_trainin[g]" 2>/dev/null || true
pkill -f "px4 $HOME/robocu[p]" 2>/dev/null || true
pkill -f "uav_navigation_nod[e]" 2>/dev/null || true
pkill -f "dynamic_block_spawne[r]" 2>/dev/null || true
pkill -f "depth_rela[y]" 2>/dev/null || true
pkill -f "depth_cut_watcher" 2>/dev/null || true
sleep 3

# 2. world + metadata + spawn.env
bash "$WS/scripts/container/generate_training_city.sh" \
    --preset unit --category "$CAT" --seed "$SEED" >/dev/null

# 3. endpoints + base config
PREP=$(python3 "$WS/scripts/vm/vm_scenario_tools.py" prepare "$CAT" "$SEED" "$WS")
echo "$PREP"
START_XY=$(printf '%s' "$PREP" | sed -n 's/^START=\([-0-9.]*\),\([-0-9.]*\).*/\1,\2/p')
GOAL_XY=$(printf '%s' "$PREP" | sed -n 's/.*[ ,]GOAL=\([-0-9.]*\),\([-0-9.]*\).*/\1,\2/p')

# 4. perception config through the RELAY topic + watchdog settings
python3 - "$STATE/scenario_$CAT.yaml" "$STATE/scenario_dynamic_block.yaml" \
    "$ACTION" "$DELAY_S" <<'PYEOF'
import sys
src, dst, action, delay = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
text = open(src).read()
text += """
# --- depth-cut acceptance (scene_mode=perception via relay) ---
scene_mode: perception
depth_topic: /camera_front/depth/relay
perception_grid_resolution_m: 0.25
perception_grid_side_m: 12.0
perception_z_min_m: 0.8
perception_z_max_m: 3.5
perception_update_hz: 5.0
perception_check_hz: 2.0
perception_lookahead_m: 5.0
perception_corridor_half_m: 0.5
perception_trigger_frames: 3
# Runtime clearance contract: vehicle_radius 0.40 + horizontal_margin 0.20.
# Inflation below 0.6 lets A* hug obstacles closer than the airframe can pass.
perception_inflation_m: 0.6
perception_depth_max_age_s: 5.0
perception_depth_stale_action: %s
""" % action
if delay != "0":
    text += "perception_delay_note: frames relayed with %ss wall delay\n" % delay
open(dst, "w").write(text)
print("PERCEPTION_CONFIG=" + dst)
PYEOF

# 5. sim stack with the depth camera
export DISPLAY=:0 ROBOCUP_GAZEBO_GUI=false
export ROBOCUP_LAUNCH=training_city_single_uav_depth.launch
nohup bash "$WS/scripts/vm/start_training_city_native.sh" > "$HOME/sim_depth_cut.log" 2>&1 &

# 6. MAVROS + param sync
source /opt/ros/noetic/setup.bash
CONNECTED=""
for _ in $(seq 1 45); do
    sleep 2
    CONNECTED=$(timeout 4 rostopic echo -n1 /mavros/state 2>/dev/null | grep connected || true)
    if echo "$CONNECTED" | grep -q "True"; then break; fi
done
if ! echo "$CONNECTED" | grep -q "True"; then
    echo "OUTCOME=DEPTH_CUT ERROR=MAVROS_NEVER_CONNECTED"; exit 3
fi
sleep 2
PARAM_OK=""
for _ in $(seq 1 20); do
    V=$(timeout 5 rosrun mavros mavparam get COM_RCL_EXCEPT 2>/dev/null | tail -1 || true)
    if [ -n "$V" ]; then PARAM_OK="$V"; break; fi
    sleep 2
done
if [ -z "$PARAM_OK" ]; then
    echo "OUTCOME=DEPTH_CUT ERROR=PARAMS_NEVER_SYNCED"; exit 5
fi
echo "PARAM_SYNC=$PARAM_OK"

# 7. fresh receipt
python3 "$WS/scripts/vm/vm_scenario_tools.py" receipt "$WS" >/dev/null

# 8. relay — always started clean (no delay): the delay is injected
# mid-flight by the watcher in 10b so the wall is mapped with fresh frames
# BEFORE the latency spike.  The 'never' mode skips the relay entirely.
RELAY_PID=""
if [ "$RELAY" = "1" ]; then
    nohup python3 "$WS/scripts/vm/relay_depth.py" \
        /camera_front/depth/relay 0 > "$HOME/depth_relay.log" 2>&1 &
    RELAY_PID=$!
    echo "RELAY_STARTED pid=$RELAY_PID delay=0 (mid-flight injection: ${DELAY_S}s)"
fi

if [ "$SPAWN" = "1" ]; then
    nohup python3 "$WS/scripts/vm/spawn_dynamic_wall.py" \
        "$START_XY" "$GOAL_XY" 0.60 > "$HOME/depth_cut_spawn.log" 2>&1 &
fi

# 9. the node
source "$WS/scripts/container/single_uav_env.sh" 2>/dev/null || true
source "$WS/devel_isolated/setup.bash"
export ROS_PACKAGE_PATH="$WS/src:$WS/devel_isolated:${ROS_PACKAGE_PATH:-}"
LOGD="$WS/logs/scenario_matrix/$(date -u +%Y%m%dT%H%M%S.%NZ)_depthcut_$MODE"
mkdir -p "$LOGD"
nohup rosrun robocup_navigation uav_navigation_node.py \
    _config_file:="$STATE/scenario_dynamic_block.yaml" \
    _log_dir:="$LOGD" > "$HOME/nav_depth_cut.log" 2>&1 &
NODE_PID=$!

# 10. cut the relay once the mid-flight replan has happened
if [ "$CUT" = "1" ] && [ -n "$RELAY_PID" ]; then
    (
        for _ in $(seq 1 240); do
            if grep -q "PERCEPTION_REPLAN_STARTED" "$LOGD/events.jsonl" 2>/dev/null; then
                sleep 2
                kill "$RELAY_PID" 2>/dev/null || true
                echo "DEPTH_CUT_AT=$(date -u +%H:%M:%S)"
                break
            fi
            sleep 1
        done
    ) > "$HOME/depth_cut_marker.log" 2>&1 &
fi

# 10b. delay mode: once the wall is mapped and replanned, RESTART the relay
# with a constant delay — observation latency spikes mid-flight while the
# stream keeps flowing.  Stamps are preserved, so the node sees age == delay.
if [ "$MODE" = "delay" ] && [ -n "$RELAY_PID" ]; then
    (
        for _ in $(seq 1 240); do
            if grep -q "PERCEPTION_REPLAN_STARTED" "$LOGD/events.jsonl" 2>/dev/null; then
                sleep 3
                kill "$RELAY_PID" 2>/dev/null || true
                sleep 1
                nohup python3 "$WS/scripts/vm/relay_depth.py" \
                    /camera_front/depth/relay "$DELAY_S" > "$HOME/depth_relay2.log" 2>&1 &
                echo "DELAY_INJECTED_AT=$(date -u +%H:%M:%S) delay=${DELAY_S}s"
                break
            fi
            sleep 1
        done
    ) > "$HOME/depth_delay_marker.log" 2>&1 &
fi

# 11. wait for RESULT
RESULT=""
for _ in $(seq 1 210); do
    sleep 2
    RESULT=$(grep '"event": "RESULT"' "$LOGD/events.jsonl" 2>/dev/null | tail -1 || true)
    if [ -n "$RESULT" ]; then break; fi
    if ! kill -0 "$NODE_PID" 2>/dev/null && ! pgrep -f "uav_navigation_nod[e]" >/dev/null; then
        RESULT=$(grep '"event": "RESULT"' "$LOGD/events.jsonl" 2>/dev/null | tail -1 || true)
        break
    fi
done

echo "--- relay ---"; cat "$HOME/depth_relay.log" 2>/dev/null | tail -3 || true
echo "--- cut marker ---"; cat "$HOME/depth_cut_marker.log" 2>/dev/null || true
echo "--- delay marker ---"; cat "$HOME/depth_delay_marker.log" 2>/dev/null || true
echo "--- spawn report ---"; cat "$HOME/depth_cut_spawn.log" 2>/dev/null | tail -3 || true

if [ -z "$RESULT" ]; then
    echo "OUTCOME=DEPTH_CUT MODE=$MODE ERROR=NO_RESULT_TIMEOUT LOGDIR=$LOGD"
    exit 4
fi
REASON=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('reason',''))")
CODE=$(printf '%s' "$RESULT" | python3 -c "import sys,json; print(json.load(sys.stdin).get('code',''))")
echo "OUTCOME=DEPTH_CUT MODE=$MODE REASON=$REASON CODE=$CODE LOGDIR=$LOGD"

echo "--- event chain ---"
grep -E '"event": "(PERCEPTION_DEPTH_STALE|PERCEPTION_DEPTH_RECOVERED|PERCEPTION_REPLAN_STARTED|PERCEPTION_REPLAN_FAILED|GOAL_ACCEPTED|GOAL_PREEMPTED|FAILURE|RESULT)"' \
    "$LOGD/events.jsonl" 2>/dev/null | python3 -c "
import sys, json
for line in sys.stdin:
    try:
        row = json.loads(line)
    except ValueError:
        continue
    keys = ['event', 'age_s', 'action', 'lag_s', 'blocked_samples', 'map_version', 'detail', 'reason', 'code', 'elapsed_s']
    print({k: (round(row[k],1) if isinstance(row.get(k), float) else row[k]) for k in keys if k in row})
"
