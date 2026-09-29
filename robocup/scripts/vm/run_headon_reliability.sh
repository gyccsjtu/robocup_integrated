#!/usr/bin/env bash
# Reliability battery (P1 stability): run the canonical 6-scenario set
# REPEATEDLY (seed 42 only -- the world-gen / endpoint substitution pipeline
# is seed-42-locked, so true multi-seed generalization is not available yet).
# 3 repeats x 6 scenarios = 18 runs. Each scenario self-cleans (pkill gzserver)
# so they can be chained. A scenario failure must NOT abort the battery -- we
# want all 18 outcomes. Writes a consolidated summary + a sentinel so a
# host-side watcher can detect completion across SSH reconnects.
set -uo pipefail

WS="${ROBOCUP_WORKSPACE:-$HOME/robocup/robocup_ws}"
LOGDIR="$WS/logs"
SUMMARY="$LOGDIR/reliability_summary.txt"
MARKER="$LOGDIR/battery_done.marker"
SEED=42
RUNS=3

: > "$SUMMARY"
rm -f "$MARKER"

echo "BATTERY_START $(date -u +%Y%m%dT%H%M%SZ) seed=$SEED runs=$RUNS" | tee -a "$SUMMARY"

run_one () {
    local label="$1"; shift
    echo "" | tee -a "$SUMMARY"
    echo "########## $label ##########" | tee -a "$SUMMARY"
    bash "$@" 2>&1 | tee -a "$SUMMARY"
    echo "########## $label DONE ##########" | tee -a "$SUMMARY"
}

for RUN in $(seq 1 "$RUNS"); do
    echo "" | tee -a "$SUMMARY"
    echo "===================== RUN $RUN (seed $SEED) =====================" | tee -a "$SUMMARY"
    run_one "R${RUN}-headon#1"    "$WS/scripts/vm/run_headon.sh" "$SEED" headon 0.30
    run_one "R${RUN}-headon#2"    "$WS/scripts/vm/run_headon.sh" "$SEED" headon 0.30
    run_one "R${RUN}-headon#3"    "$WS/scripts/vm/run_headon.sh" "$SEED" headon 0.30
    run_one "R${RUN}-cross_north" "$WS/scripts/vm/run_headon.sh" "$SEED" cross_north 0.30
    run_one "R${RUN}-cross_south" "$WS/scripts/vm/run_headon.sh" "$SEED" cross_south 0.30
    run_one "R${RUN}-dynamic_block" "$WS/scripts/vm/run_dynamic_block.sh" "$SEED"
done

echo "" | tee -a "$SUMMARY"
echo "BATTERY_DONE $(date -u +%Y%m%dT%H%M%SZ)" | tee -a "$SUMMARY"
PASS=$(grep -cE "^OUTCOME=.*REASON=MISSION_COMPLETE CODE=0" "$SUMMARY" || true)
FAIL=$(grep -cE "^OUTCOME=.*(REASON=(ROUTE_|.*FAIL)|ERROR=)" "$SUMMARY" || true)
echo "BATTERY_TOTAL=$(($PASS + $FAIL)) BATTERY_PASS=$PASS BATTERY_FAIL=$FAIL" | tee -a "$SUMMARY"
echo "BATTERY_DONE_RC=0" > "$MARKER"
