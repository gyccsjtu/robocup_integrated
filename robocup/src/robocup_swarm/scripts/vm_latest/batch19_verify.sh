#!/usr/bin/env bash
# batch19: 验证基线（6 轮：3 NBV + 3 coop 影子诊断）
#   判据：max_z<5.1、over6=0、[ALT_GUARD] 日志出现、[coop] 诊断出现
LOCK=/tmp/batch19.lock
exec 9>"$LOCK"
flock -n 9 || { echo "ALREADY_RUNNING"; exit 0; }

source /opt/ros/noetic/setup.bash
[ -f /root/team_ws/robocup/devel/setup.bash ] && source /root/team_ws/robocup/devel/setup.bash
export PYTHONPATH=/root/team_ws/robocup/src/robocup_swarm/scripts:$PYTHONPATH

run_one() {
    local tag="$1"; shift
    local start=$(date +%s)
    echo "=== $(date +%H:%M:%S) START $tag ==="
    rm -f /root/round_*.log /root/events.jsonl
    bash /root/stop_all.sh >/dev/null 2>&1
    bash /root/prelaunch_cleanup.sh >/dev/null 2>&1
    rm -f /tmp/px4_lock-* /tmp/px4-sock-*
    sleep 3
    
    setsid nohup bash /root/run_full_round.sh > /root/round.log 2>&1 < /dev/null &
    
    # 等待 manager 起来
    for i in $(seq 1 60); do
        sleep 5
        rosnode list 2>/dev/null | grep -q swarm_manager && { echo "SWARM_UP@$i"; break; }
    done
    
    sleep 10
    
    # 等待结束（最多 600s）
    for i in $(seq 1 120); do
        sleep 5
        grep -q "结束原因\|Time usage\|600.0" /root/round.log 2>/dev/null && break
    done
    
    sleep 8
    bash /root/stop_all.sh >/dev/null 2>&1
    sleep 3
    
    # 收集结果
    local elapsed=$(($(date +%s) - start))
    local result=$(grep -E "Time usage|用时" /root/round.log | tail -1)
    local found=$(grep -c "find actor" /root/round.log 2>/dev/null || echo 0)
    local alt_guard=$(grep -c "\[ALT_GUARD\]" /root/round.log 2>/dev/null || echo 0)
    local crash=$(grep -c "\[CRASH\]" /root/round.log 2>/dev/null || echo 0)
    local coop=$(grep -c "\[coop\]" /root/round.log 2>/dev/null || echo 0)
    local max_z=$(grep "max_z" /root/round.log 2>/dev/null | tail -1 | sed 's/.*max_z//' | tr -d '= \n' || echo 0)
    local over6=$(grep -c "z>6" /root/round.log 2>/dev/null || echo 0)
    
    echo "RESULT $tag t=${elapsed}s found=$found alt_g=$alt_guard crash=$crash coop=$coop max_z=$max_z over6=$over6"
    echo -e "$tag\t$elapsed\t$found\t$alt_guard\t$crash\t$coop\t$max_z\t$over6" >> /root/batch19_summary.tsv
}

export -f run_one

echo "batch19 $(date +%Y%m%d_%H%M%S)" > /root/batch19_summary.tsv
echo -e "tag\telapsed\tfound\talt_guard\tcrash\tcoop\tmax_z\tover6" >> /root/batch19_summary.tsv

run_one nbv_v1 PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 W_VIS=1.2
run_one nbv_v2 PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 W_VIS=1.2
run_one nbv_v3 PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 W_VIS=1.2
run_one cp_a   PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 COOP_ENABLE=1 COOP_FUSE=0
run_one cp_b   PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 COOP_ENABLE=1 COOP_FUSE=0
run_one cp_c   PLAN_FALLBACK=1 BACKUP_ENABLE=0 VIS_ENABLE=1 COOP_ENABLE=1 COOP_FUSE=0

echo "=== batch19 DONE ==="
cat /root/batch19_summary.tsv
