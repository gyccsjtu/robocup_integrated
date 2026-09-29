#!/usr/bin/env bash
# restore_baseline.sh —— 把 baseline/ 下已验证生效的代码一键还原到远程机
#   用途：换机 / 区域云重建 / 代码被改坏后，恢复到 2026-09-29 验证过的基线
#   用法：bash restore_baseline.sh <host> <port>         例：bash restore_baseline.sh region-42.seetacloud.com 25316
#   注意：还原后必须跑 py_compile 自检（脚本已内置），失败即中止
set -u
HOST="${1:?需要 host}"; PORT="${2:?需要 port}"
HERE="$(cd "$(dirname "$0")" && pwd)"
B="$HERE/baseline"
REMOTE_SCRIPTS=/root/team_ws/robocup/src/robocup_swarm/scripts
PX4_ROMFS=/root/team_ws/robocup/third_party/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes
PX4_BUILD=/root/team_ws/robocup/third_party/PX4-Autopilot/build/px4_sitl_default/etc/init.d/airframes

SSHO="ssh -o StrictHostKeyChecking=no -o ConnectTimeout=20 -p $PORT"
SCPO="scp -o StrictHostKeyChecking=no -o ConnectTimeout=20 -P $PORT"

put() { # put <本地文件> <远程路径>
  for try in 1 2 3; do
    $SCPO "$1" "root@$HOST:$2" >/dev/null 2>&1 && { echo "  OK   $2"; return 0; }
    sleep 2
  done
  echo "  FAIL $2"; return 1
}

echo "== 1/4 上传核心代码 =="
put "$B/swarm_agent.py"          "$REMOTE_SCRIPTS/swarm_agent.py"          || exit 1
put "$B/swarm_manager.py"        "$REMOTE_SCRIPTS/swarm_manager.py"        || exit 1
put "$B/swarm_task.py"           "$REMOTE_SCRIPTS/swarm_task.py"           || exit 1
put "$B/cooperative_tracker.py"  "$REMOTE_SCRIPTS/cooperative_tracker.py"  || exit 1
put "$B/swarm_viz.py"            "$REMOTE_SCRIPTS/swarm_viz.py"            || exit 1
put "$B/detection_to_official.py" /root/detection_to_official.py            || exit 1
put "$B/run_full_round.sh"       /root/run_full_round.sh                    || exit 1
put "$B/start_swarm_layer.sh"    /root/start_swarm_layer.sh                 || exit 1
put "$B/robocup_view.rviz"       /root/robocup_view.rviz                    || exit 1

echo "== 2/4 PX4 airframe（必须两份都写：SITL 读 build 副本）=="
put "$B/10016_3dr_iris" "$PX4_ROMFS/10016_3dr_iris" || exit 1
$SSHO root@$HOST "cp $PX4_ROMFS/10016_3dr_iris $PX4_BUILD/10016_3dr_iris 2>/dev/null && echo '  OK   build 副本已同步' || echo '  WARN build 副本不存在（首次需先编译）'"

echo "== 3/4 语法自检 =="
$SSHO root@$HOST "python3 -m py_compile \
  $REMOTE_SCRIPTS/swarm_agent.py $REMOTE_SCRIPTS/swarm_manager.py \
  $REMOTE_SCRIPTS/swarm_task.py $REMOTE_SCRIPTS/cooperative_tracker.py \
  $REMOTE_SCRIPTS/swarm_viz.py /root/detection_to_official.py \
  && echo '  ALL_COMPILE_OK' || { echo '  COMPILE_FAILED —— 已还原但代码有问题，别跑仿真'; exit 1; }" || exit 1

echo "== 4/4 基线特征校验（缺一项说明还原不完全）=="
$SSHO root@$HOST "
A=$REMOTE_SCRIPTS/swarm_agent.py; D=/root/detection_to_official.py
chk(){ c=\$(grep -c \"\$2\" \"\$1\"); [ \"\$c\" -gt 0 ] && echo \"  [x] \$3\" || echo \"  [ ] \$3  <-- 缺失\"; }
chk \$A 'ALT_EMERG_CEIL'      'E1 6m 二次保险'
chk \$A 'SEP_FAR_DIV'         'E2 ORCA 可解性(P0-1/P0-2)'
chk \$A 'ALT_GUARD'           'E3 护栏覆写打点(P1-2)'
chk \$A 'CRASH_DETECT'        'E4 坠机检测(P1-3)'
chk \$A 'ALT_TARGET_CAP'      'E5 目标高度可配'
chk \$D 'COOP_ENABLE'         'E6 coop 影子诊断接入'
chk $REMOTE_SCRIPTS/cooperative_tracker.py 'needs_backup' 'E7 CooperativeTracker 新版'
chk $REMOTE_SCRIPTS/swarm_manager.py '_build_visibility' 'E8 NBV 可见集'
grep -q \"ALT_HARD_CEIL', '5.2\" \$A && echo '  [x] E9 ALT_HARD_CEIL 默认 5.2 (P1-1)' || echo '  [ ] E9 ALT_HARD_CEIL 默认不是 5.2  <-- 极限环风险'
grep -q 'MPC_Z_VEL_MAX_DN 3.0' $PX4_BUILD/10016_3dr_iris 2>/dev/null && echo '  [x] E10 PX4 放开下降速度' || echo '  [ ] E10 PX4 下降速度未放开  <-- 快降会被裁成 1.0'
"
echo "== 还原完成 =="
