#!/bin/bash
# ============================================================================
# 一轮完整测试的编排器（新增的用户侧脚本，不改动任何官方文件）
#
# 解决的问题：
#   1) 任务结束后整套空转 —— 本脚本在裁判结束/退出后自动 stop_all.sh 收摊
#   2) 日志/ulog 撑爆磁盘 —— 收摊时 --clean 归档清理
#   3) 没有发现时刻记录 —— mission_events.py 写 events_latest.jsonl，本脚本按轮归档
#
# 用法：
#   bash /root/run_full_round.sh                 # 基线轮（PRIORITY_CORNER=0）
#   PRIORITY_CORNER=1 bash /root/run_full_round.sh   # 开角落偏置
#   PRIORITY_MAX_UAV=2 PRIORITY_CORNER=1 bash /root/run_full_round.sh
#
# 可调：MISSION_WALL_TIMEOUT（墙钟硬超时，秒）
# ============================================================================
set +u

WS=/root/team_ws/robocup
PX4WS=$WS/third_party/PX4-Autopilot
BUILD=$PX4WS/build/px4_sitl_default

ROUND=$(date +%Y%m%d_%H%M%S)
RD=/root/round_logs/$ROUND
mkdir -p "$RD"

# --- A/B 开关：默认关（基线轮）---
export PRIORITY_CORNER=${PRIORITY_CORNER:-0}
export PRIORITY_MAX_UAV=${PRIORITY_MAX_UAV:-2}
export CLEAN_ULOG=${CLEAN_ULOG:-1}
MISSION_WALL_TIMEOUT=${MISSION_WALL_TIMEOUT:-2700}

# --- 高度护栏参数（2026-09-29 基线）---
export ALT_HARD_CEIL=${ALT_HARD_CEIL:-5.2}
export ALT_EMERG_CEIL=${ALT_EMERG_CEIL:-5.0}
export ALT_TARGET_CAP=${ALT_TARGET_CAP:-4.5}
export ALT_EMERG_DESCENT=${ALT_EMERG_DESCENT:--3.0}
export ALT_EMERG_HSCALE=${ALT_EMERG_HSCALE:-0.35}

# --- NBV 参数 ---
export VIS_ENABLE=${VIS_ENABLE:-1}
export W_VIS=${W_VIS:-1.2}
export VIS_GAIN_SAT=${VIS_GAIN_SAT:-12}
export VIS_COMMIT=${VIS_COMMIT:-1}

# --- 覆盖参数 ---
export GRID_SIZE_M=${GRID_SIZE_M:-7}
export W_NOVELTY=${W_NOVELTY:-1.2}

# --- 检测参数 ---
export FUSE_ENABLE=${FUSE_ENABLE:-1}
export COOP_ENABLE=${COOP_ENABLE:-1}
export COOP_FUSE=${COOP_FUSE:-0}

ts() { echo "[$(date '+%F %T')] $*"; }

ts "=== 轮次 $ROUND 开始 ==="
ts "PRIORITY_CORNER=$PRIORITY_CORNER PRIORITY_MAX_UAV=$PRIORITY_MAX_UAV"
df -h / | tail -1

# ---------- 0. 清场 ----------
ts "0/7 清场"
pkill -f safety_monitor.py 2>/dev/null
bash /root/stop_all.sh > "$RD/stop_all.log" 2>&1
  bash /root/prelaunch_cleanup.sh > "$RD/prelaunch_cleanup.log" 2>&1
sleep 3

# ---------- 0.5 按当前地图重生成栅格元数据 ----------
# 官方地图/black_box 是随机生成的，元数据若用上一轮的会导致 A* 按错地图算。
# gen_base_metadata.py 只读官方 black_box.txt + control_actor.py 常量，不改官方文件。
ts "0.5/7 重生成栅格元数据"
python3 /root/gen_base_metadata.py > "$RD/gen_meta.log" 2>&1 \
  && tail -3 "$RD/gen_meta.log" || ts "   !! 元数据生成失败，沿用旧文件"

# ---------- 1. Gazebo + PX4 + MAVROS ----------
ts "1/7 启动 Gazebo/PX4（世界=官方 base.world）"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
source /usr/share/gazebo/setup.sh
source "$PX4WS/Tools/setup_gazebo.bash" "$PX4WS" "$BUILD"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4WS/Tools/sitl_gazebo
export LD_LIBRARY_PATH=$BUILD/build_gazebo:$LD_LIBRARY_PATH
export ROBOCUP_WS=$WS
# 栅格元数据：默认是 training_city_full_s7.json（另一张图），必须指向官方 base.world 这份
export ROBOCUP_METADATA=${ROBOCUP_METADATA:-/root/team_ws/robocup/src/robocup_training_worlds/worlds/generated/robocup_base.json}
rm -rf /tmp/px4-* 2>/dev/null
mkdir -p "$BUILD/tmp/rootfs"

# 官方 actor 插件 libros_actor_cmd_pose_plugin.so 装在 /root/actor_ws/devel/lib，
# 而默认 GAZEBO_PLUGIN_PATH 只含 PX4 的 build_gazebo，插件加载不到。后果：
#   a) world 里 actor 的 <init_pose> 不生效，6 个 actor 全落在默认原点；
#   b) /actor_<i>/cmd_motion 无人订阅，官方 control_actor.py 的指令石沉大海 → actor 不动。
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/root/actor_ws/devel/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/actor_ws/devel/lib
setsid nohup roslaunch robocup_swarm multi_uav_sitl.launch gui:=false vehicle:=iris \
  world:=/root/XTDrone/robocup/base.world \
  iris_sdf:=$PX4WS/Tools/sitl_gazebo/models/iris_2d_lidar/iris_2d_lidar.sdf \
  > /root/sim_go.log 2>&1 < /dev/null &

# 等仿真就绪
# 【2026-09-27 加固】判断依据必须是「真实起飞的 PX4 实例数」：
#   FLEET_SOCK = /tmp/px4-sock-* 的个数，活着的 PX4 实例才会建这个 socket
#   P          = px4pkg/px4 进程数
# 之前用 pgrep -f "px4_sitl_default/bin/px4" 计数不稳定，曾出现 px4=4 也放行，
# 结果 iris_3/iris_5 全程趴地、uav_3/uav_5 一次派位都没有。
count_fleet() {
  FLEET_SOCK=$(ls /tmp/px4-sock-* 2>/dev/null | wc -l)
  G=$(pgrep -x gzserver | wc -l)
  P=$(ps -eo args 2>/dev/null | grep -c "[p]x4pkg/px4")
  M=$(pgrep -f mavros_node | wc -l)
}
READY=0
for attempt in 1 2; do
  for i in $(seq 1 42); do
    count_fleet
    # 主判据是 FLEET_SOCK（活着的 PX4 实例才会建 /tmp/px4-sock-N）；
    # 进程数 P 只用于日志 —— pgrep -f "px4pkg/px4" 在这里会漏匹配，
    # 曾出现 gzserver=2 px4=0 实例=6 却把正常的仿真杀掉重起。
    if [ "$G" -ge 1 ] && [ "$FLEET_SOCK" -ge 6 ] && [ "$M" -ge 6 ]; then
      ts "   仿真就绪 (${i}0s): gzserver=$G px4=$P 实例=$FLEET_SOCK mavros=$M"; READY=1; break 2
    fi
    sleep 10
  done
  if [ "$READY" != "1" ] && [ "$attempt" = "1" ]; then
    ts "   !! 第 1 次未就绪: gzserver=$G px4=$P 实例=$FLEET_SOCK mavros=$M —— 清 PX4 实例锁后重起仿真"
    pkill -f roslaunch; sleep 3
    pkill -9 -f gzserver; pkill -9 -f "px4pkg/px4"; pkill -9 -f mavros_node; sleep 3
    rm -f /tmp/px4_lock-* /tmp/px4-sock-* /tmp/.px4_instance_*_cleaned
    setsid nohup bash /root/run_swarm_6uav.sh > /root/sim_go.log 2>&1 < /dev/null &
  fi
done
if [ "$READY" != "1" ]; then
  ts "   !! 重启一次仍未满 6 架: gzserver=$G px4=$P 实例=$FLEET_SOCK mavros=$M"
  ts "   !! 本轮数据不可用于横向对比（缺机），但继续跑以便看日志"
fi

# ---------- 2. 官方链路（bridges / actor / 裁判 / 事件记录 / 守卫）----------
ts "2/7 启动官方链路 start_official_robocup.sh"
bash /root/start_official_robocup.sh > "$RD/start_official.log" 2>&1
tail -5 "$RD/start_official.log"
JUDGE=$(pgrep -f score_cal.py | wc -l)
ts "   裁判=$JUDGE  actor=$(pgrep -f control_actor.py|wc -l)"

# ---------- 3. 真值桥接 gazebo -> /swarm/target_states ----------
ts "3/7 official_target_bridge"
setsid nohup python3 -u /root/official_target_bridge.py > /root/otb.log 2>&1 < /dev/null &
sleep 3
ts "   otb=$(pgrep -f official_target_bridge.py|wc -l)"

# ---------- 4. 协同层（拍卖分配 + 6 个 agent）----------
ts "4/7 协同层 swarm.launch"
# start_swarm_layer.sh 里的 roslaunch 是前台阻塞的，必须放后台，
# 否则编排器永远走不到第 5/6/7 步（detection_to_official 不启 -> 官方收不到检测）
setsid nohup bash /root/start_swarm_layer.sh > "$RD/swarm_layer.log" 2>&1 < /dev/null &
for i in $(seq 1 40); do
  A=$(pgrep -f swarm_agent | wc -l); M=$(pgrep -f swarm_manager | wc -l)
  [ "$A" -ge 6 ] && [ "$M" -ge 1 ] && break
  sleep 2
done
ts "   manager=$(pgrep -f swarm_manager|wc -l) agent=$(pgrep -f swarm_agent|wc -l)"

# ---------- 5. 团队检测 -> 官方检测话题 ----------
ts "5/7 detection_to_official"
# ros_actor_cmd_pose_plugin_msgs 在 actor_ws 里，必须显式加进 PYTHONPATH，
# 否则 "No module named 'ros_actor_cmd_pose_plugin_msgs'" 直接退出
setsid nohup bash -c '
source /opt/ros/noetic/setup.bash
source /root/team_ws/robocup/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/root/actor_ws/devel/lib/python3/dist-packages:/root/team_ws/robocup/src/robocup_swarm/scripts:$PYTHONPATH
exec python3 -u /root/detection_to_official.py
' > /root/d2o.log 2>&1 < /dev/null &
sleep 3
ts "   d2o=$(pgrep -f detection_to_official.py|wc -l)"

# ---------- 5.5 安全监测（新计分规则下的两条硬约束）----------
#   a) 官方 score_cal.py:314  世界坐标 z > 6.0m  -> score = 0 整轮归零
#   b) 新规则 无人机碰撞次数 × 30 分 -> 必须量机间最小水平距离
ts "5.5/7 安全监测器"
setsid nohup bash -c '
source /opt/ros/noetic/setup.bash
source /root/team_ws/robocup/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export PYTHONPATH=/root/actor_ws/devel/lib/python3/dist-packages:$PYTHONPATH
exec python3 -u /root/safety_monitor.py
' > /root/safety.log 2>&1 < /dev/null &
sleep 5
ts "   safety=$(pgrep -f safety_monitor.py|wc -l)"

# ---------- 6. 等任务结束 ----------
ts "6/7 等任务结束（墙钟上限 ${MISSION_WALL_TIMEOUT}s）"
T0=$(date +%s)
ENDED=""
while true; do
  if grep -qa "Mission finished\|mission failed\|Time out" /root/score_cal.log 2>/dev/null; then
    ENDED="裁判判定结束"; break; fi
  if ! pgrep -f score_cal.py > /dev/null; then
    ENDED="裁判进程已退出"; break; fi
  if [ $(( $(date +%s) - T0 )) -gt "$MISSION_WALL_TIMEOUT" ]; then
    ENDED="墙钟硬超时"; break; fi
  sleep 10
done
ts "   结束原因: $ENDED （用时 $(( $(date +%s) - T0 ))s 墙钟）"
sleep 30   # 留时间让 events 节点写 summary

# ---------- 7. 归档 + 收摊 ----------
ts "7/7 归档与收摊"
cp /root/events_latest.jsonl  "$RD/" 2>/dev/null
cp /root/safety_latest.json /root/safety_series.jsonl /root/safety.log "$RD/" 2>/dev/null

cp /root/score_cal.log /root/guard.log /root/watchdog.log /root/mission_events.log \
   /root/bridge.log /root/gt_bridge.log /root/otb.log /root/d2o.log /root/sim_go.log \
   "$RD/" 2>/dev/null
{
  echo "round=$ROUND"
  echo "PRIORITY_CORNER=$PRIORITY_CORNER"
  echo "PRIORITY_MAX_UAV=$PRIORITY_MAX_UAV"
  echo "SEED_TRUTH=${SEED_TRUTH:-1}"
  echo "MAP_BOUNDS_FROM_META=${MAP_BOUNDS_FROM_META:-1}"
  echo "TRUTH_NOISE=${TRUTH_NOISE:-0}"
  echo "TRUTH_DELAY=${TRUTH_DELAY:-0}"
  echo "JUDGE_UAV_TYPE=${JUDGE_UAV_TYPE:-iris}"
  echo "end_reason=$ENDED"
  echo "wall_elapsed=$(( $(date +%s) - T0 ))"
  echo "--- score_cal.log 尾部 ---"
  tail -30 /root/score_cal.log 2>/dev/null
  echo "--- events summary ---"
  grep '"summary"' /root/events_latest.jsonl 2>/dev/null | tail -2
} > "$RD/meta.txt" 2>&1

bash /root/stop_all.sh --clean > "$RD/stop_all_final.log" 2>&1
ts "=== 轮次 $ROUND 结束 ==="
df -h / | tail -1
