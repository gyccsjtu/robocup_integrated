#!/bin/bash
# 集群协同搜索启动脚本（每机一个 agent + 一个 manager + 一个目标仿真）
#
# 依赖：SITL 已启动（Gazebo + PX4 + MAVROS 就绪）。
# 用法： bash run_swarm.sh [uav_id列表，默认 uav_1,uav_2]
#        bash run_swarm.sh uav_1,uav_2,uav_3,uav_4,uav_5,uav_6
#
# 启动：
#   1. target_sim_node —— 6 名恐怖分子行为仿真（规则 1/2/3/4）
#   2. 每机一个 swarm_agent —— A* 绕障 + 目标几何判定检测
#   3. 一个 swarm_manager —— 拍卖分配 + 目标确认/消除（规则 4/5）
#
# 日志写到 $ROBOCUP_WS/logs/swarm_search/<时间戳>/
#
# 注意：后台进程用 setsid + </dev/null 启动，避免 ssh 断开时被 SIGHUP 杀掉；
#       并设 PYTHONUNBUFFERED=1，否则 Python 日志会缓冲在内存里不落盘。
set -eo pipefail

WS="${ROBOCUP_WS:-$HOME/team_ws/robocup}"
IDS="${1:-uav_1,uav_2}"
IFS=',' read -ra UAV_ARR <<< "$IDS"

LOG_DIR="$WS/logs/swarm_search/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$LOG_DIR"

echo ">>> 集群搜索启动: 机队=${IDS}"
echo "    工作空间: $WS"
echo "    日志目录: $LOG_DIR"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export PYTHONUNBUFFERED=1

# ---- 1) 目标仿真（6 名恐怖分子）----
echo ">>> 启动 target_sim_node"
setsid nohup rosrun robocup_swarm target_sim_node.py _uav_ids:="${IDS}" \
    </dev/null > "$LOG_DIR/target_sim.log" 2>&1 &
echo $! > "$LOG_DIR/target_sim.pid"

# ---- 2) 每机一个 agent ----
for uid in "${UAV_ARR[@]}"; do
    model="iris_${uid##*_}"      # uav_3 -> iris_3
    echo ">>> 启动 agent: ${uid} (model=${model})"
    ROS_NAMESPACE="$uid" setsid nohup rosrun robocup_swarm swarm_agent.py \
        _uav_id:="$uid" _model_name:="$model" \
        </dev/null > "$LOG_DIR/${uid}_agent.log" 2>&1 &
    echo $! > "$LOG_DIR/${uid}_agent.pid"
done

# ---- 3) 管理器 ----
echo ">>> 启动 manager: ${IDS}"
setsid nohup rosrun robocup_swarm swarm_manager.py _uav_ids:="${IDS}" \
    </dev/null > "$LOG_DIR/manager.log" 2>&1 &
echo $! > "$LOG_DIR/manager.pid"

echo ">>> 全部启动完成"
echo "    停止： bash $(dirname "$0")/stop_swarm.sh $LOG_DIR"
