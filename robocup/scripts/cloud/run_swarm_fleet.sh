#!/bin/bash
# 集群协同搜索启动脚本（适配 config/multi_uav/fleet.yaml 的命名与端口约定）
#
# 与 scripts/cloud/run_swarm.sh 的区别：本脚本**从 fleet.yaml 派生**每机的
# uav_id / model_name / 端口，与队友的接入层（gen_multi_uav_launch.py /
# mu_fleet.py）保持单一事实来源，避免两套命名打架。
#
# 依赖：SITL 已启动（可用 scripts/vm/start_multi_uav.sh 或
#       config/multi_uav/multi_uav_dev.launch）。
#
# 用法：
#   bash run_swarm_fleet.sh              # 全部 6 机
#   bash run_swarm_fleet.sh 2            # 只起前 2 机
set -eo pipefail

WS="${ROBOCUP_WS:-$HOME/team_ws/robocup}"
LIMIT="${1:-}"

cd "$WS"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export PYTHONUNBUFFERED=1

# ---- 从 fleet.yaml 派生每机参数（复用队友的 mu_fleet 模块）----
# 注意：队友的 mu_fleet.py 用 ROBOCUP_WORKSPACE 定位工作空间（默认
# ~/robocup/robocup_ws），必须显式传对，否则读不到 fleet.yaml。
FLEET_ARGS=$(ROBOCUP_WORKSPACE="$WS" MU_FLEET_YAML="$WS/config/multi_uav/fleet.yaml" \
python3 - "$LIMIT" << 'PYEOF'
import os, sys
ws = os.environ["ROBOCUP_WORKSPACE"]
sys.path.insert(0, os.path.join(ws, "scripts", "vm"))
import mu_fleet
doc = mu_fleet.load()
rows = mu_fleet.uavs(doc)
limit = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None
if limit:
    rows = rows[:int(limit)]
# 输出：uav_id:model_name 每行一个
for r in rows:
    print("%s:%s" % (r["ros_ns"].strip("/"), r["model_name"]))
PYEOF
) || { echo "!!! 无法从 fleet.yaml 派生机队参数（检查 scripts/vm/mu_fleet.py）"; exit 1; }

IDS=$(echo "$FLEET_ARGS" | cut -d: -f1 | paste -sd, -)
[ -n "$IDS" ] || { echo "!!! fleet.yaml 未派生出任何 UAV"; exit 1; }

LOG_DIR="$WS/logs/swarm_search/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$LOG_DIR"
echo ">>> 集群搜索启动（fleet.yaml 派生）"
echo "    机队: $IDS"
echo "    日志: $LOG_DIR"

# ---- 1) 目标仿真（6 名恐怖分子）----
echo ">>> 启动 target_sim_node"
setsid nohup rosrun robocup_swarm target_sim_node.py _uav_ids:="$IDS" \
    </dev/null > "$LOG_DIR/target_sim.log" 2>&1 &
echo $! > "$LOG_DIR/target_sim.pid"

# ---- 2) 每机一个 agent（model_name 来自 fleet.yaml）----
while IFS=: read -r uid model; do
    [ -n "$uid" ] || continue
    echo ">>> 启动 agent: $uid (model=$model)"
    ROS_NAMESPACE="$uid" setsid nohup rosrun robocup_swarm swarm_agent.py \
        _uav_id:="$uid" _model_name:="$model" \
        </dev/null > "$LOG_DIR/${uid}_agent.log" 2>&1 &
    echo $! > "$LOG_DIR/${uid}_agent.pid"
done <<< "$FLEET_ARGS"

# ---- 3) 管理器 ----
echo ">>> 启动 manager"
setsid nohup rosrun robocup_swarm swarm_manager.py _uav_ids:="$IDS" \
    </dev/null > "$LOG_DIR/manager.log" 2>&1 &
echo $! > "$LOG_DIR/manager.pid"

echo ">>> 全部启动完成"
echo "    停止： bash $WS/scripts/cloud/stop_swarm.sh $LOG_DIR"
