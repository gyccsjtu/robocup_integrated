#!/bin/bash
# 停止集群协同搜索节点（agent / manager / target_sim）。
# 用法： bash stop_swarm.sh [日志目录（可选，用于精确 kill PID）]
set -o pipefail

if [ -n "${1:-}" ] && [ -d "$1" ]; then
    echo ">>> 按 PID 文件停止：$1"
    for f in "$1"/*.pid; do
        [ -f "$f" ] || continue
        pid=$(cat "$f" 2>/dev/null)
        [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null && echo "  killed $(basename "$f" .pid) ($pid)"
    done
else
    echo ">>> 按进程名停止"
    pkill -9 -f swarm_agent 2>/dev/null || true
    pkill -9 -f swarm_manager 2>/dev/null || true
    pkill -9 -f target_sim_node 2>/dev/null || true
fi

sleep 1
n=$(ps -ef | grep -E 'swarm_agent|swarm_manager|target_sim_node' | grep -v grep | wc -l)
echo ">>> 剩余相关进程: $n"
