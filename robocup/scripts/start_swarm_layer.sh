#!/bin/bash
# 启动多机协同搜索层。
# 关键：先停掉 swarm_offboard_server —— 它发 setpoint_position，
# 与 swarm_agent 的 setpoint_velocity 冲突，同时跑会让飞机抖动失控。
# 关键词用拼接构造，避免调用本脚本的 SSH 命令行被自己的 pkill 匹配掉（踩过两次）。

KEY="swarm_""offboard_server"

for pid in $(pgrep -f "$KEY" 2>/dev/null); do
  cmd=$(tr "\0" " " < /proc/$pid/cmdline 2>/dev/null)
  case "$cmd" in
    python3*) kill -9 "$pid" 2>/dev/null; echo "已停止 offboard_server (pid $pid)" ;;
  esac
done
sleep 2
echo "剩余 offboard_server: $(pgrep -f "$KEY" 2>/dev/null | wc -l)  (应为 0)"

source /opt/ros/noetic/setup.bash
source /root/team_ws/robocup/devel/setup.bash
export ROBOCUP_WS=/root/team_ws/robocup
export PYTHONPATH=/root/team_ws/robocup/src/robocup_swarm/scripts:$PYTHONPATH

roslaunch robocup_swarm swarm.launch run_sim:=${1:-false}
