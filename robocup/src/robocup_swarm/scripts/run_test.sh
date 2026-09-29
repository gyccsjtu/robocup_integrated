#!/bin/bash
# RoboCup 仿真测试启动脚本
# 用法: ./run_test.sh [参数]

# 设置环境变量
export ROBOCUP_WS=${ROBOCUP_WS:-/home/ros/rcws}
export PYTHONUNBUFFERED=1

cd ~/team_ws/robocup/src/robocup_swarm/scripts

# 使用 python3（VM 上没有 python 命令）
python3 "$@"
