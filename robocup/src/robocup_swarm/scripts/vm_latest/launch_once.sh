#!/usr/bin/env bash
# 带 flock 的单实例启动器：防止 sshr.sh 重试导致同一批次被启动多份
# （实测：重复启动 → ROS "new node registered with same name" → 整轮双双 shutdown）
LOCK=/tmp/batch13.lock
exec 9>"$LOCK"
flock -n 9 || { echo "ALREADY_RUNNING"; exit 0; }
bash /root/batch13_nbv.sh > /root/batch13_nbv.log 2>&1
echo "BATCH13_FINISHED"
