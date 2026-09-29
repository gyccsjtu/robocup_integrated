#!/usr/bin/env bash
# 清掉被重复启动的批次（sshr.sh 重试导致同一脚本启动了 5+ 份）
# 注意：用 ps+awk 取 PID 逐个 kill，避免 pkill -f 误杀调用它的 ssh 会话
pids=$(ps -eo pid,args | awk '/batch13_nbv/ || /run_full_round/ {if ($0 !~ /awk/) print $1}')
echo "KILL LIST: $pids"
for p in $pids; do
  kill -9 "$p" 2>/dev/null
done
sleep 2
bash /root/stop_all.sh > /tmp/kill_dup_stop.log 2>&1
sleep 2
rm -f /tmp/batch13.lock
rm -f /root/batch13_summary.tsv
echo "REMAIN: $(ps -eo pid,args | awk '/batch13_nbv/ || /run_full_round/ {if ($0 !~ /awk/) print $1}' | wc -l)"
