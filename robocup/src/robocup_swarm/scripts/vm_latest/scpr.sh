#!/usr/bin/env bash
# 带重试的 scp：区域云 sshd 会随机 "Connection closed"，scp 掉一次就得重来
# 用法: bash scpr.sh <本地文件> <远程路径>
LOCAL="$1"
REMOTE="$2"
for i in $(seq 1 10); do
  out=$(scp -P 25316 -o StrictHostKeyChecking=no -o ConnectTimeout=15 \
        -o ServerAliveInterval=10 "$LOCAL" root@region-42.seetacloud.com:"$REMOTE" 2>&1)
  if [ $? -eq 0 ] && ! echo "$out" | grep -q "Connection closed"; then
    echo "SCP_OK ($i)"
    exit 0
  fi
  sleep 4
done
echo "SCP_FAILED"
echo "$out"
exit 1
