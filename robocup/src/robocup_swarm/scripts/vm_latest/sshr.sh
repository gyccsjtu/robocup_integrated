#!/usr/bin/env bash
# 带重试的 ssh：区域云 sshd 会随机 "Connection closed"，短命令也会掉
# 用法: bash sshr.sh '远程命令'
CMD="$1"
for i in $(seq 1 10); do
  out=$(ssh -o StrictHostKeyChecking=no -o ConnectTimeout=15 -o ServerAliveInterval=10 \
        -p 25316 root@region-42.seetacloud.com "$CMD" 2>&1)
  rc=$?
  if [ $rc -eq 0 ] && ! echo "$out" | grep -q "Connection closed"; then
    echo "$out" | grep -v "WARNING: connection is not using"
    exit 0
  fi
  sleep 4
done
echo "FAILED_AFTER_RETRIES"
echo "$out"
exit 1
