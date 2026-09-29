#!/usr/bin/env bash
# 等 Gazebo 的 mavlink TCP 端口进入 LISTEN，再拉起 PX4 实例。
#
# 用法（roslaunch launch-prefix）：
#   px4_delayed_start_multi.sh <gazebo_tcp_port> <px4 可执行> [px4 参数...]
#
# 为什么必须等：iris 模型由 spawn_model 异步生成，插件没起来时 PX4 的
# simulator start -c 连不上，会直接退出。等端口 LISTEN 是最可靠的判据。
# 容器里没有 nc/ss，直接读 /proc/net/tcp。
#
# 【2026-09-27 加】清理本实例的残留锁。
# 症状：iris_3/iris_5（实例 2/4）全程趴地，PX4 报
#   "PX4 daemon already running for instance 2 (Success)" 后 exit 255
# 原因：上一轮异常退出后 /tmp/px4_lock-N、/tmp/px4-sock-N 没清干净，
#       新实例误判「已有守护进程」直接退出 → 该机永远起不来。
# 对策：启动前先按实例号删掉这两个残留文件，并杀掉占用同一实例的旧 PX4。

PORT="$1"
shift

if [ -z "$PORT" ]; then
    echo "px4_delayed_start_multi.sh: 缺少端口参数" >&2
    exit 1
fi

# ---- 从 -i N / -w sitl_iris_N 推出本实例号 ----
INSTANCE=""
prev=""
for a in "$@"; do
    case "$prev" in
        -i) INSTANCE="$a" ;;
        -w) case "$a" in sitl_iris_*) INSTANCE="${a#sitl_iris_}" ;; esac ;;
    esac
    prev="$a"
done
[ -z "$INSTANCE" ] && INSTANCE="$((PORT - 4560))"

CLEANUP_DONE="/tmp/.px4_instance_${INSTANCE}_cleaned"
if [ ! -f "$CLEANUP_DONE" ]; then
    rm -f "/tmp/px4_lock-${INSTANCE}" "/tmp/px4-sock-${INSTANCE}"
    # 杀掉占用同一实例的旧 PX4（只匹配本实例的 -i/-w）
    for p in $(pgrep -f -- "-i ${INSTANCE} -w sitl_iris_${INSTANCE}" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        echo "px4_delayed_start_multi.sh: 清掉实例 ${INSTANCE} 的残留 PX4 pid=$p" >&2
        kill -9 "$p" 2>/dev/null
    done
    mkdir -p /tmp
    : > "$CLEANUP_DONE"
    sleep 1
fi

HEXPORT=$(printf '%04X' "$PORT")
DEADLINE=$(( $(date +%s) + 180 ))

while true; do
    if awk -v p=":$HEXPORT" '$2 ~ p && $4 == "0A" { found = 1 } END { exit !found }' \
           /proc/net/tcp 2>/dev/null; then
        break
    fi
    if [ "$(date +%s)" -gt "$DEADLINE" ]; then
        echo "px4_delayed_start_multi.sh: 等待端口 $PORT 超时(180s)，放弃" >&2
        exit 1
    fi
    sleep 0.5
done

# 端口 LISTEN 后插件还要一点时间初始化，多给 2 秒
sleep 2
echo "px4_delayed_start_multi.sh: 端口 $PORT 已就绪，启动 PX4 实例 ${INSTANCE}" >&2
exec "$@"
