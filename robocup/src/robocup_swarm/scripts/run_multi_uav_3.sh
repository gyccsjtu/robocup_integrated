#!/bin/bash
# 3 机 SITL 启动 + 自检（供 team_bench.py 多轮统计用）
#
# 与 2 机版（run_multi_uav_2.sh）的差异：
#   1. 自检 mavlink 端口 4560/4561/4562 是否都在**监听**
#      —— 上一轮踩过的坑：LD_LIBRARY_PATH 没设好会导致 mavlink_interface
#      插件加载失败，PX4 起来了但端口不监听，表现为 MAVROS connected=False。
#      这个自检能一眼看出，不用再去翻 PD 日志。
#   2. CPU 绑核按 3 机分配。注意 4 核跑 3×px4 + gzserver 是**偏紧**的：
#      实测 2 机时 CPU 争抢就会让 EKF 高度发散（-7~-156m 坏值）。3 机风险更高。
#      若出现 EKF 发散，优先 gui:=false（省掉 gzclient 的 CPU），再考虑加核。
set -o pipefail

WS=${ROBOCUP_WS:-$HOME/team_ws/robocup}
GUI=${GUI:-true}

echo '>>> 清理残留进程...'
pkill -9 -f 'roslaunch px4 mavros_posix_sitl' 2>/dev/null || true
pkill -9 -f 'px4_sitl_default/bin/px4' 2>/dev/null || true
pkill -9 -f 'gzserver' 2>/dev/null || true
pkill -9 -f 'gzclient' 2>/dev/null || true
pkill -9 -f 'mavros_node' 2>/dev/null || true
pkill -9 -f 'rosmaster' 2>/dev/null || true
sleep 2
rm -rf /tmp/px4-* 2>/dev/null || true
rm -rf "$WS/third_party/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/instance"* 2>/dev/null || true
rm -rf "$WS/third_party/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/sitl_iris_"* 2>/dev/null || true
echo '>>> 清理完成'

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source "$WS/devel/setup.bash"
source /usr/share/gazebo/setup.sh
source "$WS/third_party/PX4-Autopilot/Tools/setup_gazebo.bash" \
       "$WS/third_party/PX4-Autopilot" \
       "$WS/third_party/PX4-Autopilot/build/px4_sitl_default"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$WS/third_party/PX4-Autopilot/Tools/sitl_gazebo"

# 关键：必须放在 setup_gazebo.bash **之后**。顺序错了会让 mavlink_interface
# 插件解析到错误的 libmav_msgs，报 undefined symbol Airspeed → 端口不监听。
export LD_LIBRARY_PATH="$WS/third_party/PX4-Autopilot/build/px4_sitl_default/build_gazebo:$LD_LIBRARY_PATH"

echo ">>> 启动 3 机 launch（gui=$GUI，后台）..."
LOG=$HOME/multi_uav_3.log
roslaunch robocup_swarm multi_uav_sitl.launch gui:=$GUI > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "$LAUNCH_PID" > ~/multi_uav_3.pid
echo ">>> launch PID=$LAUNCH_PID, 日志 $LOG"

echo '>>> 等待 Gazebo...'
for i in $(seq 1 60); do
  pgrep -x gzserver >/dev/null 2>&1 && { echo "Gazebo 起来了 (${i}x2s)"; break; }
  sleep 2
done

echo '>>> 等待 PX4 实例...'
sleep 12
PX4_COUNT=$(pgrep -fc 'px4_sitl_default/bin/px4' 2>/dev/null || echo 0)
echo ">>> PX4 进程数 = $PX4_COUNT (期望 3)"

# CPU 绑核：gzserver 独占 2,3；三机 px4 分到 0/1（px4_2 与 px4_1 共用 1）。
# 4 核是硬约束，共用核会有争抢；先这样跑，EKF 发散再降负载。
echo '>>> 绑定 CPU 核（gzserver→2,3 / px4_0→0 / px4_1→1 / px4_2→1）...'
sleep 3
GZPID=$(pgrep -f 'gzserver -e ode' | head -1)
PX4_0=$(pgrep -f 'px4.*-i 0' | head -1)
PX4_1=$(pgrep -f 'px4.*-i 1' | head -1)
PX4_2=$(pgrep -f 'px4.*-i 2' | head -1)
[ -n "$GZPID" ] && taskset -pc 2,3 "$GZPID" 2>/dev/null
[ -n "$PX4_0" ] && taskset -pc 0 "$PX4_0" 2>/dev/null
[ -n "$PX4_1" ] && taskset -pc 1 "$PX4_1" 2>/dev/null
[ -n "$PX4_2" ] && taskset -pc 1 "$PX4_2" 2>/dev/null
echo ">>> 绑核完成 gzserver=$GZPID px4_0=$PX4_0 px4_1=$PX4_1 px4_2=$PX4_2"

echo '>>> 等待 MAVROS 连接...'
sleep 12

# ---- 自检：端口在不在监听（上一轮的坑就卡在这里）----
echo '>>> mavlink 端口自检（4560/4561/4562 必须都在监听）...'
MISSING=0
for p in 4560 4561 4562; do
  if timeout 5 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/$p" 2>/dev/null; then
    echo "    端口 $p  ✓ 监听中"
  else
    echo "    端口 $p  ✗ 未监听"
    MISSING=$((MISSING+1))
  fi
done
if [ "$MISSING" -gt 0 ]; then
  echo ''
  echo '!!! 有端口未监听。最常见原因：LD_LIBRARY_PATH 顺序错 → mavlink_interface'
  echo '    插件加载失败（报 undefined symbol ... Airspeed）。检查：'
  echo "      grep -i 'undefined symbol\|Failed to load' $LOG | head"
fi

echo '>>> MAVROS state topic 检查...'
timeout 10 rostopic list 2>/dev/null | grep -E 'uav_[123]/mavros/state' || echo '(尚未出现)'

echo '>>> 连接状态...'
for i in 1 2 3; do
  S=$(timeout 5 rostopic echo -n1 /uav_$i/mavros/state 2>/dev/null \
      | grep -E 'connected|mode|armed' | tr '\n' ' ')
  echo "    uav_$i: ${S:-（无数据）}"
done

echo ''
echo '>>> 就绪后运行多轮统计：'
echo '    ROS_NAMESPACE= rosrun robocup_swarm team_bench.py _rounds:=10 _round_time:=45 _hide:=ideal'
echo ">>> 日志尾部:"
tail -20 "$LOG"
