#!/bin/bash
# 在虚拟机上执行：把整合包落地到 catkin 工作区。
#
#   用法：  bash sync_to_vm.sh [目标工作区路径]
#   默认目标： $HOME/team_ws/robocup
#
# 行为：
#   1. 先把现有目录整体备份成 <目标>_bak_时间戳
#   2. 覆盖 src/ scripts/ config/ docs/ tests/ 等源码目录
#   3. 重建 catkin 需要的符号链接
#   4. 打印落地校验结果
#
# 注意：_extra/ 与 _docs/ 不会进入 ROS 包路径，只作为参考与工具留存。

set -u
TARGET="${1:-$HOME/team_ws/robocup}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=================================================="
echo " 整合包目录 : $PKG"
echo " 落地目标   : $TARGET"
echo " 当前用户   : $(whoami) @ $(hostname)"
echo "=================================================="
echo

# ---------- 1. 备份 ----------
# 只备份源码：third_party(13G) / build(79M) / devel 都不动，
# 它们要么可重建、要么本包不会触碰，全量备份会把磁盘撑爆。
if [ -d "$TARGET" ]; then
  BAK="${TARGET}_bak_$(date +%Y%m%d_%H%M%S)"
  echo "[1/4] 备份现有源码（排除 third_party/build/devel）..."
  mkdir -p "$BAK"
  tar cf - -C "$TARGET" \
      --exclude=./third_party --exclude=./build --exclude=./devel \
      --exclude=__pycache__ . 2>/dev/null | tar xf - -C "$BAK" 2>/dev/null
  echo "      -> $BAK  ($(du -sh "$BAK" 2>/dev/null | cut -f1))"
  echo "      (third_party/build/devel 原地保留，未备份、也未改动)"
else
  echo "[1/4] 目标不存在，跳过备份，将新建"
  mkdir -p "$TARGET"
fi

# ---------- 2. 覆盖源码 ----------
echo "[2/4] 覆盖源码 ..."
cp -a "$PKG/robocup/." "$TARGET/"
echo "      完成"

# ---------- 3. 重建符号链接 ----------
echo "[3/4] 重建符号链接 ..."
if [ -f "$PKG/_symlinks.txt" ]; then
  while IFS=$'\t' read -r name tgt; do
    [ -z "${name:-}" ] && continue
    rel="${name#robocup/}"
    dest="$TARGET/$rel"
    mkdir -p "$(dirname "$dest")" 2>/dev/null
    rm -f "$dest"
    ln -s "$tgt" "$dest" && echo "      $rel -> $tgt"
  done < "$PKG/_symlinks.txt"
else
  echo "      无 _symlinks.txt，跳过"
fi

# ---------- 4. 校验 ----------
echo "[4/4] 落地校验 ..."
ok=0; bad=0
for f in \
  src/robocup_swarm/scripts/swarm_agent.py \
  src/robocup_swarm/scripts/swarm_task.py \
  src/robocup_swarm/scripts/swarm_manager.py \
  src/robocup_swarm/scripts/route_planner.py \
  src/robocup_navigation/scripts/coordination_executor.py \
  src/robocup_swarm/launch/swarm.launch \
  ; do
  p="$TARGET/$f"
  if [ -f "$p" ]; then
    printf "      OK   %-56s %8d bytes\n" "$f" "$(stat -c%s "$p")"
    ok=$((ok+1))
  else
    printf "      MISS %-56s\n" "$f"
    bad=$((bad+1))
  fi
done

echo
echo "=================================================="
echo " 落地完成：OK=$ok  缺失=$bad"
echo
echo " 备份位置: ${BAK:-（无，目标原本不存在）}"
echo
echo " 参考件（未进 ROS 路径）:"
echo "   $PKG/_extra/los3d/swarm_heightfield.py   三维遮挡用的高度场"
echo "   $PKG/_extra/verify/                      验证脚本"
echo "   $PKG/_docs/                              本轮改动与复盘文档"
echo "   $PKG/_ops/                               运维脚本"
echo
echo " 下一步（若要在虚拟机编译）:"
echo "   cd ${TARGET}/.. && catkin_make"
echo "=================================================="
