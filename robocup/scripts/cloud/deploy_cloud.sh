#!/bin/bash
# ==============================================================================
# RoboCup 2026 集群协同搜索 —— 云端一键部署脚本
#
# 目标环境：Ubuntu 20.04 LTS + ROS Noetic（与本地 VM 一致，零代码改动）
# 用法：
#   1. 上传本脚本到云服务器，然后：
#        bash deploy_cloud.sh                     # 全量安装（约 40~90 分钟）
#        bash deploy_cloud.sh --skip-px4-build    # 跳过 PX4 编译（已编译过）
#
# 部署内容：
#   - ROS Noetic + MAVROS + Gazebo 11
#   - XTDrone（协作感知/编队等，来自 gitee）
#   - PX4-Autopilot v1.13.2（SITL 编译）
#   - 本项目（git clone）+ catkin_make
#   - 无头模式依赖（xvfb）与 CPU 绑核脚本
#
# 设计原则：
#   - 幂等：可重复执行，已装的跳过
#   - 失败即停（set -e），关键步骤打印状态
#   - 路径用 ROBOCUP_WS 统一，不写死在代码里
# ==============================================================================
set -euo pipefail

# ------------------------------ 可配置项 ------------------------------
ROBOCUP_WS="${ROBOCUP_WS:-$HOME/team_ws/robocup}"
REPO_URL="${REPO_URL:-git@github.com:lfen9048-bot/robocup.git}"
REPO_URL_HTTPS="${REPO_URL_HTTPS:-https://github.com/lfen9048-bot/robocup.git}"
PX4_TAG="v1.13.2"
XTDRONE_URL="https://gitee.com/robin_shaun/XTDrone.git"
SKIP_PX4_BUILD=0
[[ "${1:-}" == "--skip-px4-build" ]] && SKIP_PX4_BUILD=1

log()  { echo -e "\n>>> [$(date +%H:%M:%S)] $*"; }
fail() { echo -e "\n!!! 失败: $*" >&2; exit 1; }

log "云端部署开始"
echo "    工作空间: $ROBOCUP_WS"
echo "    仓库:     $REPO_URL_HTTPS"
echo "    PX4:      $PX4_TAG"

# ============================== 1. 系统依赖 ==============================
log "步骤 1/7：系统依赖（ROS Noetic + MAVROS + Gazebo + 无头依赖）"

if ! lsb_release -d 2>/dev/null | grep -q "20.04"; then
    fail "需要 Ubuntu 20.04（当前：$(lsb_release -d 2>/dev/null || echo 未知)）"
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    log "  安装 ROS Noetic（官方源）"
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' \
        --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 2>/dev/null || true
    sudo apt-get update -qq
    sudo apt-get install -y --no-install-recommends \
        ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras \
        ros-noetic-gazebo-ros-pkgs ros-noetic-geographic-msgs
else
    log "  ROS Noetic 已安装，跳过"
fi

log "  安装编译/工具依赖"
sudo apt-get install -y --no-install-recommends \
    python3-catkin-tools python3-pip python3-yaml \
    build-essential cmake git curl wget \
    xmlstarlet geographiclib-tools \
    xvfb libgl1-mesa-dri mesa-utils \
    protobuf-compiler libeigen3-dev libopencv-dev \
    python3-dev python3-numpy python3-matplotlib \
    > /dev/null
echo "  OK"

log "  安装 MAVROS 地理数据集（PX4 需要）"
if [ ! -d /usr/share/GeographicLib/geoids ]; then
    sudo geographiclib-get-geoids egm96-5 2>/dev/null || \
        wget -q https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O /tmp/glib.sh && \
        sudo bash /tmp/glib.sh
else
    echo "  已存在，跳过"
fi

log "  ROS 环境写入 ~/.bashrc"
if ! grep -q "ros/noetic/setup.bash" ~/.bashrc 2>/dev/null; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi

# ============================== 2. 目录结构 ==============================
log "步骤 2/7：创建工作空间目录"
mkdir -p "$ROBOCUP_WS/src" "$ROBOCUP_WS/third_party"
mkdir -p "$ROBOCUP_WS/logs"
echo "  $ROBOCUP_WS"

# ============================== 3. 项目代码 ==============================
log "步骤 3/7：获取项目代码"
if [ -d "$ROBOCUP_WS/.git" ]; then
    log "  已存在，执行 git pull"
    cd "$ROBOCUP_WS" && git pull --ff-only || log "  pull 失败（可能有本地改动），继续"
elif [ -d "$ROBOCUP_WS/src" ] && [ -n "$(ls -A "$ROBOCUP_WS/src" 2>/dev/null)" ]; then
    log "  src/ 已有内容但非 git 仓库，跳过 clone"
else
    # 优先 SSH（需配好 deploy key），失败则回落 HTTPS
    if git clone "$REPO_URL" "$ROBOCUP_WS" 2>/dev/null; then
        log "  已通过 SSH clone"
    else
        log "  SSH clone 失败，改用 HTTPS"
        rm -rf "$ROBOCUP_WS"
        git clone "$REPO_URL_HTTPS" "$ROBOCUP_WS" || fail "clone 失败，请检查网络/权限"
    fi
fi

# ============================== 4. PX4 ==============================
log "步骤 4/7：PX4-Autopilot $PX4_TAG"
PX4_DIR="$ROBOCUP_WS/third_party/PX4-Autopilot"
if [ ! -d "$PX4_DIR/.git" ]; then
    git clone --depth 1 --branch "$PX4_TAG" \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" || fail "PX4 clone 失败"
    # 拉全历史（SITL 编译需要子模块）
    cd "$PX4_DIR" && git fetch --unshallow 2>/dev/null || true
else
    log "  已存在，跳过 clone"
fi

log "  更新子模块（较慢，首次约 10~20 分钟）"
cd "$PX4_DIR"
git submodule update --init --recursive --depth 1 2>/dev/null || \
    git submodule update --init --recursive

if [ "$SKIP_PX4_BUILD" = "1" ] && [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    log "  --skip-px4-build 且已有构建产物，跳过编译"
else
    log "  编译 SITL（首次约 20~40 分钟，请耐心等待）"
    make px4_sitl_default -j"$(nproc)" 2>&1 | tail -20
fi
[ -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || fail "PX4 编译产物不存在"

log "  PX4 环境写入 ~/.bashrc"
if ! grep -q "setup_gazebo.bash" ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc << EOF
source ${PX4_DIR}/Tools/setup_gazebo.bash ${PX4_DIR} ${PX4_DIR}/build/px4_sitl_default
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_DIR}/Tools/sitl_gazebo
EOF
fi

# ============================== 5. XTDrone ==============================
log "步骤 5/7：XTDrone 依赖包"
XT_DIR="$ROBOCUP_WS/third_party/XTDrone"
if [ ! -d "$XT_DIR/.git" ]; then
    git clone --depth 1 "$XTDRONE_URL" "$XT_DIR" || log "  XTDrone clone 失败（不影响主流程）"
else
    log "  已存在，跳过"
fi

log "  安装 XTDrone 的 Python 依赖"
pip3 install --quiet --upgrade pip 2>/dev/null || true
pip3 install --quiet \
    numpy scipy matplotlib opencv-python pyyaml \
    2>/dev/null || log "  pip 安装有警告，继续"

# ============================== 6. 编译项目 ==============================
log "步骤 6/7：编译本项目（catkin_make）"
cd "$ROBOCUP_WS"
source /opt/ros/noetic/setup.bash
if [ -f /usr/local/bin/catkin_make ]; then
    : # 用系统 catkin_make
fi
catkin_make -j"$(nproc)" 2>&1 | tail -15 || fail "catkin_make 失败"
[ -f "$ROBOCUP_WS/devel/setup.bash" ] || fail "devel/setup.bash 未生成"

log "  项目环境写入 ~/.bashrc"
if ! grep -q "ROBOCUP_WS=" ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc << EOF

# ---- RoboCup 项目环境 ----
export ROBOCUP_WS=${ROBOCUP_WS}
source \${ROBOCUP_WS}/devel/setup.bash
EOF
fi

# ============================== 7. 无头模式 ==============================
log "步骤 7/7：无头模式配置"

# 无头启动脚本（xvfb + gui:=false），供没有显示器的云主机使用
cat > "$ROBOCUP_WS/scripts/cloud/start_headless.sh" << 'HEADLESS_EOF'
#!/bin/bash
# 无头模式启动 SITL：xvfb 提供虚拟显示，Gazebo 以 gui:=false 运行。
# 用法： bash start_headless.sh [机数，默认 2]
set -eo pipefail
N="${1:-2}"

source /opt/ros/noetic/setup.bash
source "${ROBOCUP_WS:-$HOME/team_ws/robocup}/devel/setup.bash"
source "${ROBOCUP_WS:-$HOME/team_ws/robocup}/third_party/PX4-Autopilot/Tools/setup_gazebo.bash" \
    "${ROBOCUP_WS:-$HOME/team_ws/robocup}/third_party/PX4-Autopilot" \
    "${ROBOCUP_WS:-$HOME/team_ws/robocup}/third_party/PX4-Autopilot/build/px4_sitl_default"
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"${ROBOCUP_WS:-$HOME/team_ws/robocup}/third_party/PX4-Autopilot/Tools/sitl_gazebo"

# 虚拟显示（Gazebo 即使 gui:=false 也需要 DISPLAY 才能起 gzserver）
if ! pgrep -x Xvfb >/dev/null; then
    Xvfb :99 -screen 0 1280x1024x24 >/tmp/xvfb.log 2>&1 &
    sleep 2
fi
export DISPLAY=:99

LAUNCH="${ROBOCUP_WS:-$HOME/team_ws/robocup}/src/robocup_swarm/launch/multi_uav_sitl.launch"
echo ">>> 无头启动 ${N} 机（DISPLAY=$DISPLAY）"
roslaunch "$LAUNCH" gui:=false num_uav:="$N"
HEADLESS_EOF
chmod +x "$ROBOCUP_WS/scripts/cloud/start_headless.sh"

# CPU 绑核脚本（多机 EKF 稳定性的关键，见 memory robocup-multi-uav-sitl）
cat > "$ROBOCUP_WS/scripts/cloud/bind_cpu.sh" << 'BIND_EOF'
#!/bin/bash
# 多机 SITL 的 CPU 绑核：gzserver 与各 PX4 实例分开绑核，避免 EKF 因资源竞争发散。
# 用法： bash bind_cpu.sh [机数，默认 2]
N="${1:-2}"
NCORE=$(nproc)

# 核数不足时给出警告并降级（少于 4 核不建议跑多机）
if [ "$NCORE" -lt 4 ]; then
    echo "!!! 警告：只有 $NCORE 核，多机 SITL + EKF 可能不稳定（建议 >=8 核）"
fi

# gzserver 绑到最后一组核，PX4 依次绑到前面的核
GZ_MASK="$((NCORE-2)),$((NCORE-1))"
[ "$NCORE" -lt 4 ] && GZ_MASK="0,1"

GZ=$(pgrep -f 'gzserver -e ode' | head -1)
[ -n "$GZ" ] && taskset -pc "$GZ_MASK" "$GZ" >/dev/null 2>&1 && echo "gzserver($GZ) -> 核 $GZ_MASK"

for i in $(seq 0 $((N-1))); do
    P=$(pgrep -f "px4.*-i $i" | head -1)
    [ -n "$P" ] && taskset -pc "$i" "$P" >/dev/null 2>&1 && echo "px4 -i $i($P) -> 核 $i"
done
BIND_EOF
chmod +x "$ROBOCUP_WS/scripts/cloud/bind_cpu.sh"

echo "  无头启动: $ROBOCUP_WS/scripts/cloud/start_headless.sh"
echo "  CPU 绑核: $ROBOCUP_WS/scripts/cloud/bind_cpu.sh"

# ============================== 完成 ==============================
log "部署完成"
cat << EOF

============================================================
 后续步骤
============================================================

1) 让环境变量生效：
     source ~/.bashrc

2) 启动仿真（无头模式，云端推荐）：
     bash \$ROBOCUP_WS/scripts/cloud/start_headless.sh 2

   等 Gazebo + 2 个 PX4 起来后，另开终端绑核：
     bash \$ROBOCUP_WS/scripts/cloud/bind_cpu.sh 2

3) 启动协同搜索（在另一个终端）：
     bash \$ROBOCUP_WS/src/robocup_swarm/scripts/run_swarm_search_2uav.sh uav_1,uav_2

   注：该脚本在仓库里的路径是 scripts/ 下；若不存在请用：
     rosrun robocup_swarm swarm_agent.py _uav_id:=uav_1 _model_name:=iris_1
     rosrun robocup_swarm swarm_manager.py _uav_ids:=uav_1,uav_2
     rosrun robocup_swarm target_sim_node.py

4) 查看状态：
     rostopic echo -n1 /uav_1/mavros/state
     rostopic echo -n1 /swarm/target_states

============================================================
 资源建议（按实测外推）
============================================================
  2 机：>= 4 核 / 16 GB        （本地 VM 实测临界）
  6 机：>= 8 核 / 32 GB        （无视觉）
  6 机 + YOLO：16 核 / 64 GB / 单卡 T4 或更好

  无头模式可省掉 gzclient（实测占 60% CPU）。
============================================================
EOF
