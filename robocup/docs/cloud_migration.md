# 云端迁移指南

把 RoboCup 2026 集群协同搜索的开发环境从本地 VMware VM 迁到云服务器。

## 为什么迁移

本地 VM 配置（4 核 / 15 GB / 无 GPU）已到瓶颈，实测数据：

| 组件 | 2 机场景 CPU | 说明 |
|---|---|---|
| gzserver | 68.9% | 687 模型的城市世界 |
| gzclient（GUI） | 60.2% | **无头模式可省掉** |
| swarm_agent ×2 | 93.7% | 已优化：摆脱 250Hz 的 `/gazebo/model_states`（原每机 37%） |
| px4 ×2 | 28.6% | |
| mavros ×2 | 11.7% | |
| **合计** | **约 2.6 核 / 4 核** | 负载 5.0+（超载） |

**外推到 6 机**：约 3.4~3.7 核（无头、agent 优化后）。再加 YOLO 视觉推理必然超载。

## 服务器选型

| 档位 | 配置 | 能跑什么 |
|---|---|---|
| 最低可行 | 8 核 / 32 GB / 无 GPU | 6 机仿真 + 几何判定检测（无视觉） |
| **推荐** | **16 核 / 64 GB / 单卡 T4 或 A10** | 6 机 + 几何判定 + 实时 YOLO |
| 充裕 | 32 核 / 128 GB / A100 或 4090 | 多场景并行实验 |

要点：
- **内存要足**：Gazebo 城市场景 + 6 个 PX4 + YOLO 权重，32 GB 偏紧
- **一定要 GPU**：YOLO 用 CPU 推理，6 路会吃掉 16 核的 1/3，延迟也不可接受
- **系统选 Ubuntu 20.04 + ROS Noetic**：与现有环境一致，代码零改动

## 部署步骤

### 1. 准备服务器

```bash
# 装 git（若镜像没带）
sudo apt-get update && sudo apt-get install -y git

# 配置 GitHub 访问（二选一）
#   a) SSH deploy key（推荐，若仓库是私有的）
ssh-keygen -t ed25519 -C "cloud-deploy" -N "" -f ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub   # 加到 GitHub → Settings → Deploy keys
#   b) 或用 HTTPS（公开仓库可直接 clone）
```

### 2. 上传并运行部署脚本

```bash
# 本地执行
scp scripts/cloud/deploy_cloud.sh user@<云主机IP>:~/

# 云端执行（约 40~90 分钟，主要是 PX4 编译）
ssh user@<云主机IP>
bash ~/deploy_cloud.sh
```

脚本会自动完成：
1. ROS Noetic + MAVROS + Gazebo 11 + 无头依赖（xvfb）
2. 创建工作空间 `~/team_ws/robocup`
3. `git clone` 本项目
4. PX4-Autopilot v1.13.2（clone + 子模块 + 编译 SITL）
5. XTDrone（gitee 源）
6. `catkin_make` 编译本项目
7. 生成无头启动 + CPU 绑核脚本，环境变量写入 `~/.bashrc`

**幂等**：可重复执行，已装的会跳过。已编译过 PX4 时用 `--skip-px4-build` 加速。

### 3. 启动（无头模式）

```bash
source ~/.bashrc

# 终端 A：启动仿真（xvfb 虚拟显示 + gui:=false）
bash $ROBOCUP_WS/scripts/cloud/start_headless.sh 2

# 终端 B：等 Gazebo 和 PX4 起来后，绑定 CPU 核
bash $ROBOCUP_WS/scripts/cloud/bind_cpu.sh 2

# 终端 C：启动协同搜索
bash $ROBOCUP_WS/scripts/cloud/run_swarm.sh uav_1,uav_2

# 停止
bash $ROBOCUP_WS/scripts/cloud/stop_swarm.sh
```

仓库内 `scripts/cloud/` 提供的脚本：

| 脚本 | 用途 |
|---|---|
| `deploy_cloud.sh` | 一键部署（ROS/PX4/XTDrone/项目编译） |
| `start_headless.sh` | 无头启动 SITL（xvfb + gui:=false） |
| `bind_cpu.sh` | CPU 绑核（多机 EKF 稳定性关键） |
| `run_swarm.sh` | 启动 agent + manager + 目标仿真 |
| `stop_swarm.sh` | 停止协同搜索节点 |

### 4. 验证

```bash
# 无人机是否就绪（期望 armed:True mode:"OFFBOARD" system_status:4）
rostopic echo -n1 /uav_1/mavros/state

# 恐怖分子是否在跑（期望 6 个，|v|=1 或 2 m/s）
rostopic echo -n6 /swarm/target_states

# 避障验证（穿墙率应 0%）
python3 $ROBOCUP_WS/src/robocup_swarm/scripts/verify_swarm_avoidance.py --duration 40
```

## 路径参数化

代码里不再硬编码 `/home/ros/team_ws/robocup`，改用两个环境变量：

| 变量 | 作用 | 默认值 |
|---|---|---|
| `ROBOCUP_WS` | 工作空间根目录 | `/home/ros/team_ws/robocup` |
| `ROBOCUP_METADATA` | 地图 metadata 路径 | `$ROBOCUP_WS/src/.../training_city_full_s7.json` |

设置 `ROBOCUP_WS` 后，Python 脚本、launch 文件、PX4 wrapper 全部自动适配。

## 已知的云端注意事项

1. **CPU 绑核必做**：多机 SITL 下 gzserver 与 PX4 争抢会导致 EKF 高度发散
   （详细根因见 `docs/` 与 memory `robocup-multi-uav-sitl`）
2. **ssh 后台进程**：`setsid ... </dev/null >log 2>&1 &` 且要在同一条 ssh 命令内 sleep，
   否则 ssh 断开时子进程被 SIGHUP 杀掉
3. **Python 日志缓冲**：后台跑的 ROS 节点加 `PYTHONUNBUFFERED=1`，否则日志不落盘
4. **GUI 需求**：无头模式下看不到 Gazebo 画面。需要可视化时：
   - 用 `xvfb` + RViz 截图，或
   - 在本地做 X11 转发（`ssh -X`），或
   - 用 VNC / 远程桌面（会增加带宽开销）

## 待办（官方资源发布后）

`docs/official_assets_missing.md` 列出的 P0 项仍需等待赛项方：
- 最终比赛 world 与随机化脚本
- 裁判节点与消息接口
- 赛项专用 Docker 镜像（若有）

收到后用 `.env` 的 `ROBOCUP_OFFICIAL_IMAGE` 替换，不重写工程结构。
