# 2026 多旋翼无人机集群协同搜索仿真赛项环境

本工程是独立的、可替换的比赛开发环境。当前已包含单机静态已知地图 A* 训练基线，但仍不包含目标检测、完整搜索、动态避障或多机协同。它不会修改 `../pre.data`，也不会运行或修补历史 `robocup_new.py` 与 `ego_planner.zip`。

工程实际路径为 `D:\a\.robocup\robocup_ws`。赛事规范给出的 XTDrone 网址确实是官方指定的公共仿真平台来源；目前已经按其 Ubuntu 20.04 / ROS Noetic / PX4 1.13.2 配置完成单机基线。最终比赛场景、裁判程序和接口仍应以赛项方后续交付为准，但不妨碍现在进行单机起飞、控制和避障训练。

## 已完成

- Windows 11 + WSL2 + Ubuntu 20.04.6 + Docker Desktop 已安装并审计；Linux 容器和 RTX 4060 容器 GPU 已验证。
- 已固定并导入 XTDrone `1_13_2` 与 PX4 `v1.13.2`，应用 XTDrone 官方安装页规定的 PX4 overlay。
- 已构建 `robocup-2026-single-uav:local`，包含 ROS Noetic、Gazebo 11、MAVROS、GeographicLib、Python 3 和编译工具链。
- PX4 SITL、XTDrone Gazebo 插件和 XTDrone 修改版 `gazebo_ros_pkgs` 已实际编译。
- `iris` 单机仿真、ROS master、`/clock`、MAVROS 连接和本地位姿已验证；容器重启后再次验证通过。
- 2026-09-05 新增独立 `robocup_navigation` 包，已实测单机起飞、A/B 航点、悬停和 AUTO.LAND 自动上锁。见 [运动演示说明](docs/uav_motion_demo.md) 与 [本轮验证记录](docs/uav_motion_validation_2026-09-05.md)。空旷航点演示不代表避障完成。
- 2026-09-05 新增独立 `robocup_training_worlds` 包：按 seed 生成自包含 Gazebo Classic 训练城市（unit/small/full）与 metadata，38 条离线测试通过，并已加载到 Gazebo Classic。见 [训练地图交付记录](docs/training_worlds_2026-09-05.md)。
- 2026-09-06 已把 `single_wall` metadata 栅格接入唯一飞控节点：自动执行预解锁稳定检查、OFFBOARD、解锁、Gazebo 真值起飞确认、A* 绕墙、终点悬停和受控降落。最终 normal 与飞行中取消均实测通过；这仍是已知静态地图演示，不等于在线避障完成。见 [运行说明](docs/single_wall_demo.md) 与 [验证记录](docs/single_wall_validation_2026-09-06.md)。

完整证据见 [主机审计](docs/environment_audit.md)、[版本清单](docs/version_manifest.md)、[验证记录](docs/build_and_run_validation.md) 和 [待接入官方资源](docs/official_assets_missing.md)。

## 一键启动（当前机器）

在 Windows PowerShell 中：

```powershell
Set-Location D:\a\.robocup\robocup_ws
.\scripts\start_single_uav.ps1
```

随后可执行：

```powershell
.\scripts\health_check_single_uav.ps1
.\scripts\collect_logs_single_uav.ps1
.\scripts\stop_single_uav.ps1
.\scripts\reset_single_uav.ps1       # 删除生成的 PX4/catkin 构建目录；保留镜像和源码
```

默认使用无界面 Gazebo server，适合稳定开发和自动测试。2026-09-05 已接通 WSLg 图形显示；新增独立 Gazebo 客户端，不必修改 `ROBOCUP_GAZEBO_GUI` 或重建后台仿真。见 [图形演示说明](docs/gazebo_visual_demo.md)。

打开三维窗口并自动飞行：

```powershell
.\scripts\run_single_uav_visual_demo.ps1
# 只看画面、不起飞：
.\scripts\show_single_uav.ps1
```

运行单机 A/B 运动演示（自动增量构建控制包并复用以上环境）：

```powershell
.\scripts\run_single_uav_demo.ps1
# 在另一终端受控取消，并等待落地/上锁确认：
.\scripts\cancel_single_uav_demo.ps1
```

运行单机 A* 绕墙演示（自动生成地图、重建场景、打开 Gazebo、飞行并验收）：

```powershell
.\scripts\run_single_wall_demo.ps1
```

## 构建或重建

已存在固定源码时，增量重建：

```powershell
.\scripts\build_single_uav.ps1
```

删除本工程生成的 PX4 与 catkin 构建目录后再完整编译：

```powershell
.\scripts\build_single_uav.ps1 -Clean
```

首次构建需要联网下载容器依赖；建成后启动、停止和健康检查不需要互联网。不要在赛场运行 `apt`、`pip`、`rosdep update`、`git clone` 或 `docker pull`。

## 在干净目标机重建

1. 安装 Ubuntu 20.04、WSL2 和 Docker Desktop；确认 Docker 使用 Linux containers。
2. 将本工程复制或检出到任意目录。脚本根据自身位置解析根目录，不依赖用户目录。
3. 在有网准备阶段获取并固定以下源码到 `third_party/`：

   ```text
   XTDrone:       https://gitee.com/robin_shaun/XTDrone.git
   branch:        1_13_2
   commit:        62339a816ef815113a0366a62e8aca4be3000f80

   PX4-Autopilot: https://github.com/PX4/PX4-Autopilot.git
   tag:           v1.13.2
   commit:        46a12a09bf11c8cbafc5ad905996645b4fe1a9df
   ```

   PX4 必须包含递归 submodule。Windows 检出可能把脚本转换为 CRLF；本工程只会规范化生成的 overlay/编译副本，不修改固定的 XTDrone 源档案。
4. 运行 `.\scripts\build_single_uav.ps1 -Clean`，再运行 `.\scripts\start_single_uav.ps1`。
5. 以 `.\scripts\health_check_single_uav.ps1` 为验收入口；五项全部为 `OK` 才算重建完成。
6. 离线备份开发镜像：

   ```powershell
   docker save robocup-2026-single-uav:local -o robocup-2026-single-uav.tar
   Get-FileHash .\robocup-2026-single-uav.tar -Algorithm SHA256
   ```

   大型 tar 应放在源码目录外，不提交到仓库。

## 目录职责

```text
robocup_ws/
├── docker/                    # 可复现开发镜像
├── third_party/               # 固定上游源码（git 忽略）
├── build/                     # catkin 生成物（git 忽略）
├── src/                       # 本队后续 ROS 节点
├── config/                    # 接口和环境配置
├── launch/                    # 本队启动文件
├── scripts/                   # 构建、启动、停止、检查、日志
├── tests/                     # 自动验证
├── logs/                      # 运行日志（git 忽略）
└── docs/                      # 审计、版本和验收记录
```

## 官方比赛资源接入

规则公开网页和 XTDrone 平台已经足以搭建训练基线，因此“官方比赛 Docker 镜像”不是当前开发的硬阻塞。若赛项群另行发放比赛镜像、最终 world、六机 launch、裁判程序或消息包，应记录 digest/SHA-256，并通过现有 `.env` 与只读挂载接口接入，不覆盖当前源码。

收到最终资源后使用通用官方入口：复制 `.env.example` 为 `.env`，填写真实镜像、资源路径、ROS setup 和启动命令，再运行 `scripts\build.ps1`、`scripts\start.ps1`、`scripts\health_check.ps1`。所有 `PENDING_*` 值都会被拒绝，防止猜接口。

## 训练地图（实验）

`robocup_training_worlds` 是确定性随机城市生成器：按 seed 产出自包含的 Gazebo
Classic SDF world 与机器可读 metadata，只做环境，不发布任何飞行指令。

```powershell
# 生成并启动 unit 场景（10x10 m，A* 单元场景）
.\scripts\start_training_city.ps1 -Preset unit -Seed 42 -Category single_wall
# 启动 small 场景（30x20 m；起飞前先放宽并复验 5 m 半径围栏）
.\scripts\start_training_city.ps1 -Preset small -Seed 2026
# 只生成 full（200x100 m）不飞行：当前 max_radius_m=5 的控制器禁止飞全图
.\scripts\generate_training_city.ps1 -Preset full -Seed 7
# 离线测试（不开 Gazebo）
.\scripts\run_training_city_tests.ps1
```

道路宽度、建筑与灯杆尺寸/数量/高度、出生点与目标候选区都是**团队训练假设**，
不是官方参数；`full` 的 `x[-100,100]` / `y[-50,50]` 只是团队坐标约定。正式 world、
随机脚本接口、裁判话题仍未公布。详见 [训练地图交付记录](docs/training_worlds_2026-09-05.md)
与 [包内说明](src/robocup_training_worlds/README.md)。

## 下一阶段：在线寻路与避障

单机运动执行入口已落在 `robocup_navigation`，`single_wall` 静态 A* 已接入并实飞。route mode 现已把 `/uav_navigation/goal` 接入安全 A* 重规划：规划期间悬停，只执行最新完成的合法路线，非法目标恢复原任务，最终 MAVROS 出口仍只有控制节点。当前算法与控制回归已通过，但合法在线目标的最终 Gazebo 验收因 Docker Engine 未恢复而待重测，不能把代码完成写成实飞通过；见 [在线重规划记录](docs/online_replan_2026-09-07.md)。下一步仍是完成该验收，再增加局部障碍感知和动态重规划。历史档案保持不变；YOLO 与人物识别由感知成员并行开发。

训练地图 metadata 的离线 A* 基线位于同一包内。它只计算路径，不发送 MAVROS 指令：

```powershell
.\scripts\run_astar_tests.ps1
.\scripts\plan_training_route.ps1 -AsciiOutput /workspace/build/training_city/current_route.txt
```

离线 A* 仍可单独使用；飞行入口会在执行前重新规划并校验场景哈希、原始连续几何、坐标变换和执行余量。操作与实测详情见 [A* 验证记录](docs/astar_validation_2026-09-05.md) 和 [single_wall 验证记录](docs/single_wall_validation_2026-09-06.md)。
