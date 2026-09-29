# 已安装与配置组件记录

记录时间：2026-09-04（Asia/Shanghai）。本记录只描述本机实际完成的安装与验证，不把开发基线写成最终裁判环境。

| 组件 | 实际状态 | 用途 | 已验证内容 |
|---|---|---|---|
| Ubuntu 20.04.6 LTS | 已安装为 WSL2 发行版 | 与赛事要求的 Ubuntu 20.04 保持一致，用于 Linux 开发与环境核验 | 系统版本为 20.04，Python 为 3.8.10 |
| 既有 Ubuntu 26.04 | 保留，未改动 | 原有开发环境，不作为本赛项比赛基线 | 未修改或替换 |
| Docker Desktop 4.89.0 | 已安装，WSL2/Linux containers 后端 | 承载赛项方将提供的比赛镜像，支持离线复现 | Docker Engine/CLI 29.7.2；`hello-world` 成功运行 |
| NVIDIA 容器运行能力 | 已验证 | 供官方镜像在允许时使用 GPU | CUDA 12.4 Ubuntu 20.04 容器中检测到 RTX 4060 Laptop GPU、8 GiB 显存、驱动 592.00 |
| 开发验证 ROS 镜像 | 已构建：`robocup-2026-dev-noetic:local` | 仅验证 Ubuntu 20.04 + ROS Noetic + Python 3 + catkin 基础链路；不是比赛镜像 | 两次完成干净 `catkin_make`、`roscore`、`/rosout` 和 `rosgraph_msgs/Log` 检查 |
| `robocup_ws` 工程骨架 | 已创建 | 存放比赛镜像配置、依赖清单、ROS 最小工作区、脚本、日志与文档 | `build/start/stop/reset/health_check/collect_logs` 的 PowerShell/Bash 入口已创建并做语法检查 |
| XTDrone `1_13_2` | 已固定在 commit `62339a8...` | 规范指定的公共仿真平台 | overlay、模型和修改版 `gazebo_ros_pkgs` 已实际使用 |
| PX4 v1.13.2 | 已固定在 commit `46a12a0...` | 飞控 SITL | PX4 与 134 个 Gazebo plugin target 编译成功 |
| ROS/Gazebo/MAVROS 单机镜像 | `robocup-2026-single-uav:local` | 日常单机训练基线 | `iris`、ROS master、时钟、MAVROS connected 与本地位姿通过，容器重启复现通过 |

## 尚未接入的赛项专用资源

以下内容必须以赛项方或赛项技术群提供的实际包为准，当前没有用通用版本伪造：

- 若赛项方另行发放：比赛 Docker 镜像及其 digest 或离线 tar；
- 裁判程序、ROS 话题/服务/消息类型和允许接口；
- 六架 `typhoon_h480` 的比赛 world、模型、传感器和控制资源；
- 最终比赛六机启动与复位命令；
- 若赛项方另行提供，赛项专用镜像或对公共 XTDrone 的补丁包。

拿到这些资源后，将其精确版本、镜像摘要、许可证和 SHA-256 记录到 `docs/version_manifest.md`，并填写工程根目录 `.env` 后再启动官方环境。
