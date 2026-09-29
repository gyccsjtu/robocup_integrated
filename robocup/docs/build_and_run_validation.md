# 构建与运行验证记录

最近验证：2026-09-04（Asia/Shanghai）。以下均为本机实际结果。

| 项目 | 结果 | 实测证据 |
|---|---|---|
| 历史档案保护 | 通过 | 未修改、覆盖、删除、解压或运行 `../pre.data`；未执行历史 Python/zip |
| Docker 开发镜像 | 通过 | `robocup-2026-single-uav:local` 构建成功，约 1.29 GB |
| PX4 SITL/Gazebo 编译 | 通过 | `DONT_RUN=1 make px4_sitl_default gazebo` 成功；PX4 可执行文件与 134 个 Gazebo plugin target 完成 |
| XTDrone `gazebo_ros_pkgs` | 通过 | 六个包全部 catkin build 成功；为防 15.6 GiB 主机 OOM，固定为 2 jobs / 1 parallel package |
| ROS master | 通过 | `/gazebo`、`/mavros`、`/rosout` 可见 |
| Gazebo 单机示例 | 通过（headless） | `ground_plane`、`asphalt_plane`、`iris` 模型可见；`get_world_properties` 返回 success |
| ROS 仿真时钟 | 通过 | `/clock` 类型为 `rosgraph_msgs/Clock` 且持续产生样本 |
| MAVROS 飞控链路 | 通过 | `/mavros/state` 类型为 `mavros_msgs/State`；`connected: True`；日志为 `Got HEARTBEAT, connected. FCU: PX4 Autopilot` |
| 基础位姿 | 通过 | `/mavros/local_position/pose` 类型为 `geometry_msgs/PoseStamped`，`frame_id` 为 `map` 且持续产生样本 |
| 容器重启复现 | 通过 | `docker restart robocup-single-uav` 后 ROS、Gazebo、iris、PX4、MAVROS 和位姿全部自动恢复 |
| 一键构建脚本 | 通过 | `build_single_uav.ps1` 实际执行成功：固定 commit 校验、XTDrone overlay、PX4/Gazebo 增量构建、六包 catkin build 均通过 |
| 一键停止/冷启动 | 通过 | `stop_single_uav.ps1` 删除容器与网络后，`start_single_uav.ps1` 重新创建并自动通过五项检查 |
| 日志收集 | 通过 | `collect_logs_single_uav.ps1` 已生成容器 inspect、完整日志和 ROS 图记录 |
| Docker 原生 healthcheck | 通过 | 最终 Compose 容器状态为 `healthy`，`FailingStreak=0` |
| 最终赛事资源缺失诊断 | 通过 | 通用 `health_check.ps1` 在 `.env` 未配置时退出码为 1，并明确指出不得猜测的配置文件缺失 |
| GUI 客户端 | 2026-09-05 已通过补充验证 | 原 Ubuntu distro mount 路径曾失败；现改用 Docker Desktop 自身 WSLg socket，独立 gzclient 窗口与带 GUI 航点飞行均已通过，见 [图形验证记录](gazebo_visual_validation_2026-09-05.md) |
| 六架 `typhoon_h480` | 未执行 | 公共 XTDrone 包含模型与 launch，但尚未把它当成 2026 最终比赛场景；不伪造最终六机接口 |
| 最终裁判/比赛场景 | 等待赛项方 | 属于赛前接口集成项，不阻塞当前单机控制和避障训练 |

## 已验证启动链

```text
Docker Desktop
  -> Ubuntu 20.04 / ROS Noetic 容器
  -> roslaunch px4 mavros_posix_sitl.launch interactive:=false
  -> PX4 SITL <-> Gazebo 11 iris <-> MAVROS
  -> /clock + /mavros/state + /mavros/local_position/pose
```

后台容器必须使用 `interactive:=false`，否则 PX4 会因 stdin EOF 退出。该参数已固化到 `scripts/container/start_single_uav.sh`。

## 自动验收

```powershell
.\scripts\start_single_uav.ps1
.\scripts\health_check_single_uav.ps1
```

健康检查会同时验证 ROS master、Gazebo `iris`、仿真时钟、MAVROS 连接和本地位姿。缺少源码、构建产物、容器或话题时会立即退出并说明具体缺失项。

最近一次自动采集证据位于 `logs/single-uav/20260904-180926/`（运行产物，已由 `.gitignore` 排除）。交付时容器 `robocup-single-uav` 保持运行且状态为 `healthy`。
