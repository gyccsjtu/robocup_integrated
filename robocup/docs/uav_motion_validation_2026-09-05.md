# 单机运动验证记录（2026-09-05）

实际工程：`D:\a\.robocup\robocup_ws`。用户消息中的 `D:\a.robocup\robocup\_ws` 不存在；现有 README、脚本和挂载指向前者。工作区根及工程根无 Git 元数据。原 `src` 仅含 `robocup_environment_baseline`。

## 当前证据

采用现有 `robocup-single-uav`、Ubuntu 20.04 / ROS Noetic / Gazebo 11 / PX4 v1.13.2 / MAVROS。首次检查时 Docker Engine 未启动；修复残留 socket 后实测现有健康检查五项通过。恢复过程与可恢复备份见 [Docker 故障记录](docker_recovery_2026-09-05.md)。本次没有安装或下载依赖、重编译 PX4、修改 `pre.data`、开发 YOLO 或启动六机。

测试为真实 headless Gazebo 物理仿真。没有把 GUI 截图或历史记录当成新飞行证据。

## 已完成测试

最终复验时间为 2026-09-05 16:27–16:28（Asia/Shanghai），通过用户交付入口 `scripts/run_single_uav_demo.ps1` 执行，退出码 0。日志为 `logs/flight/20260905T082743.152725Z/events.jsonl`，独立日志断言再次通过。

| 最终版本复验指标 | 实测值 |
|---|---:|
| 墙上 / 仿真时间 | 39.001 / 38.908 s |
| TAKEOFF / A / B 到达误差 | 0.1696 / 0.0768 / 0.0685 m |
| A / B 到达速度 | 0.0531 / 0.0514 m/s |
| 最高相对高度 | 1.5100 m |
| 采样水平 / 垂直速度峰值 | 0.4129 / 0.4802 m/s |
| 参数恢复 | 7/7 项 |

复验后另行读取实时状态：Docker `running healthy`；MAVROS `connected=true`、`armed=false`、`mode=AUTO.LAND`；`extended_state.landed_state=1`。Gazebo `get_world_properties` 只列出 `ground_plane`、`asphalt_plane`、`iris`，确为空旷单机场景。最终代码的 11 项安全回归再次通过。到此有两轮完整正常航线通过；早期失败和中断用例分开计数。

最终主文件 SHA-256：

- `src/robocup_navigation/scripts/uav_navigation_node.py`：`9A02FB0F687868054633283F728641CF8E38C066AF442D729156B0513B8D4CE5`
- `src/robocup_navigation/config/single_uav_waypoints.yaml`：`6AC61984F6BF2C1BD9EB3DD7BEF02EF15959045BDA1C93E38C8DED96672352BE`

| 用例 | 结果与证据 |
|---|---|
| 独立 catkin 包编译 | 首次 29.7 秒（含 prebuild），后续增量约 8–9 秒；无构建失败或警告 |
| Python/ROS 安全回归 | 11/11 通过：速度目标插值、无超调、非法配置、位姿接收/时间戳过期、错误 frame、NaN、限高、时钟停滞、失联、解锁前取消、服务不阻塞、落地联合判定 |
| 完整 A/B 航线 | `20260905T043026.051253Z` 通过；运行 39.48 墙上秒 / 39.24 仿真秒，退出码 0 |
| 重复控制器 | `safety_duplicate_20260905T043254` 通过；第二实例退出码 3，原节点未被挤掉，随后取消并确认落地，原节点退出码 2 |
| 位姿断流 | `safety_pose_stale_20260905T043321` 通过；达到起飞悬停后停止位姿转发，触发 `POSE_STALE`，约 8.21 秒后确认落地上锁，退出码 1 |
| 空中取消 | `safety_cancel_20260905T043350` 通过；起飞悬停后取消，约 8.37 秒后确认落地上锁，退出码 2 |
| 参数恢复 | 上述成功/安全用例均记录 7 项 `PARAM_RESTORED`；下降速度先于落地检测阈值恢复 |
| PowerShell 解析 | 运行与取消脚本解析错误均为 0 |

安全用例在发出中断后未继续到达 A/B。位姿断流只针对本控制器的测试转发输入，PX4/Gazebo 继续真实运行；检测失败后通过 PX4 AUTO.LAND 完成降落。

所有轮次日志均为 `logs/flight/<上表目录>/events.jsonl`。原始运行日志属于 `.gitignore` 中的产物；如需提交到团队仓库，应单独归档这些证据。

### 正常航线数值（043026 轮）

| 到达点 | 实际三维误差 | 实际速度 |
|---|---:|---:|
| 起飞点 | 0.1643 m | 0.0918 m/s |
| A | 0.1177 m | 0.0197 m/s |
| B | 0.0952 m | 0.0466 m/s |

最高相对高度 1.5215 m（保护上限 2.5 m）。日志采样的水平/垂直速度峰值分别为 0.4269 / 0.5035 m/s。它们是实际速度，允许看见 PX4 对 0.4/0.3 m/s 目标变化率的瞬态跟踪超调；这些配置并不是实际机体速度的硬约束。位置/速度数据来自 MAVROS 实际反馈，约 2 Hz 持久化采样；不将采样峰值宣称为连续时间全局峰值。

完整顺序为 TAKEOFF → A → B，每点均有 ARRIVED 和 HOVER_DONE；最后为 `AUTO.LAND`、`landed_state=1`、`armed=false`、`RESULT code=0`。独立脚本 `tests/check_uav_flight_log.py` 对顺序、误差/速度、悬停完成、限高/范围、最终落地上锁和无失败事件逐项断言通过。

## 暴露并修复的问题

1. `20260905T042208.027995Z`：OFFBOARD 模式超时，解锁前退出码 1。没有遥控器且 `COM_RCL_EXCEPT=0` 导致 PX4 立即走遥控器丢失返航保护；已由节点设置 OFFBOARD 例外位，同时保留 OFFBOARD 断链 Land 保护。
2. `20260905T042534.605544Z`：完成 A/B，但降落 120 秒未确认，退出码 1，不计为成功。Gazebo 模型实际静止在地面（z≈0.1047 m），PX4 `vehicle_land_detected.in_descend=false`。源码要求指令下降速度至少达到 `0.9 * max(MPC_LAND_CRWL, 0.1)`，单改 `MPC_LAND_SPEED=0.25` 小于当时阈值 0.27。恢复记录中的原下降速度 0.7 后，飞控自行确认落地上锁。随后还原该轮参数，包含被 PX4 自动夹紧的 `LNDMC_Z_VEL_MAX` 到当前源码基线 0.5。最终版本记录并配套调整/逆序恢复下降和检测参数；最终下降速度默认 0.35 m/s。
3. 安全测试后复跑：旧 `/uav_navigation/log_dir` 参数在 ROS master 中残留，节点以独占创建日志时拒绝覆盖，解锁前退出。已改为仅接受本次进程明确传入的日志目录，默认每轮生成 UTC 唯一目录。

这些失败均保留，没有计入成功率。该轮记录证明最小演示与已列安全路径，不证明长期成功率门槛已经达到。

## 文件清单

新增：

- `src/robocup_navigation/package.xml`
- `src/robocup_navigation/CMakeLists.txt`
- `src/robocup_navigation/scripts/uav_navigation_node.py`
- `src/robocup_navigation/config/single_uav_waypoints.yaml`
- `src/robocup_navigation/launch/single_uav_waypoints.launch`
- `scripts/run_single_uav_demo.ps1`
- `scripts/cancel_single_uav_demo.ps1`
- `scripts/container/run_single_uav_demo.sh`
- `tests/test_uav_motion.py`
- `tests/check_uav_flight_log.py`
- `tests/sitl_motion_safety.py`
- `docs/uav_motion_demo.md`
- `docs/uav_motion_validation_2026-09-05.md`
- `docs/docker_recovery_2026-09-05.md`

更新 `README.md` 的演示入口和下一阶段说明。前期曾给 `build_xtdrone_gazebo_ros.sh` 添加同步本队包的逻辑，最终已移除本次添加，恢复原逻辑；新包在单独生成的 `build/navigation_catkin_ws` 内增量编译。其余成员源码、规划/审核文档、历史 `pre.data` 均未编辑。

## 未完成与验证边界

- 未开发障碍地图、A*、局部避障、动态障碍、覆盖搜索、六机或比赛裁判接口；空场飞行不算避障完成。
- 未接 YOLO/人物识别；保留目标、取消和状态接口，尚非完整 Navigate action/轨迹协议。
- 当前故障实测覆盖取消、输入位姿断流和重复控制器；超高、时钟、失联等有代码与单元覆盖，但未全部做真实飞行故障注入。
- 尚未达到专项计划建议的 20 次空场成功或赛事级随机场景验收。
- Docker socket 恢复解决本次启动故障；未证明 Windows/WSL 后续重启时该上游问题不再出现。

操作命令、配置语义和接口详见 [操作说明](uav_motion_demo.md)。
