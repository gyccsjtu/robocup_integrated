# single_wall 单机 A* 实飞验证记录（2026-09-06）

## 结论

当前最终配置已在本机 Docker 容器 `robocup-single-uav` 中，用 ROS Noetic、Gazebo Classic 11、PX4 v1.13.2 和 MAVROS 完成真实仿真验收：一架 Iris 自动连接、等待地面状态稳定、进入 OFFBOARD、解锁、起飞，按新生成的 A* 路线绕过 `single_wall`，在目标点悬停，受控下降并切换 AUTO.LAND，最后落地和自动锁桨。飞行中取消流程也通过。

本次只证明已知静态地图上的单机规划与执行，不代表未知障碍、动态避障、YOLO 联动、完整城市或六机协同已经完成。

## 最终参数与安全契约

- 地图：unit / single_wall / seed 42，world ENU，出生点 `[-3, 0]`，目标 `[3, 0]`。
- 巡航中心高度：2.4 m；实际机体顶部加垂直余量的保护上限：3.2 m，低于比赛 6 m 上限。
- 最大水平设定点速度：0.30 m/s；最大垂直设定点速度：0.25 m/s。
- 最大水平/垂直设定点加速度：0.25 / 0.20 m/s²。
- 到点容差与速度：0.12 m / 0.12 m/s；终点悬停 2 s。
- Iris 水平半径 0.40 m，静态安全余量 0.20 m，规划跟踪/制动余量 0.20 m；metadata 栅格膨胀 0.80 m。
- 解锁前连续稳定 1.5 s，垂直估计绝对速度不超过 0.08 m/s，Gazebo 出生点偏差不超过 0.15 m。
- 解锁后 10 s 内 Gazebo 必须真实爬升至少 0.15 m，否则失败并进入有界降落。
- 控制器持续检查连接、位姿/状态/时钟新鲜度、唯一写入者、场景模型、坐标对、水平围栏、实际高度、连续几何净空、路线偏差和各阶段超时。

## 离线测试

在容器内执行：

```powershell
docker exec robocup-single-uav bash /workspace/scripts/container/run_training_city_tests.sh
docker exec robocup-single-uav bash /workspace/scripts/container/run_route_tests.sh
```

结果：地图生成器 38 项全部通过；导航 48 项全部通过，其中 A* 14、连续路线安全 15、运行时场景/规划 6、飞控运动与门禁 13。覆盖无路可走、非法目标、角点穿越、连续几何、旧 receipt/哈希变化、Gazebo 模型错位、规划余量不足、加速度限制、预解锁稳定、起飞无真实爬升、取消和降落确认。

## 最终实飞

最终 normal 命令：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
.\scripts\run_single_wall_demo.ps1
```

结果文件：`logs/single_wall/20260906T122801.933729Z_normal/validation.json`（原始 logs 被 Git 忽略）。

| 项目 | 结果 |
|---|---:|
| 总体 | PASS |
| 控制器退出码 | 0 |
| 墙钟时长 | 101.464 s |
| Gazebo 真值样本 | 2067 |
| 最大 ROS 采样间隔 | 0.124 s |
| 最小机体表面净空 | 0.429 m |
| 最大 Gazebo 世界中心高度 | 2.370 m |
| 终点误差 | 0.024 m |
| 最大观测水平速度 | 0.366 m/s |
| 最终状态 | connected=true, armed=false, landed_state=1 |

所有验收项均为 true：控制结果、落地、无超时、净空、围栏、限高、采样连续、真实起飞、实际到达目标、航点顺序、到点速度/容差、悬停、第二坐标对验证和无失败事件。

飞行中取消命令：

```powershell
.\scripts\run_single_wall_demo.ps1 -Headless -SkipBuild -Case cancel
```

结果文件：`logs/single_wall/20260906T122526.161706Z_cancel/validation.json`。总体 PASS，控制器专用取消退出码 2；取消已触发、原因保留为 `CANCELLED`、未到目标、受控降落并锁桨。963 个 Gazebo 样本，最小机体净空 1.180 m，最大世界中心高度 2.584 m。

飞行后再次执行 ground guard 和五项健康检查：ROS master、Gazebo Iris、`/clock`、MAVROS 连接、本地位姿全部通过；容器 healthy，Gazebo viewer 保持运行。

## 排障过程与修正

验收前保留了失败证据，没有把旧成功冒充最终结果：

- 首次集成暴露纯函数返回 tuple 后被控制器原地修改的问题，已在发布边界转为 list。
- 一轮 PX4 已解锁但 Gazebo 完全未升空，旧逻辑等待到 MOVE_TIMEOUT。现在增加解锁前连续稳定门禁、Gazebo 真值起飞看门狗和只针对该启动类故障的最多两次全场景重建。
- 旧 0.60 m 栅格膨胀使理论路线几乎贴着最低净空，实际一轮只剩 0.143 m，安全检查正确中止。现在机体半径统一为 0.40 m，栅格膨胀改为 0.80 m，并强制校验“半径＋静态余量＋规划执行余量”。最终净空约 0.429 m。
- 2.5 m 中心目标的一轮取消测试出现约 3 mm 的顶部保护越限。没有抬高保护阈值，而是将保守巡航中心高度降为 2.4 m；重测取消和 normal 均通过。
- unsafe 事件现在先更新本次实际净空，不再记录上一帧通过值。
- Docker Desktop 中途再次因 stale sailor socket 退出；两个运行目录被移动为可恢复的 `*.codex-backup-20260906-201225` 后重启成功，没有恢复出厂设置、删除镜像、卷或工程文件。

## 主要文件

- `src/robocup_navigation/scripts/uav_navigation_node.py`：唯一 MAVROS 控制节点；场景门禁、OFFBOARD/解锁、运动、真值保护、取消和降落状态机。
- `src/robocup_navigation/src/robocup_navigation/route_runtime.py`：receipt/哈希、world/metadata/Gazebo 一致性、新鲜 A*、坐标和规划余量校验、加速度限制。
- `src/robocup_navigation/src/robocup_navigation/route_safety.py`：基于原始 box/cylinder 几何的连续路线、净空与降落通道检查。
- `src/robocup_navigation/config/single_wall_route.yaml`：本轮全部飞行、安全和超时参数。
- `scripts/run_single_wall_demo.ps1`：一条命令的安全编排和有界启动恢复。
- `scripts/container/run_single_wall_demo.sh`、`scripts/container/single_wall_tools.py`：容器执行、ground guard 和场景 receipt。
- `tests/sitl_single_wall.py`：独立 Gazebo 真值观察器和机器可读验收。
- `tests/test_route_runtime.py`、`tests/test_route_safety.py`、`tests/test_uav_motion.py`：离线回归。
- `src/robocup_training_worlds/config/training_city.yaml`、`scripts/generate_training_city.py`：0.80 m 栅格膨胀、2.4 m unit 规划高度和 0.20 m 仿真出生高度。

没有修改 `pre.data` 或 `third_party/*`，也没有增加第二个 MAVROS 控制节点。

## 未完成

- 在线传入新目标后重新规划；route mode 当前有意拒绝绕过规划器的直接目标。
- 激光雷达/深度相机等在线障碍观测、局部避障、动态障碍和飞行中重规划。
- small/full 城市的飞行参数与验收；full 仅用于地图/算法压力测试。
- 官方最终 world、裁判接口和六机出生/命名约束接入。
- 六机协同、搜索覆盖、YOLO/人物目标位置接入。

下一阶段应保持 `uav_navigation_node.py` 为唯一最终指令出口，在它之前增加“目标接口 → A* 重规划 → 局部避障/轨迹跟踪”，先做单机在线目标和静态重规划，再进入动态障碍与多机。
