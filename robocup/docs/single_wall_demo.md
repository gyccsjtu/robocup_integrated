# 单机 A* 绕墙演示

本阶段使用已知静态训练地图，在 Gazebo Classic 中运行 iris。YOLO、在线障碍观测、动态避障、城市覆盖和多机不包含在此演示中。实际验收结果见同目录的本轮验证记录；本文的操作步骤本身不代表实测通过。

## 启动

先打开本机已有 Docker Desktop，等 Engine 可用。在 Windows PowerShell 中运行：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
.\scripts\run_single_wall_demo.ps1
```

该命令检查旧场景没有活动控制器且无人机已落地、未解锁，再生成 unit / single_wall / seed=42，强制重建仿真容器以加载本次 world，等待健康检查、写入场景校验记录、打开 Gazebo，增量编译本队导航包并运行绕墙任务。每次执行都重置仿真到出生位置；不要在其他任务飞行中切图。默认最多尝试两次，但只会在“预解锁状态始终不稳定”或“解锁后 Gazebo 没有真实爬升”且再次确认落地时重建场景；碰撞、净空、越界、地图不一致等错误不会自动重试。

只准备并显示场景，不起飞：

```powershell
.\scripts\run_single_wall_demo.ps1 -PrepareOnly
```

无显示窗口运行：

```powershell
.\scripts\run_single_wall_demo.ps1 -Headless
```

自动在水平飞行途中请求取消并验收落地：

```powershell
.\scripts\run_single_wall_demo.ps1 -Headless -Case cancel
```

自动在飞行中提交 Gazebo world ENU 目标 `(2.5,-2.0)`，重新运行 A*、替换旧路线并验收：

```powershell
.\scripts\run_single_wall_demo.ps1 -Headless -Case replan
```

在线目标使用 `/uav_navigation/goal`（`geometry_msgs/PoseStamped`），要求 `frame_id=gazebo_world` 和新鲜 ROS 时间戳。消息 x/y 是地图世界坐标；z 会记录但不会直接成为飞行高度，控制器统一使用配置的 `route_altitude_m`。目标必须通过占用栅格、机身包络、连续净空、围栏和降落走廊检查。规划期间无人机保持位置；新目标覆盖等待中的旧目标，晚返回的旧规划结果会被丢弃；非法或不可达目标恢复原路线。

开发验收时可重复执行并附带一次取消测试：

```powershell
.\scripts\validate_single_wall_demo.ps1 -Runs 5 -IncludeCancel
```

`-SkipBuild` 只用于源码已在本轮成功编译且此后未修改时的重复测试；日常运行省略它。

飞行中在另一个 PowerShell 窗口受控取消：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
.\scripts\cancel_single_uav_demo.ps1
```

取消不是任务成功。确认落地、上锁后才能重新开始；不要用关闭 Docker 或强行终止进程替代取消。

## 控制与地图关系

- A* 使用训练 metadata 中已膨胀的二维栅格；连续安全检查独立使用原始障碍几何，检查全线段、端点接线与垂直起降通道。
- 机体水平半径配置为 0.40 m：iris.sdf 的旋翼中心约为 (0.13, ±0.22)，旋翼半径 0.128 m，水平外包约 0.3835 m。运行时静态余量为 0.20 m，规划再预留 0.20 m 跟踪/制动余量，因此地图栅格膨胀至少为 0.80 m。安全模块报告的 clearance 是减去机体半径后的净空，不是中心到墙的距离。
- 地图目标坐标为 world ENU 米，出生点 [-3,0]，默认目标 [3,0]，巡航中心高度 world z=2.4 m。保护高度 3.2 m 按“实际机体顶部＋垂直余量”检查，给 PX4 高度跟踪瞬态留出空间，同时低于比赛 6 m 上限。MAVROS local 的原点另行标定，不能直接把地图 x/y 作为原控制器的起点相对偏移。
- 解锁前必须连续满足 PX4 在地面、Gazebo 位于出生点、MAVROS 垂直估计稳定；解锁后 10 秒内 Gazebo 真值必须爬升至少 0.15 m，否则按失败受控降落。
- 本轮采用共同 ENU 轴假设，以 Gazebo 与 MAVROS 初始位置标定平移，随后用水平移动后的第二对位置和持续一致性检查验证。Gazebo 真值是本轮仿真辅助依赖，不声称可直接用于正式比赛。
- 每次新场景记录 world/metadata SHA-256、容器 ID、启动时间和 launch_id；控制器检查记录时效、磁盘文件及实际场景模型，拒绝旧路径或地图不匹配。
- 由既有 uav_navigation_node 唯一发布 MAVROS 最终指令；规划、安全检查与评测模块不另发飞行指令，评测模块只可请求取消或向该控制节点提交测试目标。

## 配置和结果

绕墙参数在 `src/robocup_navigation/config/single_wall_route.yaml`；原空场演示继续使用独立的 `single_uav_waypoints.yaml`。高度、速度、加速度、围栏、到点判定、悬停、安全余量和超时以配置为准。速度/加速度参数约束发送的位置目标变化，实际飞机可能有跟踪超调，需要实测检查。

每次执行保存在 `logs/single_wall/<UTC时间>_<normal|cancel|replan>/`：

- `events.jsonl`：控制节点状态、位姿、规划路线、参数和失败原因。
- `ground_truth.jsonl`：独立订阅 Gazebo 的约 20 Hz 实际位置和速度。
- `console.log`：本轮控制器输出。
- `validation.json`：机器可读验收结果，必须检查 passed 和各项 checks。
- `route_overlay.svg`：蓝色计划路线与橙色实际轨迹，灰色为障碍。

这些原始日志在 Git 忽略目录，分享证据时需单独归档到 docs 或打包。不要把一次通过的旧文件当成本轮结果。

离线回归（不解锁、不起飞）：

```powershell
docker exec robocup-single-uav bash /workspace/scripts/container/run_route_tests.sh
```

评测中的净空来自有频率上限的实际位姿采样和保守几何包络，需要同时查看采样间隔。视频、轨迹图和采样净空不等同于无限精度的连续接触检测证明。

## 恢复与范围

如果 Docker 再次出现 sailor-ingest.sock 错误，参考 `docker_recovery_2026-09-05.md`。不要恢复出厂设置或删除镜像、卷、虚拟磁盘。

本轮只接受经过核验的静态 unit 场景几何。所有允许执行的路线必须同时具有可用垂直降落走廊；不能把这项策略泛化为城市中任意位置都能原地降落。发生失联、时钟异常或未验证的降落条件时，结果须报告失败或未确认，并依赖既有 PX4 失效保护。
