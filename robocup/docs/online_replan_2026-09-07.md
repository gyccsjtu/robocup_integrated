# 单机在线目标与静态 A* 重规划记录（2026-09-07）

## 范围与结论

本阶段在既有唯一 `uav_navigation_node.py` 中接入在线目标，不新增 MAVROS 指令发布者。外部 `geometry_msgs/PoseStamped` 只提供 Gazebo world ENU 的水平目标；控制器在后台重新运行 A*，连续发送悬停 setpoint，规划成功后原子替换旧路线。最新目标优先，旧规划晚返回即丢弃；非法、占用、越界、净空不足、超围栏或无安全降落走廊的目标被拒绝并恢复原任务。

截至本文保存时，纯算法重规划 11 项测试通过；之前同一容器中的导航回归 55 项通过，随后新增的“最新目标覆盖”控制测试已通过语法检查但因 Docker Engine 不可用尚未在 ROS Python 环境执行。合法目标 `(2.5,-2.0)` 的最终 Gazebo 实飞尚未完成，因此本阶段不能标记为实飞验收通过。

## 主要实现

- `route_runtime.replan_route(route, current_world_xy, goal_world_xy, config)`：复用已验证场景的 metadata、膨胀栅格和 `SafetyMap`，从 Gazebo 实际位置重新运行 A*。
- A* 继续禁止栅格角切；每个精确起终点连接和内部线段再用原始几何连续验证，不降低 0.40 m 机体半径、0.20 m 静态余量和 0.20 m 跟踪预留。
- 目标围栏在规划前检查“目标半径＋机体/静态余量”，避免把超围栏错误误报成中间路径错误。
- 控制节点新增 `REPLAN` 状态：以单个后台 worker 规划，规划时保持 setpoint；只保留最新到达的目标；成功后发布新的 world/local `nav_msgs/Path` 并执行，失败后恢复原路线。
- PX4 local z 在起飞后可能重定基准。水平 world/local 变换保持严格连续验证；物理高度使用 Gazebo 真值检查。目标高度只在航点边界通过最新 world/local 对重标定一次，避免每个 20 Hz 周期移动目标破坏制动不变量。
- 自动观察器新增 `replan` 用例；ROS 会自行重写 `Header.seq`，因此验证器使用控制器实际记录的序号，不把发布端填写值误当任务 ID。完整任务 ID 仍需后续 action/自定义消息协议。

## 测试与仿真证据

离线/容器测试：

- A* 14 项、安全几何 16 项、运行时重规划 10 项、飞控 15 项，共 55 项曾在容器内全部通过。
- 增加目标保留半径和任意空中起点反例后，`test_route_runtime.py` 为 11 项并全部通过。
- Python 源码编译语法检查和 PowerShell 脚本解析通过。

探索性 Gazebo 运行均保留原始日志，且所有失败均完成受控落地和上锁：

1. `logs/single_wall/20260906T131402.904620Z_replan`：起飞后 PX4 local z 重定基准导致三维变换误报；改为严格验证平面变换，物理高度继续使用 Gazebo 真值。
2. `logs/single_wall/20260906T131755.755302Z_replan`：每周期动态移动 local z 目标触发 `SLEW_BRAKE_INVARIANT_VIOLATED`；改为只在航点边界重标定。
3. `logs/single_wall/20260906T132000.961834Z_replan`：复用已连续起降的旧 PX4 场景产生明显横向漂移，0.50 m 路线偏差保护触发；未放宽阈值，正式验收必须使用启动器创建干净场景。
4. `logs/single_wall/20260906T132158.581574Z_replan`：动态目标已送达规划器，但 `(3,-3)` 加上 0.60 m 机身/静态包络后超出 7 m 围栏。控制器拒绝该目标、恢复原任务并成功降落；目标改为仍需绕墙但完整包络合法的 `(2.5,-2.0)`。

第四轮控制器退出码为 0，原路线安全完成；观察器正确给出总体 FAIL，因为动态目标未被接受。采样 2108 个 Gazebo 真值，最小机身净空 0.448 m、最大世界中心高度 2.531 m，无碰撞、越界或失败落地事件。这些数据只证明拒绝与恢复流程安全，不证明合法在线重规划已经实飞成功。

## 当前环境阻塞与唯一重测命令

Luna Max 执行了有界环境检查和标准 WSL/Docker 恢复。当前 Docker Client 为 29.7.2，但 Server named pipe 不存在，`com.docker.backend` 未恢复；已停止重复尝试，没有恢复出厂、重装或删除 Docker 数据。

重启 Windows、打开 Docker Desktop 并确认 Engine running 后，仅执行：

```powershell
Set-Location 'D:\a\.robocup\robocup_ws'
.\scripts\run_single_wall_demo.ps1 -Headless -Case replan -MaxAttempts 1
```

只有新生成目录中的 `validation.json` 为 `passed: true`，且包含 `replan_triggered`、`replan_accepted`、`actual_goal_reached`、`landed` 全部为 true，才能将在线重规划标记为 Gazebo 验收通过。

## 尚未完成

- 合法在线目标的干净 Gazebo 场景最终实飞。
- 完整 action/自定义消息任务 ID、反馈、结果和期限协议；`Header.seq` 不能承担稳定业务 ID。
- 传感器地图更新、未知空间、局部避障和动态障碍重规划。
- full 城市、六机、搜索覆盖以及 YOLO/人物位置接入。
