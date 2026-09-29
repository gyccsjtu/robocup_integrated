# Gazebo 图形显示与航点飞行验证（2026-09-05）

## 当前环境复核

本轮重新检查了实际运行状态，未沿用此前“已验证”的状态作为结论：Docker Desktop 可用，`robocup-single-uav` 正在运行；ROS master、iris 模型、持续 `/clock`、MAVROS 连接和本地位姿均通过现有健康脚本。

WSL 中有 Ubuntu-20.04、Ubuntu-26.04 和 docker-desktop。Ubuntu 的 WSLg socket 存在，但本轮**没有使用 Ubuntu UNC 挂载**，而是验证 Docker Desktop 后端已有 `/mnt/host/wslg/.X11-unix/X0`。通过只读 bind mount，在现有单机镜像内执行 `test -S /tmp/.X11-unix/X0` 成功。未新增安装包、修改防火墙或重建镜像。

## 图形结果

- 创建显示容器 `robocup-single-uav-gui`，复用现有仿真网络、模型挂载和镜像；容器仅运行 `gzclient`。
- GUI 日志确认 Gazebo 11.15.1，连接 `http://127.0.0.1:11345`。
- 桌面检查实际发现 `Gazebo (docker-desktop)` 窗口，并看到地面与 iris 四旋翼。不是仅根据容器 running 状态判断显示成功。
- 初始飞机在世界坐标约 `(3.846, 3.982, 0.105)`，在默认镜头外；通过 `gz camera -c gzclient_camera -f iris` 成功对准。新入口自动执行此步骤。
- 采用软件渲染。观察时普通窗口约 13–37 FPS，全屏约 8 FPS；这是不同画面状态的即时读数，不是性能基准。仿真实时倍率观察约 0.98–1.00。
- 原 Compose headless 服务保持运行，不增加第二个 MAVROS 控制节点。替换过一次本轮临时建立的显示容器；没有删除仿真容器、项目文件、镜像或历史数据。

## 启动入口实测

使用用户同款 **Windows PowerShell 5.1** (`powershell.exe -NoProfile -ExecutionPolicy Bypass -File ...`)：

1. `show_single_uav.ps1`：健康检查全部通过，创建 GUI，约 30 秒完成整个入口，退出码 0；不启动飞行。
2. `run_single_uav_visual_demo.ps1`：识别并复用已有 GUI，完成健康检查及既有控制包增量构建，运行完整飞行，退出码 0。
3. `start_single_uav.ps1`：移除容易在 Windows PowerShell 5.1 跨 shell 丢失引号的内联 Bash，改用 `container/check_single_uav_connected.sh`。复用运行中的仿真测试成功、健康检查全部通过。连接轮询采用 90 秒预算（最后一次有界探测/轮询可能使实际退出略晚）。
4. 新增 Bash 文件 `bash -n` 语法检查和 PowerShell 解析检查通过。

本轮没有为了图形验证而关闭/重启 Docker Engine，也没有重新执行整机冷启动；不能将本次复用测试描述成重启电脑后的验证。

## 带 GUI 的完整飞行

本机时间 17:02:10–17:02:51，日志：

`logs/flight/20260905T090210.874913Z/events.jsonl`

GUI 容器全程保持运行。流程：持续 setpoint → OFFBOARD 确认 → 解锁确认 → TAKEOFF → A/悬停 → B/悬停 → 低速接近地面 → AUTO.LAND → 落地自动上锁 → 恢复 7 项飞控参数。

独立日志检查命令：

```powershell
docker exec robocup-single-uav python3 /workspace/tests/check_uav_flight_log.py /workspace/logs/flight/20260905T090210.874913Z/events.jsonl
```

| 验收项 | 本轮结果 |
|---|---|
| 独立日志检查 | `passed: true` |
| 控制节点 / 包装脚本退出码 | 0 / 0 |
| 飞行墙上时间 / 仿真时间 | 40.585 s / 40.104 s |
| 起飞高度设置 / 记录最高相对高度 | 1.5 m / 1.483 m |
| A 到点误差 / B 到点误差 | 0.1266 m / 0.0531 m，均小于 0.25 m |
| 记录最大水平 / 垂直速度绝对值 | 0.4828 / 0.5240 m/s |
| 最终状态 | connected=true，AUTO.LAND，armed=false，landed_state=1 |
| 结果 / 原因 | succeeded=true，MISSION_COMPLETE |
| 参数恢复 | 7 项全部记录 PARAM_RESTORED |

速度配置限制的是 setpoint 变化率，不是实测机体速度硬上限；本轮观测存在跟踪超调，不能将 0.4/0.3 m/s 描述成实际速度绝不超过的保证。详见原运动说明。

## 文件与未完成项

新增：`scripts/show_single_uav.ps1`、`scripts/run_single_uav_visual_demo.ps1`、`scripts/container/show_single_uav.sh`、`scripts/container/check_single_uav_connected.sh`、本记录和 `docs/gazebo_visual_demo.md`。

更新：`scripts/start_single_uav.ps1` 的连接探测；README 和运动说明增加可见演示入口；旧环境验证记录追加本轮结果。没有改动 `pre.data`、第三方模型、飞行控制算法或 YOLO 工作。

未完成/未宣称：Gazebo 硬件加速、传统直升机建模、地图/寻路/避障、YOLO、六机、最终比赛场景。原有空旷单机航点飞行不代表避障完成。
