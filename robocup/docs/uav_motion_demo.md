# 单机运动与航点演示

工作目录：`D:\a\.robocup\robocup_ws`。仅面向现有 `robocup-single-uav` 容器中的 PX4 v1.13.2 / ROS Noetic / Gazebo iris 仿真。

## 一条命令

需要看到 Gazebo 三维画面时，改用 `scripts/run_single_uav_visual_demo.ps1`；只看画面用 `scripts/show_single_uav.ps1`。详见 [图形演示说明](gazebo_visual_demo.md)。下方原命令保留为运动执行入口，本身不负责打开窗口。

PowerShell：

```powershell
& 'D:\a\.robocup\robocup_ws\scripts\run_single_uav_demo.ps1'
```

Docker Desktop 需要处于可用状态。脚本复用 `single_uav_common.ps1`、`start_single_uav.ps1` 和 `health_check_single_uav.ps1`；自动增量构建本次新增的 `robocup_navigation`，随后启动演示。构建目录为 `build/navigation_catkin_ws`，扩展现有 XTDrone devel 空间，无须重装环境或重编译 PX4/插件。原 `build_xtdrone_gazebo_ros.sh` 保持原逻辑。

正常流程：连接与落地检查 → 记录/配置飞控参数 → 持续预发送 setpoint → 确认 OFFBOARD → 解锁 → 起飞 → A → 悬停 → B → 悬停 → 低速下降 → AUTO.LAND → 确认落地且自动上锁 → 恢复参数。

按当前 PX4 v1.13.2 的实测顺序，先预发送，再 OFFBOARD、再解锁。服务返回 `mode_sent` 只表示请求发出，切换成功以 `/mavros/state` 为准。[PX4 官方 Python 示例](https://docs.px4.io/v1.13/en/ros/mavros_offboard_python)。持续 setpoint 和丢失保护见 [OFFBOARD 文档](https://docs.px4.io/v1.13/en/flight_modes/offboard)。

## 参数与坐标

配置：`src/robocup_navigation/config/single_uav_waypoints.yaml`。起始原点为连接后采集的、已确认落地的 MAVROS 本地位姿；配置航点是相对该原点的 ENU 米制偏移。x 向东、y 向北、z 向上。不会再次做 ENU/NED 转换。这里的高度是相对起始地面的高度，不是海拔，也不是复杂地形上的实时离地高度。

| 项目 | 默认值 |
|---|---|
| 起飞高度 / A / B | 1.5 m / `(1,0,1.5)` / `(1,1,1.5)` |
| 水平 / 垂直 setpoint 变化速度 | 0.4 / 0.3 m/s |
| 降落接近 / PX4 最终下降速度 | 0.25 / 0.35 m/s |
| 到点位置 / 速度门限 | 0.25 m / 0.2 m/s，持续 0.8 ROS 秒 |
| A/B 悬停 | 各 2 ROS 秒，位置与速度需持续满足门限 |
| 高度 / 水平范围保护 | 相对起始原点 2.5 m / 半径 5 m |
| 位姿 / 时钟停滞看门狗 | 2 秒墙上时间 |
| 连接 / 单动作 / 任务 / 降落超时 | 60 / 120 / 480 / 120 秒墙上时间 |

setpoint 速度限制约束目标位置变化率，实际飞行速度受 PX4 跟踪误差影响，不能把它描述为实际速度绝不超调。日志记录实际位置与速度。运动、预发送和悬停使用 ROS 时间；超时和断流检测使用单调实时时间，适应仿真快慢，并检测 `/clock` 停滞/回退。

自定义配置（路径为容器内路径）：

```powershell
.\scripts\run_single_uav_demo.ps1 -ConfigFile /workspace/config/my_flight.yaml
```

## 取消、结果与日志

在另一个 PowerShell 终端执行：

```powershell
& 'D:\a\.robocup\robocup_ws\scripts\cancel_single_uav_demo.ps1'
```

取消后由同一个控制节点请求 `AUTO.LAND` 并等待落地/自动上锁。等待终端 `LANDED` 和 `RESULT` 后再停止仿真。不要把关闭 PowerShell、停止 Docker 或 `rosnode kill` 当作受控取消；Docker exec 客户端的 Ctrl-C 不保证把信号传到容器内节点。直接给节点进程的 SIGINT/SIGTERM 会设置取消标记并保留 ROS 通信直到降落结束。

每次记录在 `logs/flight/<UTC时间>/events.jsonl`，包括完整配置、连接/模式/解锁变化、实际位姿/速度、目标和 setpoint、到点误差、状态转换、原飞控参数、失败原因和结果。`/uav_navigation/status` 为 `std_msgs/String` JSON 状态/事件输出，最后一条 `RESULT` 是结果。节点退出码：0 成功，1 失败或落地未确认，2 取消/非法配置，3 控制权冲突。包装脚本遇到非零退出码会报错，不会将失败写成成功。

## 单一控制权与失效处理

唯一运动控制节点是 `uav_navigation_node.py`；所有最终 MAVROS setpoint、模式和解锁请求都由它产生。服务调用在有界数量的后台线程中执行，主循环以 20 Hz 墙上时间发送 setpoint。启动前先获取容器内排他文件锁，防止同名 ROS 节点互相挤掉；同时定期查询 ROS 图，发现其他 setpoint 发布者则拒绝启动或降落。ROS1 没有全局强制访问控制，无法阻止其他程序直接调用飞控服务，因此团队仍需遵守“只向此节点提交目标”的约定。

启动时拒绝接管已解锁/未落地飞机，拒绝错误坐标系、过期/非有限位姿及越界配置。飞行中失联、位姿/速度过期、时钟异常、超高/越界、OFFBOARD 丢失会触发带原因的降落流程。降落以 `ExtendedState.ON_GROUND` 和新鲜的 `armed=false` 共同确认；不因高度接近零而强制上锁。降落超时会报告 `LANDING_UNCONFIRMED` 并停止 setpoint，将恢复留给 PX4 失去 OFFBOARD 的降落保护；链路彻底失效时不能保证从本节点观察到安全落地。

## 飞控参数配套

无人值守仿真没有遥控器。节点在解锁前记录并配置：`COM_RCL_EXCEPT` 保留原位并设置 OFFBOARD 例外位 4；`COM_OBL_ACT=0`、`COM_OBL_RC_ACT=4`（OFFBOARD 丢失时 Land）；`COM_OF_LOSS_T=1` 秒。

最终下降由 `MPC_LAND_SPEED` 采用 YAML 的 `auto_land_speed_mps`。节点先记录 `LNDMC_Z_VEL_MAX` 和 `MPC_LAND_CRWL`，必要时下调到不超过最终下降速度，避免低速下降无法满足落地检测。当前源码的检测门限为 `0.9 * max(MPC_LAND_CRWL, 0.1)`；初次测试暴露了只调下降速度导致无法自动上锁的问题，详情见验证记录。

正常落地或可确认已上锁的失败结束后，按逆序恢复原参数并记录结果。恢复顺序先还原下降速度，再还原检测阈值，避免 PX4 自动夹紧阈值。如果失联/仍解锁，则记录 `PARAM_RESTORE_DEFERRED`，保留降落保护；应在确认落地后依据该轮 `PARAM_ORIGINAL` 恢复，不能盲目重启演示。

## 后续目标输入接口

默认 `accept_external_goals: false` 保持固定 A/B 演示。启用后，在起飞结束后的 MOVE/HOVER 阶段接收 `/uav_navigation/goal`（`geometry_msgs/PoseStamped`）。目标必须是 `frame_id=map` 的绝对本地 ENU 位置，带有效 ROS 时间戳和 `header.seq`，并满足高度/半径检查。其坐标与配置中的相对偏移不同。过期、起飞阶段或越界目标会被拒绝。

接受的新目标替换当前目标和剩余 A/B 队列，发送 `GOAL_PREEMPTED`/`GOAL_ACCEPTED`；到达新目标、悬停后降落。全局任务期限不因新目标而延长。该接口是本轮预留的运动执行入口，不是完整 Navigate action 协议；需要继续完成目标 ID、抢占结果、TF/公共坐标转换和路径执行契约。YOLO 不应直接把人物当前位置当飞行终点，上层应先生成可行观察点。

## 测试入口

```powershell
docker exec robocup-single-uav bash -lc 'source /workspace/scripts/container/single_uav_env.sh >/dev/null; python3 /workspace/tests/test_uav_motion.py'
docker exec robocup-single-uav bash -lc 'source /workspace/scripts/container/single_uav_env.sh >/dev/null; python3 /workspace/tests/sitl_motion_safety.py cancel'
```

实飞安全用例依次运行 `cancel`、`pose_stale`、`duplicate`；每轮都会真实起飞，开始前应无其他控制节点。断流用例只中断测试转发给本控制器的位姿，不停止 Gazebo/PX4。正常飞行日志用 `python3 /workspace/tests/check_uav_flight_log.py /workspace/logs/flight/<该轮>/events.jsonl` 验收。

本轮没有 YOLO、人物识别、地图、A*、静态或动态避障，也没有六机。空旷场景航点通过只证明该单机运动流程。
