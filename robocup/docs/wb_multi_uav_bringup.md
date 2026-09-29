# 六机 Gazebo/SITL 打通记录（WorkBuddy 接入层）

DEV HARNESS（接口 v0 §5）：不是官方比赛 launch，也不代表协同已验收。

## 1. 状态

| 项 | 结果 |
|---|---|
| 六机 SITL 启动 | ✅ 6/6 MAVROS `connected: True` |
| Gazebo 模型 | ✅ `iris1..iris6`，位于 -20/-12/-4/4/12/20 m |
| 仿真链路配对 | ✅ TCP 4561–4566 一一对应，无串线 |
| 核心接收消息 | ✅ 无 schema 拒绝 |
| 执行器发出 ROUTE_OFFER | ✅ `clearance=25m, static_safe=true, grid_safe=true`（证据来自配置源） |
| **OFFBOARD / 起飞** | ❌ **PX4 拒绝切 OFFBOARD** → 未解锁、未起飞 |
| **协同授权** | ❌ 因此核心正确地**不放行**，报 `SAFETY_VIOLATION/DEADLOCK` |

## 2. 怎么跑

```bash
# 1) 起六机 SITL（headless）
export ROBOCUP_WORKSPACE=$HOME/robocup/robocup_ws
ROBOCUP_MU_SIM=1 ROBOCUP_MU_NODES=0 bash scripts/vm/start_multi_uav.sh

# 2) 起协同栈（协调器 + N 执行器 + 感知替身）
source /opt/ros/noetic/setup.bash
source scripts/container/single_uav_env.sh
python3 scripts/vm/start_coordination_sim.py --out logs/coord_run_X --duration 150 [--limit N]
```

## 3. 三个环境坑（都已修，别再踩）

1. **CRLF 杀死 `jinja_gen.py`**：shebang 变成 `python3\r` → exit 127。
   根因是 Windows 侧 `core.autocrlf=true`；已加 `.gitattributes` 锁死 LF。
2. **MAVLink 端口方案是打过补丁的**（`ROMFS/.../px4-rc.mavlink`）：
   PX4 绑 `34580+instance`、发往 `24540+instance`。
   PX4 自带 `multi_uav_mavros_sitl.launch` 用 14540→14580，**永远连不上**。
   → 改用 `gen_multi_uav_launch.py` 从 `fleet.yaml` 生成。
3. **SDF 端口必须 `base + px4_instance`**（不是 `base+(n-1)`）：
   PX4 侧是 `px4-simulator ... -c 4560+instance`。错一位会**串线**
   （PX4 连到隔壁模型的插件，MAVROS 仍显示 connected，极隐蔽）或卡死 rcS。

## 4. 配置约束（来自核心，不是我定的）

- `position_tolerance_m <= tracking_bound_m`，否则 `CONFIG_START_OUTSIDE_TRACKING_BOUND`。
- 出生间距必须 > `min_separation_m + 2*tracking_bound_m`，否则六机互相"贴脸"，
  任何路线都判冲突 → 永不放行。
- 拿到 `ROUTE_GRANT` 需要 `static_safe` **且** `grid_safe`、净空/跟踪上界达标、
  且飞机实测速度 ≈ 0（停稳）。**未知证明不得填 `true`** —— 执行器在证据未知时
  **直接不发 offer**（fail-closed）。
- `grid_source=static_substitute` 是**开发替身**：没有动态感知，用静态图代替，
  仅用于验证协同，**不得用于比赛**。

## 5. 打通 OFFBOARD：三个接入层 bug（都已修）

1. **SITL 无遥控器 → RC 丢失保护闩锁 → PX4 拒绝 OFFBOARD。**
   PX4 在无 RC 时约 0.5 s 判定丢控，按 `NAV_RCL_ACT` 进入 LOITER/RTL；
   **只要 failsafe 处于闩锁状态，OFFBOARD 一律被拒**（命令发出、模式不变）。
   修：`COM_RCL_EXCEPT=7`（豁免 stick/switch/mode）+ `NAV_RCL_ACT=0`。
   只把 `COM_RC_LOSS_T` 调大**无效**——必须让飞机根本不进 failsafe。
2. **核心被 `start_sim_s=0` 播种，而仿真已跑到 ~2400 s。**
   于是第一个 TICK 就 `now - progress > deadlock_timeout_s` → `halted`
   → **永不调度**（现象：只有 `RUN_STARTED` + `DEADLOCK`，0 次授权）。
   修：节点用当前 `rospy.Time.now()` 播种核心。
3. **监测器采样了零值位置占位。**
   协调器 `_positions` 初值 `[0,0,0]`，在六架全部上报前就开始采样 →
   六架"重合"原点 → `min_separation=0.0`、82 次假碰撞。
   修：全编队上报后才采样，**占位值绝不当测量值**。

配套修正：MAVROS setpoint 消息头必须用 MAVROS 的本地帧（默认 `map`），
不是协议里的 `world_enu`（否则消息被静默丢弃）；offer 首点取**当前实际位置**
（否则核心跟踪上界检查立刻判 `AUTHORIZATION_CONFLICT`）；加优雅收尾
（SIGTERM/SIGINT → `bus.finish()`）才会产生 `RUN_FINISHED`。

## 6. 最新结果（六机平行航线，真实 verdict）

| 项 | 值 |
|---|---|
| 任务分配 / 航线授权 | **6 / 6** |
| 六架是否飞抵目标 | 是 |
| collision_count | **0** |
| min_separation_m | **6.6 ~ 6.9**（要求 3.0） |
| Codex verdict | **FAIL / ARRIVAL_ERROR**（2.5 m vs 容差 1.0 m） |

**结论**：协同链路（分配 → 授权 → 飞行 → 到点）已在真实 Gazebo 六机上跑通，
且无碰撞。当前 FAIL 原因是我执行器的**终端到位精度**，不是协同算法本身。

## 7. 下一步

- 提升终端精度：减速进近 / 速度前馈 / 更紧的末段控制（当前靠 `waypoint_tolerance_m`）。
- 之后再加**交叉航线**场景，验证冲突排队与防死锁（当前是互不干扰的平行航线）。
- dev 容差取值可与 Codex 确认：这些是开发值，不是比赛安全参数。
