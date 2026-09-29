# 两套并行方案的对齐分析

**结论先行**：`robocup_swarm`（搜索栈）与 `robocup_navigation`（协同核心）在能力上互补、在三个点上冲突。建议以协同核心为安全权威、把搜索栈改造为执行器接入，而不是二选一或各跑各的。

状态：2026-09-16。作者：搜索栈一侧。

## 1. 现状

仓库里目前并行着两条线：

| | 协同核心（Codex / 接入层） | 搜索栈（robocup_swarm） |
|---|---|---|
| 代码位置 | `robocup_navigation/coordination/`（959 行）<br>`coordination_adapter/`（546 行） | `robocup_swarm/scripts/` |
| 解决的问题 | **机间安全**：防碰撞、时空预约、路线授权 | **任务完成**：覆盖搜索、目标消除 |
| 抽象层 | 协议层（JSON schema v1，传输无关，已冻结） | 应用层（ROS 话题直接对接） |
| 控制方式 | `setpoint_position/local`（位置控制） | `setpoint_velocity/cmd_vel`（速度控制） |
| 任务分配 | `TASK_ASSIGN` + 路线代价 | 拍卖效用函数 |
| 目标处理 | `TARGET_REPORT` + `TARGET_LOCK` | `TargetTracker`（规则 4/5） |
| 与飞行的耦合 | 执行器只接受 `ROUTE_GRANT` | agent 自主决策飞行 |

## 2. 互补性（不重叠，可直接复用）

| 能力 | 归属 | 说明 |
|---|---|---|
| 机间防碰撞、时空预约 | 协同核心 | `RESERVATION`、`min_separation_m`、`deadlock_timeout_s` |
| 路线安全证明 | 协同核心 | `ROUTE_OFFER` 的 `static_safe` / `grid_safe` / `clearance_m` |
| 失联隔离、看门狗 | 协同核心 | `QUARANTINED`、`max_monitor_gap_s` |
| 覆盖搜索、拍卖分区 | 搜索栈 | `CoverageGrid`、`TaskAllocator`、`LeaseManager` |
| A* 绕障执行 | 搜索栈 | 已实测穿墙率 0%（0/196 采样点） |
| 目标消除判定（规则 5） | 搜索栈 | 连续 15 s 确认、中断清零 |
| 目标瞬移判定（规则 4） | 搜索栈 | 被感知 30 s 未消除 |
| OFFBOARD 时序 | 搜索栈 | 已跑通（见 §4） |

**天然接口点**：协同核心的 `TARGET_REPORT`（`target_id` / `frame_id` / `xyz` / `confidence` / `observation_id`）**正好是搜索栈 agent 检测层的输出内容**。搜索栈目前发的是自建 `TargetDetection.msg`，字段可一一映射，改造量很小。

## 3. 三个冲突点

### 3.1 控制方式：位置 vs 速度

- 协同核心的 `coordination_executor.py` 发布 `setpoint_position/local`
- 搜索栈的 `swarm_agent.py` 发布 `setpoint_velocity/cmd_vel`

两者不能同时向同一个 PX4 发 setpoint。需要二选一，或明确分工（例如：授权段用位置控制、避障段用速度控制——但这会引入切换时的状态一致性问题）。

**建议**：统一到位置控制。协同核心的授权语义（`ROUTE_GRANT` 给出 `points` 序列）本身就是位置轨迹，位置控制更贴合。搜索栈的 ESDF-DWA 避障可以改为「在位置目标上叠加避障修正」，或在授权路线已保证 `grid_safe` 的前提下简化。

### 3.2 两套任务分配

- 协同核心：`TASK_ASSIGN` 的代价来自「可行路线长度 / 预计时间」，且**不存在可行路线时不分配**
- 搜索栈：拍卖效用 `U = w1·未搜索收益 − w2·飞行时间 − w3·重复率 − w4·路径风险`

接口文档明确要求「任务分配代价来自可行路线长度/预计时间及配置」，所以搜索栈的效用函数需要重新校准：把 `w1·未搜索收益` 作为**任务优先级**输入，把路线代价交给核心算。

**建议**：搜索栈的 `TaskAllocator` 降级为**任务优先级生成器**（输出「哪些格需要搜、优先级多少」），由核心决定最终分配给谁、走什么路线。

### 3.3 目标锁双重管理

接口文档明确警告：

> 分配层和协调层共同消费此状态，**不各建一套独立锁**。

- 协同核心：`TARGET_LOCK`（`HELD` / `REVOKING` / `QUARANTINED` / `RELEASED`），唯一键 `run_id/target_id`
- 搜索栈：`TargetTracker`（`confirm_since` / `evaded` / `eliminated`）

**建议**：搜索栈的 `TargetTracker` 保留「规则 4/5 的计时与判定」作为**算法实现**，但把**状态权威**交给核心的 `TARGET_LOCK`。即：搜索栈计算「这个目标是否已连续确认 15 s」，把结论作为 `TARGET_REPORT` 的 `confidence` 上报，由核心决定锁的状态迁移。

## 4. 搜索栈已经解决、核心侧仍标记为阻塞的问题

`docs/wb_multi_uav_bringup.md` 记录：

> | **OFFBOARD / 起飞** | ❌ **PX4 拒绝切 OFFBOARD** → 未解锁、未起飞 |

**根因已定位并修复**：出生点落在建筑内。

`config/multi_uav/fleet.yaml` 原来的一排出生点 `start_xy: [-20, 0]`，经校验有 **4/6 架不安全**：

```
✗ uav_2 (-12.0, 0.0)  净空 -3.40 m  撞 building_1171
✗ uav_3 ( -4.0, 0.0)  净空  1.99 m  撞 building_1169
✗ uav_4 (  4.0, 0.0)  净空 -1.49 m  撞 building_1169
✗ uav_6 ( 20.0, 0.0)  净空 -0.77 m  撞 building_1344
```

**根因链**（每一步都有实测证据）：

1. 模型卡在建筑碰撞体内 → Gazebo 物理引擎持续推挤
2. → 陀螺仪报**假角速度**（实测 0.34 rad/s，正常静止应 < 0.005）
3. → PX4 报 `WARN [ekf2] primary EKF changed N (gyro fault)`
4. → EKF 不再融合位置（`estimator_status` 的 `pos_horiz_abs_status_flag=False`）
5. → `system_status` 卡在 3(STANDBY)，不升 4(ACTIVE)
6. → **PX4 静默拒绝 OFFBOARD**：`set_mode` 返回 `mode_sent: True` 但模式不变，且**不产生任何 statustext**，所以从日志侧看不到原因

**为什么难以归因**：第 6 步的「静默」特性意味着常规排查（看 statustext、看 mode 返回值）都得不到线索。需要从陀螺仪原始值（`/uav_N/mavros/imu/data` 的 `angular_velocity`）和 EKF 标志位（`/uav_N/mavros/estimator_status`）反向定位。

**已修复**：

- `fleet.yaml` 出生点改为 `start_xy: [-18, 45]`，最小净空 14.0 m
- 新增 `scripts/vm/check_spawn_points.py`：读 fleet.yaml + 地图 metadata 校验出生点，`--find` 可搜索安全排，不安全时**退出码 1**（可用于启动前自检 / CI）
- 换地图或换随机布局后**必须重跑**该校验

**注意**：RC failsafe（`COM_RCL_EXCEPT` / `NAV_RCL_ACT`）是**另一个独立问题**，两者都存在。搜索栈用 `COM_RCL_EXCEPT=4`，接入层用 `=7`（豁免 stick/switch/mode，更彻底），建议统一到 `=7`。

## 5. 建议的整合架构

```
┌────────────────────────────────────────────────────────────┐
│  协同核心 Coordinator（安全权威，单写者状态机）              │
│   输入：VEHICLE_STATE / TARGET_REPORT / ROUTE_OFFER / ACK   │
│   输出：TASK_ASSIGN / ROUTE_GRANT / TARGET_LOCK / RESERVATION│
└────────────────────────────────────────────────────────────┘
        ↑ TARGET_REPORT（检测结果）      ↓ ROUTE_GRANT（授权路线）
┌────────────────────────────────────────────────────────────┐
│  搜索栈 swarm_agent（改造为执行器）                          │
│   保留：A* 绕障（已验证）、几何判定检测、OFFBOARD 时序         │
│   改造：接收 ROUTE_GRANT 而非自主拍卖                         │
│   产出：TARGET_REPORT / VEHICLE_STATE / COMMAND_ACK          │
└────────────────────────────────────────────────────────────┘
        ↑ 覆盖状态                        ↓ 任务优先级
┌────────────────────────────────────────────────────────────┐
│  搜索栈 TaskAllocator（降级为优先级生成器）                   │
│   输出：哪些格需要搜索、优先级多少（不决定分给谁）             │
└────────────────────────────────────────────────────────────┘
```

## 6. 核心实现审查结论（2026-09-16 补）

对 `robocup_navigation/coordination/` 做了实现层面的核查（不只是读接口文档），结论是**可以放心作为安全权威**。

### 6.1 三个关键疑虑均验证通过

| 疑虑 | 核查结果 |
|---|---|
| `_conflict` / `_schedule` 是真实现还是占位？ | **真实现**。`_conflict` 遍历所有未 `RELEASED` 的预约 + 所有在飞飞机，用 `route_distance` 比对 `min_separation_m + 2*tracking_bound_m` |
| 「不可行路线不分配」是否落到代码？ | **落到代码**。`matching()` 用位掩码 DP 做最大基数 + 最小代价匹配，注释与实现一致：「缺失的 cost 边是不可行，**绝不用直线距离替代**」 |
| 安全性是否靠自证？ | **不是**。`verdict.py` 基于**独立的 monitor report** 判定（`collision_count` / `min_separation_m` 等来自外部监测），且**证据不全时返回 `ABSTAIN`，不返回 `PASS`** |

### 6.2 `geometry.py` 的实现质量

`segment_distance()` 是完整的**线段-线段**最短距离（clamped 参数化解法），正确处理退化情形（点-点、点-线段）；`route_distance()` 取**所有线段对**的最小值 —— 这是正确的路线冲突语义，而非简单的点对点距离。

### 6.3 值得肯定的 fail-closed 设计

`_schedule()` 的第一道闸门：

```python
# An unobserved/lost aircraft could be anywhere: no new motion grants.
if any(t['status'] in ('REVOKING', 'QUARANTINED') for t in self.tasks.values()):
    return
```

**只要有一架飞机状态不明，全体停止新授权**。这是保守到位的做法。

### 6.4 测试覆盖

`tests/test_coordination.py` 共 **34 个测试，全部通过**（0.2 s 跑完）。测试覆盖的是**安全属性本身**而非正常路径：

```
test_claimed_pass_with_collision_fails          # 声称安全但实际碰撞 → 失败
test_missing_clearance_cannot_pass             # 缺净空证明 → 不能通过
test_infeasible_is_not_assigned                # 不可行 → 不分配
test_unknown_run_or_missing_reservation_never_flys  # 未知 run/缺预约 → 绝不起飞
test_out_of_order_revokes_existing_authorization    # 乱序消息 → 撤销已有授权
test_global_matching_beats_greedy              # 全局匹配优于贪心
test_crossing_and_head_on                      # 对穿 / 迎面
test_missing_and_malformed_evidence_abstains   # 证据缺失 → 弃权
```

### 6.5 对 §3.1 结论的加强

原建议「统一到位置控制」需要**加强为架构前提**，而非偏好选择：

协同核心的整个安全模型建立在 **`ROUTE_GRANT` 给出的 `points` 就是实际飞行路线**之上 —— `_conflict()` 拿这条路线去比对所有其他预约，`_schedule()` 为它建立 `RESERVATION`。**若执行器自主绕道，这套证明即失效。**

因此位置控制不只是"更贴合"，而是**该架构成立的必要条件**。

## 7. 合并方案：两套代码如何成为一个更优解

### 7.1 各模块处置

| 模块 | 现状 | 处置 | 理由 |
|---|---|---|---|
| 协同核心 `coordination/` | 959 行，34 测试全过 | **保留为安全权威** | 质量高、fail-closed、有测试 |
| `coordination_adapter/` | 546 行 | **保留** | 位置缓存、停止检测、等待点生成，核心需要 |
| `coordination_executor.py` | 已完成授权→飞行 | **保留为主执行器** | 位置控制、`(epoch, route_version)` 防覆盖、按实测报告状态 |
| `swarm_agent.py` 的 **A\*** | 穿墙率 0%（0/196） | **移到 `ROUTE_OFFER` 生成侧** | 路线仍由核心授权，我们只负责"提议一条安全路线" |
| `swarm_agent.py` 的 **目标检测** | 几何判定 + LOS 遮挡，已验证 | **保留，输出改 `TARGET_REPORT`** | 字段天然对应（`target_id`/`xyz`/`confidence`） |
| `swarm_agent.py` 的 **控制输出** | `setpoint_velocity/cmd_vel` | **改为 `setpoint_position/local`** | 见 §6.5，架构前提 |
| `swarm_agent.py` 的 **OFFBOARD 时序** | 已跑通（含"永不放弃"重试 + 出生点校验） | **移植进 executor** | 解决核心侧 `wb_multi_uav_bringup.md` 标为 ❌ 的阻塞 |
| `swarm_manager.py` | 拍卖 + 租约 | **逐步退役** | 职责被核心的 `TASK_ASSIGN` + `lease` 覆盖 |
| `TaskAllocator` | 拍卖效用函数 | **降级为优先级生成器** | 只输出"哪些格要搜、多急"，不决定分给谁 |
| `LeaseManager` | 30 s 租约 | **退役** | 核心已有 `lease_s` + `REVOKING` + `QUARANTINED` |
| `TargetTracker` | 规则 4/5 计时 | **保留算法，交出锁权威** | 计时在搜索栈，锁在核心（见 §8 问题 3） |
| `target_sim_node.py` | 6 名恐怖分子行为 | **保留** | 核心不模拟恐怖分子 |
| `CoverageGrid` | 20×10 覆盖栅格 | **保留** | 核心只管任务，不管覆盖统计 |
| `check_spawn_points.py` | 出生点校验 | **保留并强制前置** | 见 §4 |
| `scripts/cloud/*` | 云端部署 | **保留** | 与协同无关，独立可用 |

### 7.2 合并后的数据流

```
┌─────────────────────────────────────────────────────────────┐
│  Coordinator（安全权威，单写者状态机）                        │
│   VEHICLE_STATE / TARGET_REPORT / ROUTE_OFFER / ACK  →       │
│   → TASK_ASSIGN / ROUTE_GRANT / TARGET_LOCK / RESERVATION    │
└─────────────────────────────────────────────────────────────┘
      ↑ VEHICLE_STATE          ↓ ROUTE_GRANT
      ↑ TARGET_REPORT          ↓ （授权路线 points）
      ↑ ROUTE_OFFER
┌─────────────────────────────────────────────────────────────┐
│  coordination_executor + 搜索栈能力                          │
│   · 位置控制飞行（executor 原有）                             │
│   · OFFBOARD 时序（移植自 swarm_agent，含出生点校验）          │
│   · A* 绕障 → 用于生成 ROUTE_OFFER 的 points                  │
│   · 几何判定检测 → 发出 TARGET_REPORT                         │
└─────────────────────────────────────────────────────────────┘
      ↑ 覆盖状态
┌─────────────────────────────────────────────────────────────┐
│  TaskAllocator（降级为优先级生成器）                          │
│   CoverageGrid + 未搜索格 + 目标规则 4/5 计时                 │
└─────────────────────────────────────────────────────────────┘
```

### 7.3 合并的收益（相比任一单独方案）

| 能力 | 单独用核心 | 单独用搜索栈 | 合并后 |
|---|---|---|---|
| 机间防碰撞 / 预约 | ✅ | ❌ | ✅ |
| 覆盖搜索 / 拍卖分区 | ❌ | ✅ | ✅ |
| 目标规则 4/5 | ❌ | ✅ | ✅ |
| A* 绕障（已验证 0% 穿墙） | ⚠️ 有路线安全但无避障飞行 | ✅ | ✅ |
| OFFBOARD 可起飞 | ❌ 曾阻塞 | ✅ 已跑通 | ✅ |
| 出生点安全自检 | ❌ | ✅ | ✅ |
| 验收判定（防自证） | ✅ | ❌ | ✅ |
| 防覆盖 / epoch 管理 | ✅ | ❌ | ✅ |

**即：合并后两边的短板互补，无功能丢失。**

## 8. 需要共同确认的三个问题

1. **是否要求所有飞行必须经过 `ROUTE_GRANT`？**
   接口文档立场是「是」（「执行器只接受匹配当前 run、任务 epoch 和更新路线版本的授权」）。
   若确认，搜索栈的 `swarm_agent` 自主飞行逻辑需移除，A* 改用于生成 `ROUTE_OFFER`。

2. **控制方式统一到位置还是速度？**
   经 §6.5 的核查，**位置控制是该架构的必要条件**（核心的安全证明建立在"实际飞行 = 授权路线"之上），而非偏好。
   需要确认的只剩工程细节：PX4 位置控制在追踪 2 m/s 移动目标时的表现（搜索栈目前用速度控制追踪）。

3. **目标消除的权威在哪一侧？**
   规则 5 要求「连续 15 s 正确广播 ID + 坐标」。搜索栈实现了计时，核心有 `TARGET_LOCK` 且文档明确「核心不得把重复 observation 当作独立确认」。
   建议：**计时在搜索栈（发 `TARGET_REPORT`），锁在核心**。

## 7. 附：可复现的验证命令

```bash
# 出生点安全自检（换地图后必跑）
ROBOCUP_WORKSPACE=$PWD python3 scripts/vm/check_spawn_points.py

# 搜索避障验证（穿墙率、贴墙距离、高度稳定性、覆盖率）
python3 src/robocup_swarm/scripts/verify_swarm_avoidance.py --duration 40
```
