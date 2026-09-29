# WB 执行器侧接入设计 + 协议问题自带答案（2026-09-10）

只出设计，**未改核心**。对应 `coordination_interface_v0.md` 执行器握手 #1–#8。

## 1. 单机导航节点如何成为执行器

现状：`uav_navigation_node.py` 有 `~goal` 外部目标通路，收到后自行 A* 并飞。
**接口 §5 明确：`~goal` 不是 `ROUTE_GRANT`，不得未经授权校验直接放飞。**

改造原则（WB 接入层，不改核心语义）：
1. `~goal` 只作为**任务请求**（转成上游 `TARGET_REPORT`/任务候选），**不再直接规划飞行路线**。
2. 唯一可执行的飞行授权 = `ExecutorGate.receive()` 返回的 `ROUTE_GRANT`；执行器**必须逐点执行授权 points**，不允许再自行 A* 生成另一条路线。
3. 每周期调用 `gate.check(sim, wall, map_revision)`（**不是只在收到网络包时**）；返回 True 即实施已验证的停止/降落策略。
4. 广播契约：**完整输出流**（含别机消息）按 seq 送进每个 gate（`GateFanout.deliver` 已实现）；禁止先按机过滤。

## 2. 状态回报映射（phase → COMMAND_ACK.status）

| 单机节点状态/判据 | 回报 | 说明 |
|---|---|---|
| 收到 ROUTE_GRANT 且本机飞行条件满足 | `ACCEPTED` | ACCEPTED ≠ 已执行 |
| 开始沿授权 points 移动（离开当前点） | `STARTED` | |
| **末点**：距末点 ≤ `position_tolerance_m` 且**实测速度** ≤ `stop_speed_mps` 持续 N 周期 | `COMPLETED` | 单纯到达但仍在漂移不算；悬停不算完成 |
| 收到 HOLD/LAND/CANCEL 请求并经实测停止 | `STOPPED` | 必须先发同 `command_id/epoch/route_version` 的停止状态，**不得伪造** |
| 无法证明停止/降落策略可行 | `REJECTED` | 保留冲突空间，不得自行释放预约 |
| 落地完成 | （状态字段）`mode=LANDED`、`armed=false` | |

`STOPPED` 只证明"匹配命令已停止"，**不证明已离开通道**。

## 3. RESOURCE_CLEAR 的时机（关键，别做错）

1. 任务完成/取消后，执行器实施**经验证的降落/离开策略**；
2. 飞机（含制动/误差包络 `tracking_bound_m`）**完整离开旧路线三维包络**后，
3. 由**验证器**（不是执行器）提交 `RESOURCE_CLEAR`，带匹配的 `epoch/route_version` 与**停止 ACK 之后的新鲜位置**。

> 单纯在终点悬停 **不能** RESOURCE_CLEAR。离开动作本身也必须满足与其他机的安全约束（不能借"离开"绕过核心新增冲突空间）。

## 4. 回报 Codex 的 4 个问题 —— WB 侧自带答案（请确认或纠正）

| # | 问题 | WB 的方案（默认按此实施） |
|---|---|---|
| 1 | `waiting_points` 的证明字段谁算 | **WB 算**：只有 WB 有本地图与几何。由 `route_runtime`/`local_grid` 产出候选并填 `clearance_m / tracking_bound_m / connector_safe / outside_bottleneck`（含地图版本、有效时间）。**核心只做多机预约冲突复核**——与 AGENTS.md 一致。 |
| 2 | `map_revision` 权威来源 | **WB 维护内容 revision**：`LocalOccupancyGrid.content_revision` 已是"内容变化才 +1"的计数器，直接作为 `map_revision`；**禁止**用 recenter 的 `map_version` 冒充。到执行器本地即刻用新 revision 调 `gate.check`，后续 `ROUTE_OFFER` 必须基于当前 revision。 |
| 3 | 执行器状态映射表 | 见本文 §2/§3（`STOPPED` 判据 = 实测速度≤`stop_speed_mps` 且位移在 `position_tolerance_m` 内持续 N 周期；`COMPLETED` = 末点距离≤`position_tolerance_m` 且实测停止）。 |
| 4 | `Coordinator.snapshot()` 字段 | **已由 Codex 澄清**：仅 `tasks`、`reservations`、`halted`、`failures` 四个顶层字段，**无位置字段**。显示改用 `PositionCache`（带时间戳 + 过期标记）；验收监测必须用独立测量，不得从 snapshot/授权路线反推。 |

## 5. 逐机隔离剩余清单（未完成，不得记为六机 E2E）

控制器 flock、ROS 节点名唯一化、**控制话题冲突检测**、TF（map/odom/base_link 逐机）、出生变换、MAVLink ID/端口、Gazebo 模型允许列表。
`start_multi_uav.sh` 第 2 步仍是 TODO（等官方 launch）。

## 6. 已可运行的证据（主机，非 ROS）

`python scripts/coordination_harness.py --out <新目录>`：六机运动学 + 三场景
- `parallel_six` → **PASS**（`ACCEPTANCE_PASSED`，最小间距 4.0m）
- `link_loss`（1 机失联）→ **FAIL `TASKS_OR_LANDING_INCOMPLETE`**（失联机不被当作已消失）
- `stale_command`（注入旧授权）→ 旧授权被 gate 拒（`GATE_ROUTE_NOT_AUTHORIZED`）

**KINEMATIC ONLY：不是 PX4/Gazebo 证据，也不代表六机 E2E 通过。**

## 7. 澄清后的修订（2026-09-10 晚，依据 `coordination_adapter_clarifications.md`）

§4 的"待确认问题"已全部由 Codex 明确；以下是按规则落地的实现，**规则由 Codex 定，WB 只实现**：

| 规则 | 落地实现（`coordination_adapter/`） |
|---|---|
| 等待点：Codex 定规则，WB 算候选 + 证明；**证明不足返回空集合** | `waiting_points.py`：净空取等待点 **及整条连接段** 的采样下界（扣机体预算；**FREE ≠ 足够净空**）；`tracking_bound_m` 必须来自**执行器配置**（未知或超预算 → 不出候选）；`connector_safe` 在同一快照上整段检查；`outside_bottleneck` 需**通道几何**（无几何 → 不出候选）。绝不填常量 `true`，绝不猜安全点。**核心只有候选筛选，没有自动让行授权**（初期可传 `waiting_points=[]`）。 |
| `map_revision`：每机地图提供者维护；**原子**读；内容/recenter/清空递增 | `map_revision.py`：`snapshot()` 返回 `(cells_copy, revision)` 原子对，禁止分开读计数器；FREE/OCCUPIED/UNKNOWN 互转、`clear`、`recenter` 递增；同内容重写不变。已测 UNKNOWN 写入 / 清空 / recenter / **并发融合**。不使用只在 recenter 递增的 `map_version`。 |
| ACK 按**实际执行结果**，不是 phase 机械转换；**核心只查最新速度** | `stop_check.py`：`SustainedStop` 保留**连续停稳**窗口——sim+墙钟双新鲜、三维速度范数 ≤ `stop_speed_mps`、mode ∈ {HOLDING, LANDED, IDLE}、命令标识（command_id/epoch/route_version）匹配、漂移 > `position_tolerance_m` 则**重新计时**、**仿真暂停不累计**。`COMPLETED` 另需三维到点 ≤ `arrival_tolerance_m`（不是中间航点、不是字符串）。停止/完成 **不自动释放预约**。 |
| `snapshot()` 无位置；显示须标过期；监测须**独立测量** | `position_cache.py`：缺失 → `None`（不填 0、不外推），返回 `age_s` 与 `stale`；**仅供显示**，验收监测另走独立测量流。 |

> 铁律：**未知的安全证明仍阻止放行**；不为打通接口填默认 `true`。

### 7.1 新组件主机测试
`tests/test_coordination_adapter_components.py`（**14/14**）：revision 内容/清空/recenter/并发；候选在 UNKNOWN/OCCUPIED/无界/无通道几何/过期时**一律拒绝**；连续停稳（单帧不算、运动重置、仿真暂停不计时、漂移重新计时、命令标识不匹配拒绝）；到点用三维容差；位置缓存缺失/过期。
