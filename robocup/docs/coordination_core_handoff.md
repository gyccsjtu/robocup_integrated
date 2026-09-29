# Codex 核心首版交接 — 2026-09-10

状态：纯 Python 核心首版已实现，可交给 WorkBuddy 做接入与环境验证。
协议：`coordination_interface_v0.md` / schema v1；未修改消息字段或版本。
这是一版保守的任务导航协调器，不是完整比赛搜索/视觉跟踪/报告算法，也不是六机 SITL 已通过。

## 交付文件

| 文件（相对 robocup_ws） | 职责 |
|---|---|
| src/robocup_navigation/src/robocup_navigation/coordination/core.py | 单写者状态机、最小代价分配、目标锁、通行预约、隔离与接替 |
| .../coordination/geometry.py | 连续三维线段距离、整条路线冲突、六机精确匹配 |
| .../coordination/protocol.py | schema v1 输入验证 |
| .../coordination/executor.py | ExecutorGate，本地旧指令/版本/租约/地图变化保护 |
| .../coordination/verdict.py | 同一份完整事件流验收判定 |
| scripts/coordination_verdict.py | WorkBuddy collect_runs.py 插件入口 |
| tests/test_coordination.py | 34 项核心、协议、执行端、验收回归 |
| examples/coordination_demo.py | 六机运动学运行示例（无 ROS） |

## 算法选择及明确限制

1. 任务配置给定 task_id、唯一 target_id、无人机需到达的世界 xyz。这里的 xyz 是飞行任务目标，不应直接将地面 actor 坐标当作飞行目标。搜索发现、观测点生成、视觉融合和 15 秒报告编排不是本模块已实现功能。
2. 上游规划器提供各机到各任务的 ROUTE_OFFER。核心对可行边做最大匹配数量优先、其次最小预计时间的精确 DP；最多六架飞机，无第三方依赖。不是按直线距离猜路线。
3. 本版预约**整条路线的空间包络**，包络间距为 min_separation_m + 2*tracking_bound_m。连续三维线段距离包含迎面、交叉、端点接触。时间字段是预约开始/预计结束信息，**不按预计结束自动释放，也不做时空交错穿行优化**。
4. 无冲突路线并行；相交路线串行。空闲飞机当前位置也构成障碍。有冲突的候选保持待分配，后续 TICK 再试。
5. 路线不变时不会主动生成新绕行路径。若迎面对换目标且双方互堵，可能无路可授；到 deadlock_timeout_s 输出 DEADLOCK 并停止新授权，不能声称已实现所有拓扑下的自动脱困。上游应提供新的安全绕行方案；本版安全退出优先于冒险通行。
6. 等待点函数只筛选带证明的候选，不自动命令飞机移到等待点。执行器在等待授权期间必须已经处于可安全保持的位置；不能将 TASK_ASSIGN 当作移动授权。
7. 失联/租约到期/地图 revision 变化撤销路线；存在 REVOKING/QUARANTINED 时暂停新飞行授权。旧预约保留。失败接替只在停止 ACK + 后续新鲜位置 + 验证器 RESOURCE_CLEAR 证明完整离开后执行；确认已落地的故障机可退出本 run，新任务分给其他飞机。仅失联的飞机不会假定已消失。
8. 未提供状态的飞机阻止新授权。退出机的位置仍作为静态障碍保留。退出机本 run 不自动恢复接单。
9. 单个逻辑授权者，无分布式一致性或热重启恢复；重启必须停止授权并执行经确认的复位、新 run_id。
10. 所有地图/静态安全证明由注册规划器提供；核心不读取 Gazebo 真值、不生成占据栅格。clearance_m 必须是扣除机体等预算后的可用净空；tracking_bound_m 必须包含执行误差、定位误差和停止期间可能的额外位移。上游无法保证该包络时不能宣称此协调器能保证物理避碰。

## Python 调用约定

Python 3.8+ 标准库。开发时将 `src/robocup_navigation/src` 加入 PYTHONPATH。

```python
from robocup_navigation.coordination import Coordinator, CoordinationError
from robocup_navigation.coordination.executor import ExecutorGate

core = Coordinator(run_id, fleet_ids, tasks, limits, roles, test_case_id,
                   start_sim_s=sim_now, start_wall_s=monotonic_now)
core.receive(message, received_wall_s=monotonic_now)
commands = core.drain_outputs()
events = core.drain_events()
```

- 使用单线程事件队列串行调用；不要让 ROS 回调并发写 core。
- receive 返回本次新增输出的副本；drain_outputs 取出所有未取走输出。**二选一发送，不能两种都重复发**。推荐忽略 receive 返回值，统一 drain。
- 启动后先 drain_events 保存 RUN_STARTED。接收错误时也 drain 已产生的保护命令/事件；格式/身份/序号错误不消耗输入序号，业务语义拒绝可能已消耗合法序号，调用方不能修改同一 seq 后重试。
- 输入 seq 按生产者在整个 run 内递增，不能每个 topic 独立计数；每机状态和 ACK 共用 seq。上游重发同一消息可幂等处理，不能跳号。
- roles 是明确的 source_id→输入 kind 白名单。VEHICLE_STATE/COMMAND_ACK 的 source_id 必须等于 uav_id。ROUTE_OFFER、RESOURCE_CLEAR 分别来自受信任规划器、清空验证器；身份认证由适配器负责。
- TARGET_REPORT 消费上游已确认目标（首版 confidence=1 才更新）；重复 observation_id 不计新观察。移动到达目标超过容差会撤销旧路线等待安全重新分配，不做连续追踪控制。
- 地图更新到执行器本地时立刻调用 gate.check，不等协调器往返；后续 ROUTE_OFFER 必须基于当前 revision。不要用只在 recenter 递增的 map_version 冒充内容 revision。
- 所有时间均显式传入。适配器发送 TICK，且无消息时继续调用 ExecutorGate.check；只在收到网络包时才检查失联是不完整实现。
- 库的公开 snapshot 是深拷贝。不要直接改 core.tasks/vehicles/reservations 等内部字典。

### 配置（不设隐式安全默认值）

limits 必须显式提供：min_separation_m、arrival_tolerance_m、mission_timeout_s、deadlock_timeout_s、max_monitor_gap_s、state_timeout_s、lease_s、stop_speed_mps、required_clearance_m、tracking_bound_m、position_tolerance_m、nominal_speed_mps。
所有值有限正数；position_tolerance_m <= tracking_bound_m。名义速度仅用于分配代价/预计时间，不是飞行速度控制器。示例值只用于运动学测试，不代表比赛/PX4 安全参数。

### 执行器握手

1. 为每机建 ExecutorGate(run_id, uav_id, start_tolerance_m, heartbeat_timeout_s)。start_tolerance_m 使用相同 position_tolerance_m。
2. 将**完整 coordinator_commands 输出流**按 seq 广播给每个 gate，包括别的飞机的消息。gate 自行筛选飞机；若先按飞机过滤再送 gate，会因 seq 缺号被拒绝。
3. gate.receive(msg, sim_s, local_monotonic_s, map_revision, world_xyz) 返回可执行动作；返回 None 不触发动作。实际飞机收到 ROUTE_GRANT 后还需检查自身飞行条件。必须执行授权 points，不能再让旧 ~goal 通路自行 A* 生成另一条路线。
4. 必须真实回报 ACCEPTED→STARTED→COMPLETED；末点到达且实测停止后才发送 COMPLETED。STOPPED/COMPLETED 前先发同 command_id/epoch/route_version 的停止状态，不能伪造停止 ACK。
5. gate.check 返回 True 或 receive 抛异常：执行器实施已有经验证的停止/降落策略。HOLD/CANCEL/LAND 请求不等于已经停止，拒绝动作时必须反馈 REJECTED，不能自行释放预约。
6. 实际停止后可调用 gate.confirm_stopped，关闭旧 epoch；该函数不向 core 自动发送 ACK，也不释放通道。收到 CANCEL_REQUEST 时状态/ACK 要使用它的 command_id。
7. 完成任务后由执行器实施经验证的降落/离开策略，保留预约直到完整离开三维包络；单纯在终点悬停不能 RESOURCE_CLEAR。离开动作本身也必须与其他飞机安全约束一致，不能凭此绕过核心新增冲突空间。
8. RESOURCE_CLEAR 必须来自验证器，匹配预约 epoch/version，且引用停止 ACK 后的新状态；位置要完整在旧路线包络外。任务/预约被释放后才允许重分配。

## WorkBuddy 需要完成的工程工作

- 在 catkin `setup.py` 的 packages 中加入 `robocup_navigation.coordination`，否则 install 模式可能不安装子包。
- 实现 ROS 消息编解码/传输、单写者队列、规划器候选与地图 revision 接口、执行器 gate、独立物理监测器。
- 继续补齐进程锁、TF、出生变换、MAVLink ID、控制话题冲突检查等逐机隔离；启动 TODO 不算六机验收通过。
- `collect_runs.py --verdict-module scripts/coordination_verdict.py` 已实测能消费 verdict。现采集器概要仍只读旧 events.jsonl，会对纯 coord 日志显示 n_events=0 / missing_events=1；WB 增加 coord_events.jsonl 概要统计，**不要改 Codex 的判定逻辑**。
- 独立监测器要覆盖整个 run，包括着陆过程；碰撞、间距和到点误差不能从“已发 ROUTE_GRANT”推断。
- 出现无法对接的协议点，附最小消息序列与错误回报给 Codex，不自行改放行/释放/超时语义。

## 已运行验证

- `python tests/test_coordination.py`：34 项通过，包括六机并行、交叉排队、目标独占、失联保留锁、迟到心跳不复活授权、停止但未离开不释放、确认故障降落后接替、旧指令与缺序拒绝、地图变更撤销、缺证 ABSTAIN、碰撞优先 FAIL。
- 随机几何检查：150 对三维线段，每对验证距离对称性、端点逆序一致性及 121 组采样上界。
- 全部 10 个不依赖 ROS 的 test_*.py 文件通过。test_uav_motion.py 依赖 ROS，不在本轮主机验收范围。
- `python examples/coordination_demo.py --out <全新目录>`：六机纯运动学运行 20 仿真秒，最小间距 4m，六任务完成、确认离开预约后 PASS。
- 示例日志：`D:/a/.robocup/tmp/coordination_core_demo_20260910/coord_events.jsonl`。
- 真实采集器插件汇总：`D:/a/.robocup/tmp/coordination_verdict_manifest.json`。
- 尚未跑六机 PX4/Gazebo；没有 30 分钟稳定性、复杂窄道吞吐量或官方赛规符合性的证明。

交接验收顺序：先重跑纯逻辑与运动学示例→接两机交叉/旧指令/失联→再六机独立路线→增加动态障碍及任务接替。核心若 FAIL/ABSTAIN，不通过调宽安全值或伪造监测报告变成 PASS。
