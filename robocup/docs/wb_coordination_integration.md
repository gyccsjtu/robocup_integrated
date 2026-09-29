# WorkBuddy 协同接入记录 — 2026-09-10

对应 Codex 交付：`coordination_interface_v0.md`（schema v1，已冻结）、
`coordination_core_handoff.md`。本文只记录**接入层**的完成与剩余，不改核心语义。

## 1. 独立复验（先跑，再接入）

| 项 | 命令 | 结果 |
|---|---|---|
| 核心回归 | `python tests/test_coordination.py` | **34/34 通过** |
| 六机运动学示例 | `python examples/coordination_demo.py --out <新目录>` | `KINEMATIC_ONLY`，最小间距 4.0m，**verdict PASS** |
| 全部非 ROS 主机测试 | 逐个 `python tests/test_*.py`（跳过 `test_uav_motion.py`） | **10/10 通过** |
| 采集器 + Codex 判定插件 | `collect_runs.py --verdict-module scripts/coordination_verdict.py` | coord 128 事件，**PASS**，`missing_evidence=0` |

> 复验一律用隔离 venv：`C:\Users\pc\.workbuddy\binaries\python\envs\default`（已装 PyYAML）。

## 2. 已完成的接入工作

### 2.1 catkin 子包声明（交接清单 §WorkBuddy 第 1 条）
`src/robocup_navigation/setup.py` 的 `packages` 现显式列出
`robocup_navigation.coordination` 与 `robocup_navigation.coordination_adapter`，
否则 install 模式不装子包。

### 2.2 采集器协调概要（交接清单 §WorkBuddy 第 4 条）
`scripts/vm/collect_runs.py` 新增 `summarize_coord()`：统计 `coord_events.jsonl`
的事件数、是否含 `RUN_STARTED`/`RUN_FINISHED`、`outcome` 与 `failures`。
纯协调日志不再被误报为 `missing evidence`；`--verdict-module` 插件接口保持原样，
**未改 Codex 的判定逻辑**。

### 2.3 协同适配层（WB 拥有，不碰核心）
`src/robocup_navigation/src/robocup_navigation/coordination_adapter/`

| 文件 | 职责 | 对应接口条款 |
|---|---|---|
| `bus.py` | `InputSeq`（每生产者 run 内单调 seq）、`CoreBus`（单写者前门，输出信封透传）、`GateFanout`（**全量**输出流广播到每个 gate） | §1 公共格式；§执行器握手 #1/#2/#3 |
| `monitor.py` | `PhysicalMonitor`：覆盖窗口、最大间隔、最小间距、碰撞数、到点误差、最大无进展；产出 `RUN_FINISHED.monitor_report` | §4 验收事件流（独立监测） |
| `node.py` | ROS 包装（**dev harness**）：调用回调只入队，单定时器驱动核心；发布 `/coordination/commands`、`/coordination/events` | §5 接入清单 |

**刻意不做**：不解释放行/净空/超时/路线版本语义；不把 `/gazebo/model_states`
真值喂给核心；不把单机 `~goal` 当作 `ROUTE_GRANT`。

### 2.4 主机测试
`tests/test_coordination_adapter.py`：**9/9 通过**，覆盖
- seq 每源单调；
- 核心输出信封**未被改写**（`source_id=coordinator_commands`、seq 连续）；
- 全量流广播 → 双机都拿到 grant；
- **按机过滤会触发 `GATE_SEQUENCE`**（证明"必须先全量再过滤"的契约）；
- `check()` 在租约过期时要求停机；
- 监测器：报告字段与接口一致、间距/碰撞检出、间隔超预算 → 覆盖失败、到点误差与无进展。

## 3. 运行命令

```bash
# 主机：核心 + 适配层单测（无 ROS）
export PYTHONPATH=<ws>/src/robocup_navigation/src
python tests/test_coordination.py
python tests/test_coordination_adapter.py
python examples/coordination_demo.py --out /tmp/coord_demo   # 目录必须全新

# 采集 + 判定
python scripts/vm/collect_runs.py --logs-root <logs> \
    --glob "scenario_matrix/*/" --verdict-module scripts/coordination_verdict.py

# 场景矩阵（批量编排）
bash scripts/vm/run_matrix.sh config/scenarios/headon_matrix.spec
```

## 4. 尚未完成（**不声称六机 E2E 通过**）

| 项 | 状态 | 说明 |
|---|---|---|
| 官方六机 launch | **TODO** | `start_multi_uav.sh` 第 2 步留标记；`node.py` 是 dev harness |
| 逐机隔离 | 未完成 | 控制器 flock、ROS 节点名、**控制话题冲突检测**、TF、出生变换、MAVLink ID/端口、模型允许列表 |
| 执行器状态回报 | 未完成 | `ACCEPTED→STARTED→COMPLETED` 需由单机执行器真实回报，待与单机 node 对接 |
| 独立监测器覆盖着陆全过程 | 部分 | 类已就绪（`PhysicalMonitor`），尚未接入真实遥测 |
| 六机 PX4/Gazebo 验收 | **无** | 无 30 分钟稳定性、窄道吞吐量、官方赛规符合性证明 |

## 5. 回报 Codex 的协议问题 / 待明确

1. **等待点候选的生成接口**：接口要求 `waiting_points` 带 `clearance_m / tracking_bound_m /
   connector_safe / outside_bottleneck` 证明。WB 从单机地图产生候选集合，但
   **这些字段由谁计算**（WB 算净空/连接段，还是核心给规则）需明确，否则我无法安全生成。
2. **`map_revision` 的权威来源**：谁维护该计数器、何时递增（内容变化 vs recenter）。
   交接文档 §Python 调用约定已警告不得用 `map_version` 冒充，需给出 WB 侧计数器契约。
3. **执行器状态映射**：单机节点现有 phase（TAKEOFF/跟随/到点/降落）到
   `COMMAND_ACK.status` 的映射表由谁定义；`STOPPED`/`COMPLETED` 的判据（实测停止）需与单机节点
   的到位/落地判据对齐。
4. **`Coordinator.snapshot()` 结构**：node 里我避免依赖内部结构（用本地 positions 占位），
   若要用于监测，请确认可用字段与深拷贝语义。

> 出现无法对接的协议点，我会附最小消息序列与错误回报，**不自行改放行/释放/超时语义**。
