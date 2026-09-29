# schema v1 接入澄清（2026-09-10）

本文解释现有接口与实现，不增加消息字段或改变授权规则。若需要新增字段或放宽规则，由 Codex 升版。WorkBuddy 负责实现以下适配；本文件不表示 ROS 接入已完成。

## 1. 等待点证明

Codex 定义规则；WorkBuddy 的地图/规划适配器计算候选和证明，核心不从栅格生成候选。

- `clearance_m`：等待点及当前位姿到等待点连接段的保守可用净空下界，扣除机体等已声明预算；必须达到 `limits.required_clearance_m`。不能把 FREE 格等同于有足够净空。现有 SafetyMap.clearance 只扣机体半径，不自动扣所有误差预算；使用时要记录预算来源，避免漏扣或重复扣。
- `tracking_bound_m`：定位误差、跟踪误差、停止额外位移的保守包络上界，由执行器配置及验证结果提供，地图适配器引用；不得填写当前瞬时跟踪误差。不得超过 `limits.tracking_bound_m`。未知上界时不能出安全证明。
- `connector_safe`：从该次规划起点到候选点的整个连接段，在同一地图快照上通过机体及误差包络的连续/栅格检查；只检查终点不成立。执行前起点或地图变化须重新验证。
- `outside_bottleneck`：候选点的完整停留包络不侵入规划器识别的窄通道/出入口；由地图适配器依据通道几何证明，不能从单格 FREE 推断。没有通道几何或无法证明时填 false 或不提供候选。
- `static_safe`、`grid_safe` 必须是实际检查结果，不能为通过接口填常量 true。坐标统一 world_enu；候选 map_revision 与当前状态相同，valid_until_sim_s 严格大于当前仿真时刻。

核心还检查候选连接段与其他飞机位置、未释放预约的冲突。当前 `safe_waiting_points()` 仅筛选，不自动发让行授权。初期可传 `waiting_points=[]`；这意味着没有可用让行点，不意味着可以随意找点移动。缺少经验证的误差预算仍是物理安全验证前置，示例参数不能代替。

## 2. map_revision

权威来源是每架飞机的地图提供者（WorkBuddy 接入），核心只消费。该机的状态、路线证明、等待点、执行器 gate 必须引用同一计数器及快照。

任何影响安全证明的栅格内容变化（包括 FREE/OCCUPIED/UNKNOWN 互转）、窗口 recenter/reset 或地图几何变化，必须使新快照 revision 增大。同一内容重复写入可以保持不变；仅更新观测时间不等于内容变化，但证明有效期仍需独立检查。安全配置不得在原授权下悄悄变更。

地图数据与 revision 必须原子读取/发布，不得先读地图再无锁读取计数器。`map_version` 只随 recenter 增长，不能直接使用。现有 `content_revision` 是接入候选，不代表原子快照及所有修改路径已经验证；WorkBuddy 必须覆盖 UNKNOWN 写入、清空、recenter、融合并发的测试。

run 内单调递增，不跨飞机比较。地图进程重启不能从零复用旧授权；停止旧执行并经复位开启新 run。当前 v1 在 revision 改变时撤销旧路线，可能保守频繁停车；不能以只计 recenter 来规避。优化为路线局部重验证需另行设计版本。

## 3. phase、状态与 ACK

ACK 是具体 command_id 的执行事实，不是 phase 改名。

|事实|ACK|
|---|---|
|gate 校验通过，执行器接收并绑定该命令|ACCEPTED|
|确实开始执行该命令|STARTED|
|中止/保持/取消后，实测满足停止条件|STOPPED|
|ROUTE_GRANT 最终目标到达且实测停止|COMPLETED|
|不能接受或不能执行，带明确 reason|REJECTED|

允许顺序：初始 ACCEPTED/REJECTED；ACCEPTED 后 STARTED/STOPPED/REJECTED；STARTED 后 STOPPED/COMPLETED/REJECTED。HOLD/CANCEL/LAND 不发送 COMPLETED。新 CANCEL_REQUEST 使用新命令 ID，不能继续确认被取代的飞行命令。

VEHICLE_STATE.mode 的适配原则：MOVE 为 EXECUTING；中间航点 HOVER 仍属于路线执行，不报最终 COMPLETED；保持位置且停止后为 HOLDING；DESCEND/AUTO_LAND 为 LANDING；LANDED 要有新鲜着地及解锁状态证据，DONE 本身不证明着地或成功。REPLAN 按实际运动/保持事实报告；仅进入 REPLAN 不证明停止。初始化缺少位姿时不伪造 IDLE 新鲜状态。

STOPPED/COMPLETED 前先发送真实 VEHICLE_STATE：状态在仿真与接收墙钟两个尺度均新鲜，三维速度范数 <= limits.stop_speed_mps，mode 属于 HOLDING/LANDED/IDLE，active_command_id、observed_epoch、route_version 均匹配。接入端保留已有 settle_time_s 的连续停稳检查，不能以单帧低速度替代。**当前核心只检查最新速度，不独立检查持续停稳时间。**

COMPLETED 另需三维世界坐标距任务最终 xyz <= limits.arrival_tolerance_m，不是中间航点 ARRIVED，也不是单机 MISSION_COMPLETE 字符串。停稳期间超限应重新计时；仿真暂停不增加停稳时长，新鲜度/时钟保护继续独立执行。

停止/完成 ACK 不释放预约。RESOURCE_CLEAR 由验证器依据停止 ACK 后的新状态证明飞机完全离开旧三维预约包络，再发送；失联、超时、终点悬停或单纯 landed 均不自动释放。

## 4. Coordinator.snapshot()

当前顶层字段只有 `tasks`、`reservations`、`halted`、`failures`，返回深拷贝。没有 `positions`、`vehicles` 或时钟字段；内部任务字典不作为新增冻结遥测 schema。

界面可用适配器实际接收并转换为 world_enu 的 positions 缓存，同时显示时间戳与过期状态。缺位置应显示缺失，不能用零值/推算值冒充测量。

验收监测应从独立测量流生成物理证据，不能从 snapshot、授权路线或 TASK_COMPLETED 反推无碰撞和实际到达。仿真测试可由独立 Gazebo 监测流提供；比赛允许的数据来源尚待核实。不要为了接监测自行给 snapshot 添加或假定字段。
