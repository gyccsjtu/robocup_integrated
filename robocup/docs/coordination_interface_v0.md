# 协同接口 v0 — schema v1

状态：冻结；日期：2026-09-10。所有者：Codex。
范围：纯 Python 协同核心的传输无关 JSON 消息、授权生命周期与验收事件。
这不是算法实现完成或六机飞行验收证明。ROS topic、官方裁判接口与 launch 不在本次冻结范围。
字段、枚举、默认行为或安全语义变更必须提升 schema_version 并附迁移说明；禁止悄悄改 v1。

## 1. 公共格式

所有消息为 JSON 对象，必须包含以下字段；未声明字段拒绝（扩展只放 data.extensions）。

| 字段 | 类型 | 约束 |
|---|---|---|
| schema_version | integer | 固定 1；bool 不算整数 |
| run_id | string | 非空，每次全系统运行唯一；不能重用 |
| source_id | string | 配置中注册的生产者；无人机逻辑 ID 为 uav_1..uav_6 |
| seq | integer | 每个 run_id/source_id 从 1 连续递增 |
| sim_s | number | 有限、非负，同一运行同一仿真时基 |
| kind | string | 下表定义的类型 |
| data | object | 对应载荷；extensions 可选 object，无控制语义 |

相同 seq 的相同消息幂等；同 seq 不同内容拒绝。旧 run、乱序消息不得改变授权。
乱序/丢失状态报告不能用来释放锁；需重同步或请求新鲜状态。日志缺序另按证据不足处理。
时间回退终止当前 run，重启使用新 run_id。仿真时间用于运动/预约；接收端单调墙钟用于失联检测，不跨主机比较墙钟值。阈值显式配置，不依赖机器速度。
全部 xyz 为有限数的三元素数组、米、统一世界 ENU，frame_id 固定 world_enu。
接入层必须完成各机 odom→world 变换，不得仅改 frame_id 标签。
生产者身份认证属于传输层，source_id 不是认证凭据。

## 2. 输入载荷

下列列出的字段均必填，允许 null 的字段显式说明。

- VEHICLE_STATE：uav_id、frame_id、xyz、velocity_xyz、health（OK/DEGRADED/FAILED）、mode（IDLE/EXECUTING/HOLDING/LANDING/LANDED）、armed（bool）、active_command_id（string 或 null）、observed_epoch（非负 integer）、route_version（非负 integer）、map_revision（非负 integer）。报告动作完成还需 COMMAND_ACK，不由 mode 推断。
- TARGET_REPORT：target_id（非空稳定 ID）、frame_id、xyz、confidence（0..1）、observation_id（非空 string）。目标融合/识别正确性由输入提供者负责；核心不得把重复 observation 当作独立确认。
- ROUTE_OFFER：offer_id、uav_id、task_id、frame_id、points（至少两个 xyz）、map_revision、valid_until_sim_s、clearance_m（非负）、tracking_bound_m（非负）、static_safe（bool）、grid_safe（bool）、waiting_points（以下候选数组）。只是候选，不是飞行授权。
- WAITING_POINT 候选对象：point_id、xyz、map_revision、valid_until_sim_s、clearance_m、tracking_bound_m、static_safe、grid_safe、connector_safe（bool）、outside_bottleneck（bool）。候选必须与 ROUTE_OFFER 同机、同世界系；valid_until_sim_s 到期即失效。
- COMMAND_ACK：uav_id、command_id、task_id（string 或 null）、epoch、route_version、status（ACCEPTED/STARTED/STOPPED/COMPLETED/REJECTED）、reason（非空 string）。ACCEPTED 不等于已执行。STOPPED 只证明匹配命令已停止，不证明已离开通道。
- RESOURCE_CLEAR：uav_id、reservation_id、epoch、route_version、frame_id、xyz、map_revision。只能由注册验证器提交，基于新鲜位置证明飞机及制动/误差包络已完整离开资源；旧 epoch 无效。
- CANCEL_TASK：task_id、reason。取消不立即释放物理占用。
- TICK：data 为空对象；推动超时检查。单调墙钟接收时间由适配器作为核心本地输入提供，不放远程消息。

任务分配代价来自可行路线长度/预计时间及配置；不存在可行路线时不分配。候选的安全距离需达到冻结的本次 run 配置，bool 声明不能替代版本、时效和距离校验。等待位置及连接段还须检查多机预约冲突。

## 3. 输出与目标锁

核心是单写者状态机；调用方按顺序输入消息，输出零个或多个消息。无 ROS、socket、sleep 或进程管理副作用。

- TASK_ASSIGN：task_id、target_id、uav_id、epoch（正 integer）、lease_until_sim_s。只授权持有任务，不授权移动。
- ROUTE_GRANT：command_id、task_id、target_id、uav_id、epoch、route_version（正 integer）、offer_id、map_revision、frame_id、points、reservation_ids（string 数组）、valid_until_sim_s。执行器只接受匹配当前 run、任务 epoch 和更新路线版本的授权。
- HOLD_REQUEST / LAND_REQUEST / CANCEL_REQUEST：command_id、uav_id、task_id（或 null）、epoch、route_version、reason。HOLD 不能凭空创造安全等待位置；执行器不能证明停止/降落策略时反馈 REJECTED，协调器保留冲突空间。
- TARGET_LOCK：task_id、target_id、owner_uav_id、epoch、state（HELD/REVOKING/QUARANTINED/RELEASED）、lease_until_sim_s、reason。分配层和协调层共同消费此状态，不各建一套独立锁。
- RESERVATION：reservation_id、uav_id、task_id、epoch、route_version、resource_id、state（RESERVED/OCCUPIED/QUARANTINED/RELEASED）、enter_after_sim_s、expected_exit_sim_s。resource_id 对应核心已验证空间包络；计划退出时间不构成释放证据。

任务转换：PENDING→ASSIGNED→EXECUTING→COMPLETED；取消/超时先 REVOKING；失联进入 QUARANTINED。
锁唯一键为 run_id/target_id；epoch 每次重新授权严格递增，禁止复用。
续约只接受新鲜且匹配 epoch 的持有者状态。到期撤销未来授权，但旧持有者/空间进入隔离。
重新分配要求确认旧任务停止，并确认所需空间已清空；失联机恢复必须重同步，拒绝旧命令。
任务结束不等于通道清空，目标锁与物理预约分别处理。新机不得在旧机可能占用的区域获批。
协调器重启不能清空占用后继续发令；本版要求停止新授权、重建并确认全机状态，或开始经落地复位的新 run。
通信可替换，但本版仅有一个逻辑授权者。分布式共识/领导者切换未实现，不在 v1 安全保证内。

## 4. 验收事件流

写入独立 coord_events.jsonl；现有单机 events.jsonl 保留，不伪装成 schema v1。
使用相同公共格式，source_id 固定为本 run 注册的核心输出者；kind=EVENT。
data 必填 event、uav_id（或 null）、task_id（或 null）、reason、details（object）。
事件枚举：RUN_STARTED、TASK_ASSIGNED、LOCK_CHANGED、ROUTE_GRANTED、COMMAND_ACKED、RESERVATION_CHANGED、SAFETY_VIOLATION、TASK_COMPLETED、RUN_FINISHED。

RUN_STARTED.details：fleet_ids（非空且唯一）、required_task_ids（唯一）、test_case_id、config_digest（SHA256 hex）、limits（以下对象）。
limits 必填：min_separation_m、arrival_tolerance_m、mission_timeout_s、deadlock_timeout_s、max_monitor_gap_s（均有限正数）。实际数值由场景配置显式提供，不能由 WB 调参绕过失败。
SAFETY_VIOLATION.reason：COLLISION / SEPARATION_BREACH / DEADLOCK / MISSION_TIMEOUT / ARRIVAL_ERROR / AUTHORIZATION_CONFLICT / STALE_COMMAND_EXECUTED。
RUN_FINISHED.details：outcome（PASS/FAIL/ABSTAIN）、failures（上述失败枚举数组）、completed_task_ids、all_landed_disarmed（bool）、evidence_complete（bool）、monitor_report（以下对象）。
monitor_report：coverage_start_sim_s、coverage_end_sim_s、max_gap_s、collision_count、min_separation_m（少于两机可 null）、max_arrival_error_m、max_no_progress_s。监测数据由测试台提供，核心汇总；核心自身授权事件不能证明物理无碰撞。

判定优先级：
1. 有有效安全违规证据 → FAIL，即使最终正常降落也不能覆盖。
2. 无有效违规，但缺文件/坏 JSON/缺序/混 run/错误 schema/缺 RUN_STARTED 或 RUN_FINISHED/监测不覆盖整个运行/缺必填证据 → ABSTAIN。
3. 仅当任务集合全部完成、全机落地上锁、无失败、监测完整且各阈值满足、未超时，才允许 PASS。
重复 seq 的字节等价事件可去重；不同内容视为证据冲突。最后一条必须为唯一 RUN_FINISHED。
DEADLOCK 由有待完成任务且无任务/路权实际进展持续超阈值判定；单纯心跳和重复发令不算进展。
这是项目开发验收，不宣称官方裁判判定。故障注入导致正确降落可以证明恢复行为，但不能因此把任务完成判定改为 PASS；单独记录恢复测试指标。

## 5. WorkBuddy 接入清单与边界

- 可开始解析 coord_events.jsonl 的公共结构；失败分类和最终 verdict 逻辑由 Codex 提供适配模块，WB 不另写一套。
- 保留 collect_runs.py 的 verdict(run)->(bool|None,str) 插件接口；插件从 run_dir 读取完整事件，而非仅依赖 terminal/reasons 摘要。None 映射 ABSTAIN。
- 独立六机命名空间不等于逐机隔离完成：控制器 flock、ROS 节点名、控制话题冲突检测、TF、模型允许列表、出生坐标、MAVLink ID/端口均需覆盖。
- 导航节点现有 goal 消息不是 ROUTE_GRANT，不得未经授权校验直接转换后放飞；完整飞行接入等待核心与执行器握手测试通过。
- 无官方 launch 时使用明确标记的开发 harness；不能将 TODO 骨架记为六机 E2E 通过。

## 6. 版本记录

- v0 / schema v1，2026-09-10：首次冻结接口与安全生命周期。无旧协同协议需迁移；单机旧日志仍按旧格式保存，不可用来证明 v1 验收通过。
