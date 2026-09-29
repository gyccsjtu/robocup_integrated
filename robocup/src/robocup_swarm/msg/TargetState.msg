# 恐怖分子的真值状态（自建消息，由 target_sim_node 发布，供判分/可视化/调试用）。
# 坐标系：x/y 为「地图系」世界坐标（米），与生成器 metadata 一致（ENU，x:-100~100，y:-50~50）。
#
# 规则映射（2026 RoboCup 集群协同搜索）：
#   state=0 WANDER  : 规则2 未感知无人机，1 m/s 随机走动
#   state=1 FLEE    : 规则3 感知到无人机，2 m/s 逃跑（并向同伙广播无人机位置）
#   state=2 RELOCATED: 规则4 被感知 30s 未消除，已瞬移到新位置躲藏
#   state=3 ELIMINATED: 规则5 连续 15s 确认，已从仿真世界移除
Header header

string  target_id     # 目标 id（t0, t1, ... t5）
float64 x             # 地图系 x
float64 y             # 地图系 y
float64 vx            # 速度 x（供无人机做匀速外推预测）
float64 vy            # 速度 y
uint8   state         # 0=游走 1=逃跑 2=已瞬移 3=已消除
float64 tracked_time  # 已被无人机感知的累计时长 s（规则4 的 30s 计时）
float64 confirm_time  # 当前连续确认时长 s（规则5 的 15s 计时）
bool    eliminated    # 是否已被消除（布尔便捷位，等价 state==3）
