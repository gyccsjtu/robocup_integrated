# 集中式管理器指派给某架无人机的搜索任务（自建消息）。
# target_x/y 为任务格中心的地图系坐标，agent 可据此规划/飞行。
Header header

string  uav_id        # 指派给谁
int32   cell_ix       # 任务格索引 x
int32   cell_iy       # 任务格索引 y
float64 target_x      # 任务格中心 x（地图系）
float64 target_y      # 任务格中心 y（地图系）
uint8   task_type     # 0=搜索 1=目标确认 2=追踪接力（本阶段只用 0）
