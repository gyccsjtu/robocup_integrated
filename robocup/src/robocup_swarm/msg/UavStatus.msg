# 单架无人机上报给集中式管理器的状态（自建消息，供集群协同搜索）。
# 坐标系：x/y 为「地图系」世界坐标（米），与生成器 metadata 一致（ENU，x:-100~100，y:-50~50）。
Header header

string  uav_id        # 无人机 id（uav_1, uav_2, ...）
float64 x             # 地图系 x
float64 y             # 地图系 y
float64 z             # 高度
bool    connected     # mavros 是否已连接

int32   cell_ix       # 当前任务格索引 x（无任务 = -1）
int32   cell_iy       # 当前任务格索引 y（无任务 = -1）
float32 confidence    # 当前任务格覆盖置信度 [0,1]（管理器据此判定「已覆盖」）
int32   assigned_cell_ix   # 被分配格索引 x
int32   assigned_cell_iy   # 被分配格索引 y
float32 assigned_target_x  # 被分配格中心 x
float32 assigned_target_y  # 被分配格中心 y
