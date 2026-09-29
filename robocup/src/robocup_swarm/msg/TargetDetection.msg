# 无人机对恐怖分子的一次检测事件（自建消息，由 swarm_agent 发布给管理器）。
#
# 判定方式（规则3 的「几何判定」，比赛主判定）：
#   水平距离 < DETECT_RADIUS 且 视线（LOS）无建筑遮挡 → 视为检测到
# 可选增强：机载下视相机（YOLO）也发布本消息，source 字段区分来源。
Header header

string  uav_id        # 哪架无人机检测到
string  target_id     # 检测到哪个恐怖分子
float64 x             # 检测到的目标位置（地图系 x，供管理器做预测/接力）
float64 y             # 检测到的目标位置（地图系 y）
float32 confidence    # 检测置信度 [0,1]（几何判定给 1.0；视觉给模型置信度）
uint8   source        # 0=几何判定（LOS） 1=下视相机视觉 2=融合
