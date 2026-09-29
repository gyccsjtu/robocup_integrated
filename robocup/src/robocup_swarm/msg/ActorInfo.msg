# ActorInfo.msg
# 用于官方评分系统的目标检测消息

std_msgs/Header header
string cls  # 目标类别：green, blue, brown, white, red
float64 x
float64 y
float64 z
