# PlaneState.msg
# 载机当前状态消息，包含经纬度、海拔高度和航向角
# 使用 float64（double）类型，保证至少 15 位有效数字的精度

std_msgs/Header header

float64 latitude   # 纬度 (度, WGS84)
float64 longitude  # 经度 (度, WGS84)
float64 altitude   # 海拔高度 (米)
float64 course     # 航向角 (度, 以正北为0，顺时针为正)
