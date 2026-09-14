# PowerTowerGPS.msg
# 高压塔GPS坐标消息，包含若干高压塔的经纬高度信息
# 三个数组同一索引下的数据对应同一座高压塔的位置
# 使用 float64（double）类型，保证至少 15 位有效数字的精度

std_msgs/Header header

float64[] latitudes   # 纬度数组 (度, WGS84)，索引 i 对应第 i 座高压塔的纬度
float64[] longitudes  # 经度数组 (度, WGS84)，索引 i 对应第 i 座高压塔的经度
float64[] altitudes   # 高度数组 (米)，索引 i 对应第 i 座高压塔的高度
