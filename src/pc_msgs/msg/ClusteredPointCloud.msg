# ClusteredPointCloud.msg
std_msgs/Header header

# 展平的 1D 数组 [x1, y1, z1, ...]
float32[] points

# 每个点对应的簇编号，与 points 数组中的点一一对应，长度为 N
int32[] labels

# 极大值，告知下游节点一共聚了多少个类
int32 max_label