# O3DPointCloud.msg
std_msgs/Header header

# 展平的 1D 数组，存储 N*3 个浮点数 [x1, y1, z1, x2, y2, z2, ...]
float32[] points

# 可选：颜色的 1D 数组 [r1, g1, b1, r2, g2, b2, ...] (0.0 到 1.0)
float32[] colors