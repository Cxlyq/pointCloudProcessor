# O3DMesh.msg
std_msgs/Header header

# 顶点的 1D 数组 (V*3 个元素)
float32[] vertices

# 面片（三角形）索引的 1D 数组 (T*3 个元素)
int32[] triangles

# 顶点法线的 1D 数组 (用于渲染光照)
float32[] vertex_normals

# 顶点颜色的 1D 数组 (在这里通常是 [0.9, 0.9, 0.9, ...])
float32[] vertex_colors