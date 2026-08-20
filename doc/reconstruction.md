# reconstruction 节点说明文档

`reconstruction` 是点云处理流水线中的三维白模网格重建功能包。该功能包采用 `message_filters` 的近似时间同步机制，同步接收由上游发来的**地面点云**与**带有聚类标签的非地面点云**，利用多种三维几何重建算法（泊松表面重建、2D Delaunay 三角剖分、3D 凸包、定向包围盒 OBB），将离散点云融合成连续且具有拓扑结构的三维白模网格（Triangle Mesh），并发布自定义 ROS 2 消息供渲染可视化节点使用。

包内针对不同地形与建筑重建需求，提供了 4 个可执行节点。

---

## 1. 节点与可执行文件概览

| 节点名称 | 可执行文件 | 主 C++ 类 | 地面重建策略 | 非地面物体重建策略 | 特色功能 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **基础重建节点** | `reconstruction_node` | `ReconstructionNode` ([reconstruction_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/src/reconstruction_node.cpp#L24)) | 3D 凸包 / OBB | 3D 凸包 / OBB | 基础轻量重建 |
| **泊松+凸包混合节点** | `reconstruction_g_poisson_ng_qh_node` | `ReconstructionPoissonQHNode` ([reconstruction_g_poisson_ng_qh_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/src/reconstruction_g_poisson_ng_qh_node.cpp#L32)) | 泊松表面重建 (Poisson) | 3D 凸包 / OBB | 平滑平整的地形曲面拟合 |
| **DLN+凸包混合节点** | `reconstruction_g_dln_ng_qh_node` | `ReconstructionDLNNode` ([reconstruction_g_dln_ng_qh_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/src/reconstruction_g_dln_ng_qh_node.cpp#L30)) | 2D Delaunay 三角剖分 (DLN) | 3D 凸包 / OBB | 极速地形网格化、长边剔除 |
| **泊松+OBB混合节点** | `reconstruction_g_poisson_ng_obb_node` | `ReconstructionPoissonOBBNode` ([reconstruction_g_poisson_ng_obb_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/src/reconstruction_g_poisson_ng_obb_node.cpp#L30)) | 泊松表面重建 (Poisson) | 定向包围盒 (OBB) | 规则建筑立方体外形 & PLY 文件导出 |

---

## 2. 核心架构与同步机制

### 2.1 近似时间戳同步 (`message_filters`)
由于地面点云（来自地面分割节点）和聚类点云（来自聚类节点）是并行异步计算发布的，两者的 ROS 消息时间戳需要精确对齐。
节点内部使用 `message_filters::Synchronizer` 与 `ApproximateTime` 策略匹配同源点云：

```cpp
typedef message_filters::sync_policies::ApproximateTime<
    pc_msgs::msg::O3DPointCloud,
    pc_msgs::msg::ClusteredPointCloud> SyncPolicy;
```

当一对帧时间戳匹配成功后，触发 `sync_callback` 执行真正的网格重建流程。

---

## 3. 重建算法原理与策略细节

重建逻辑分为**阶段 1（地面地形重建）**和**阶段 2（非地面障碍物重建）**，最后将生成的网格拼接为统一的 `TriangleMesh`。

```
 [地面点云 /gs/ground_pointcloud]       [聚类点云 /clustering/clustered_pointcloud]
                   │                                         │
                   └────────────────────┬────────────────────┘
                                        ▼ (message_filters 结合近似时间同步)
                                 [sync_callback]
                                        │
             ┌──────────────────────────┴──────────────────────────┐
             ▼ (地面策略选择)                                        ▼ (非地面策略)
 ┌───────────────────────┐                               ┌───────────────────────┐
 │ 策略 A: Poisson 泊松  │                               │ 按 cluster_id 遍历     │
 │ 策略 B: 2D Delaunay   │                               │ 过滤 size < min_size  │
 │ 策略 C: Convex Hull   │                               │ 计算 3D 凸包 ConvexHull│
 └───────────┬───────────┘                               │ 降级计算 OBB 包围盒    │
             │                                           └───────────┬───────────┘
             └──────────────────────────┬────────────────────────────┘
                                        ▼
                           [*combined_mesh += *sub_mesh]
                                        │
                                        ▼
             [打包并发布 pc_msgs::msg::O3DMesh 至 /reconstruction/white_mesh]
```

---

### 3.1 地面重建算法对比

#### A. 泊松表面重建算法 (Poisson Surface Reconstruction)
用于生成高连续性、光滑的地形网格（如 `reconstruction_g_poisson_ng_qh_node`）：
1. **法线估计与方向强制对齐**：使用 `KDTreeSearchParamHybrid(normal_radius, normal_max_nn)` 计算地面点法线，并通过 `OrientNormalsToAlignWithDirection(Eigen::Vector3d(0, 0, 1))` 强制将法线朝向统一校正为 $Z$ 轴正方向（向上）。
2. **泊松重建**：调用 `CreateFromPointCloudPoisson(*ground_pcd, poisson_depth, ...)`，在八叉树深度 `poisson_depth` 下指示隐式函数重建曲面。
3. **密度分位数剪枝 (Density Pruning)**：泊松重建会在没有点云的空旷边缘产生一个闭合的“橄榄球底座”假阳性网格。算法提取每个顶点的重建密度 `densities`，按分位数 `density_quantile`（如最低 6% 密度）过滤低密度顶点 (`RemoveVerticesByMask`)，剔除边缘假阳性结构。
4. **包围盒裁剪**：使用原地面点云的 AABB 轴对齐包围盒 `GetAxisAlignedBoundingBox()` 进一步裁剪 `Crop(bbox)`，死死将网格限制在实际测量点云范围内。

#### B. 2D Delaunay 三角剖分算法 (Delaunay Triangulation)
适用于极速、无空洞损耗的地形表面重建（如 `reconstruction_g_dln_ng_qh_node`）：
1. **2D 平面投影**：提取地面点坐标的 $(x, y)$ 投影坐标。
2. **极速 2D Delaunay 剖分**：使用 [delaunator.hpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/include/reconstruction/delaunator.hpp) 库生成 2D 拓扑三角网 `delaunator::Delaunator d(coords)`。
3. **拉扯长边凹包剔除**：标准 Delaunay 会自动补全凸包，导致边缘产生拉扯的非物理空洞三角形。算法检查每个三角形的三条边长平方，若任一边长大于阈值 $\text{max\_edge\_length}^2$，则直接舍弃该三角形。
4. **法线朝向校正**：计算三角形两边向量叉乘 $(v_1 \times v_2).z$，若朝下（$<0$）则翻转顶点序号顺序，确保法线一致向上。

---

### 3.2 非地面障碍物/建筑重建算法

#### A. 3D 凸包重建 (Convex Hull)
对聚类标签 $i \in [0, \text{max\_label}]$ 的障碍物点云：
- 过滤包含点数小于 `min_cluster_size` 的微小碎块噪声。
- 调用 Open3D `cluster_pcd->ComputeConvexHull()` 计算三维凸包 Mesh。
- 计算顶点法线并统一涂上现代白模浅灰颜色 (`PaintUniformColor(0.9, 0.9, 0.9)`)。

#### B. 面向包围盒重建 (Oriented Bounding Box, OBB)
- 降级保护：若凸包算法抛出异常（例如障碍物点云共面无法构成 3D 凸包），则触发降级策略：
  使用 `cluster_pcd->GetOrientedBoundingBox()` 计算紧致的 3D 最小外接包围盒，并转化为 6 面体白模网格 `CreateFromOrientedBoundingBox(obb)`。
- 在 `reconstruction_g_poisson_ng_obb_node` 中，所有建筑/障碍物均统一采用 OBB 立方体建模，非常适合规则建筑物外形展示。

---

## 4. ROS 2 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/gs/ground_pointcloud` | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 地面分割节点发布的地面点云 |
| `/clustering/clustered_pointcloud` | [pc_msgs/msg/ClusteredPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/ClusteredPointCloud.msg) | 聚类节点发来的聚类点云及标签 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/reconstruction/white_mesh` | [pc_msgs/msg/O3DMesh](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DMesh.msg) | 生成的三维白模 Mesh 消息，供可视化节点渲染 |

### 4.3 关联消息数据结构 (`pc_msgs::msg::O3DMesh`)
```protobuf
std_msgs/Header header       # 继承的时间戳与坐标系 ID
float32[] vertices          # 展平的顶点坐标 [v0_x, v0_y, v0_z, v1_x, ...]
int32[] triangles           # 展平的三角形索引 [t0_v0, t0_v1, t0_v2, t1_v0, ...]
float32[] vertex_normals    # 展平的顶点法线 [n0_x, n0_y, n0_z, ...]
float32[] vertex_colors     # 展平的顶点颜色 [r0, g0, b0, ...]
```

---

## 5. 参数配置说明

配置文件位于 [reconstruction/config/r_config.yaml](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/reconstruction/config/r_config.yaml)。

| 参数分类 | 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- | :--- |
| **通用话题** | `sub_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 订阅的地面点云话题 |
| | `sub_cluster_topic` | `string` | `"/clustering/clustered_pointcloud"` | 订阅的聚类点云话题 |
| | `pub_mesh_topic` | `string` | `"/reconstruction/white_mesh"` | 发布的三维网格话题 |
| | `min_cluster_size` | `int` | `15` | 参与网格重建的障碍物最小点数阈值 |
| **泊松重建** | `poisson_depth` | `int` | `6` ~ `8` | 泊松表面重建的八叉树求解深度 |
| | `density_quantile` | `double` | `0.06` | 低密度假阳性网格切除分位数 |
| | `normal_radius` | `double` | `20.0` | 估计顶点法线时的 KDTree 搜索半径 |
| | `normal_max_nn` | `int` | `15` ~ `30` | 估计法线时的最大近邻点数 |
| **DLN 重建** | `max_edge_length` | `double` | `80.0` ~ `1080.0` | 2D Delaunay 三角形最大允许边长 |
| **PLY 保存** | `data_saved_dir` | `string` | `""` | 白模网格保存的目标文件夹路径 |
| | `enable_terrain_mesh_saving` | `bool` | `false` | 保存地形 Mesh 为 `terrain_*.ply` |
| | `enable_architecture_mesh_saving` | `bool` | `false` | 保存建筑/障碍物 Mesh 为 `architecture_*.ply` |
| | `enable_combined_mesh_saving` | `bool` | `false` | 保存总 Mesh 为 `combined_*.ply` |

---

## 6. 启动与使用方式

根据所需调用的算法节点选择对应的 Launch 文件启动：

```bash
# 1. 启动基础凸包/OBB 重建节点
ros2 launch reconstruction r_reconstruction.launch.py

# 2. 启动地面泊松 + 非地面凸包重建节点
ros2 launch reconstruction r_g_poisson_ng_qh.launch.py

# 3. 启动地面 2D Delaunay + 非地面凸包重建节点
ros2 launch reconstruction r_g_dln_ng_qh.launch.py

# 4. 启动地面泊松 + 非地面 OBB 包围盒重建节点（支持本地 PLY 保存）
ros2 launch reconstruction r_g_poisson_ng_obb.launch.py
```

---

## 7. 文件结构与索引

```
src/reconstruction/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── r_config.yaml                           # 白模重建节点参数配置文件
├── include/reconstruction/
│   └── delaunator.hpp                          # 极速 2D Delaunay 三角剖分头文件
├── launch/
│   ├── r_reconstruction.launch.py              # 基础重建 Launch
│   ├── r_g_poisson_ng_qh.launch.py             # 泊松+凸包重建 Launch
│   ├── r_g_dln_ng_qh.launch.py                 # DLN+凸包重建 Launch
│   └── r_g_poisson_ng_obb.launch.py            # 泊松+OBB重建 Launch
└── src/
    ├── reconstruction_node.cpp                 # 基础重建节点源码
    ├── reconstruction_g_poisson_ng_qh_node.cpp # 泊松+凸包重建节点源码
    ├── reconstruction_g_dln_ng_qh_node.cpp     # DLN+凸包重建节点源码
    └── reconstruction_g_poisson_ng_obb_node.cpp# 泊松+OBB重建节点源码
```
