# clustering 节点说明文档

`clustering` 是点云处理流水线中的障碍物聚类功能包。该功能包接收由地面分割节点发来的**非地面点云**，利用特征工程将点云转换至**加权极坐标空间**，并使用 **DBSCAN**（Density-Based Spatial Clustering of Applications with Noise）密度聚类算法，将离散的点云归类分割为独立的障碍物或建筑目标实体。

---

## 1. 节点与可执行文件概览

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `clustering` |
| **可执行文件** | `parallel_dbscan_node` |
| **主 C++ 类** | `ParallelDbscanNode` ([parallel_dbscan_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/clustering/src/parallel_dbscan_node.cpp#L17)) |
| **依赖库** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `Eigen3` |

---

## 2. 核心算法逻辑与特征工程

`parallel_dbscan_node` 的核心逻辑是将传统的笛卡尔空间聚类转换为**加权极坐标特征空间**聚类，流程如下：

```
[非地面点云 /gs/non_ground_pointcloud]
                   │
                   ▼
  [笛卡尔坐标 (x, y, z) 提取与校验]
                   │
                   ▼
    [特征工程：极坐标转换与权重加权]
  r = √(x² + y² + z²)
  θ = atan2(y, x)
  φ = asin(z / max(r, 1e-6))
  虚拟点坐标: (r · w_r,  θ · w_θ,  φ · w_φ)
                   │
                   ▼
       [Open3D ClusterDBSCAN()]
  基于 cluster_eps 与 cluster_min_samples
                   │
                   ▼
       [获取每个点的 Cluster Label]
  (正常类: 0, 1, ..., max_label; 噪声点: -1)
                   │
                   ▼
[打包并发布 pc_msgs::msg::ClusteredPointCloud 至 /clustering/clustered_pointcloud]
```

### 关键步骤与数学原理：

1. **加权极坐标空间特征映射 (Polar Coordinate Feature Mapping)**：
   - **背景与痛点**：激光雷达点云具有“近密远稀”的物理特性。若直接在三维笛卡尔空间 $(x,y,z)$ 中使用固定的 $\epsilon$ 半径执行 DBSCAN 聚类，近处的点云容易被过度过拟合聚成一大块，而远处的点云因为点间距增大容易被误判定为离群噪声（Noise）。
   - **坐标变换与权重**：节点将每个三维坐标 $(x_i, y_i, z_i)$ 转换为球极坐标：
     $$r_i = \sqrt{x_i^2 + y_i^2 + z_i^2}$$
     $$\theta_i = \operatorname{atan2}(y_i, x_i)$$
     $$\phi_i = \operatorname{asin}\left(\frac{z_i}{\max(r_i, 10^{-6})}\right)$$
   - **构造虚拟特征点**：乘以调优好的空间权重因子 $(w_r, w_\theta, w_\phi)$，构造出虚拟特征空间点 $\mathbf{p}_{\text{feat}} = (r_i \cdot w_r, \theta_i \cdot w_\theta, \phi_i \cdot w_\phi)$。在极坐标特征空间中，点云的分布密度更加均匀一致，显著增强了基于欧式距离的 DBSCAN 算法对不同距离目标的聚类效果。

2. **DBSCAN 算法执行**：
   - 将虚拟特征点载入 Open3D `PointCloud` 对象 `feature_pcd`。
   - 调用 Open3D 的 C++ API：
     `labels = feature_pcd->ClusterDBSCAN(cluster_eps_, cluster_min_samples_, false)`
   - 算法自动计算并输出大小为 $N$ 的标签数组 `labels`，其中非负整数表示点归属的聚类簇 ID，`-1` 表示噪声点。

3. **高效零拷贝消息构建**：
   - 创建 `pc_msgs::msg::ClusteredPointCloud` 消息。
   - 消息中的 `points` 直接复用输入消息中的原始三维点坐标，避免重新转换拷贝；`labels` 填入计算出的聚类 ID，`max_label` 记录最大的类别标签索引（方便下游重建节点遍历）。

---

## 3. ROS 2 通信接口

### 3.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/gs/non_ground_pointcloud` | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 由地面分割节点发来的非地面点云 |

### 3.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/clustering/clustered_pointcloud` | [pc_msgs/msg/ClusteredPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/ClusteredPointCloud.msg) | 带有聚类标签的点云数据，供下游白模重建使用 |

### 3.3 关联消息数据结构 (`pc_msgs::msg::ClusteredPointCloud`)
```protobuf
std_msgs/Header header   # 继承的时间戳与坐标系 ID
float32[] points        # 展平的三维点坐标 [x0, y0, z0, ...]
int32[] labels          # 每个点对应的聚类标签，与 points 逐点对应 (-1 表示噪声)
int32 max_label         # 当前帧中最大的类别 ID 索引
```

---

## 4. 参数配置说明

配置文件位于 [clustering/config/c_config.yaml](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/clustering/config/c_config.yaml)。

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/gs/non_ground_pointcloud"` | 订阅的非地面点云话题 |
| `publish_topic` | `string` | `"/clustering/clustered_pointcloud"` | 发布聚类点云的话题 |
| `r_weight` | `double` | `3.0` | 极坐标距离 $r$ 轴维度权重 |
| `theta_weight` | `double` | `650.0` | 极坐标方位角 $\theta$ 轴维度权重 |
| `phi_weight` | `double` | `600.0` | 极坐标仰角 $\phi$ 轴维度权重 |
| `cluster_eps` | `double` | `18.0` | DBSCAN 聚类的邻域搜索半径 $\epsilon$ |
| `cluster_min_samples` | `int` | `5` | DBSCAN 构成核心点（Core Point）所需的最小邻域点数 |

---

## 5. 启动与使用方式

### 5.1 Launch 启动
使用 ROS 2 Launch 启动脚本：
```bash
ros2 launch clustering c_pdbscan.launch.py
```

### 5.2 Launch 脚本解析
[c_pdbscan.launch.py](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/clustering/launch/c_pdbscan.launch.py) 会自动读取 `c_config.yaml` 配置，启动 `parallel_dbscan_node` 可执行文件。

---

## 6. 文件结构与索引

```
src/clustering/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── c_config.yaml                # 聚类节点配置文件
├── launch/
│   └── c_pdbscan.launch.py          # 聚类节点 Launch 脚本
└── src/
    └── parallel_dbscan_node.cpp     # 极坐标加权 DBSCAN 节点核心实现
```
