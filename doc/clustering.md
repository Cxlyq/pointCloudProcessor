# clustering 节点说明文档

`clustering` 是点云处理流水线中的非地面物体聚类与障碍物分割功能包。它接收来自上游地面滤波后的悬浮非地面点云，通过极坐标特征工程、物理量纲对齐、距离密度自适应平衡以及并行 DBSCAN 算法，将离散的三维点汇聚成独立的语义物体簇（如单座高压铁塔、孤立树木、建筑物等），并滤除超大连通巨团与孤立噪点，输出附带聚类标签的点云数据供后续三维白模外包盒构建使用。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `clustering` |
| **可执行文件** | `parallel_dbscan_node` |
| **主 C++ 类** | `ParallelDbscanNode` ([parallel_dbscan_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/clustering/src/parallel_dbscan_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **各向异性物理量纲对齐与投影**：激光雷达点云在径向（$r$）、水平方位角（$\theta$）与天顶俯仰角（$\phi$）方向的几何分辨率差异极大。通过转换为距离加权的物理米制空间，赋予三个维度独立的调参权重（$r_{\text{weight}}, \theta_{\text{weight}}, \phi_{\text{weight}}$）。
2. **距离密度平衡策略 (Range Balancing)**：解决激光雷达“近密远疏”的固有物理特性——防止近处点云过于密集粘连成超级大长方体，同时防止远端稀疏物体因间距过大被 DBSCAN 当作离群噪声遗漏。
3. **超级巨团与尺寸超限防御过滤**：针对大面积茂密森林或地面未完全剔除干净导致的“连绵巨团”，增加点数上限（`cluster_max_samples`）与物理外接盒最大跨度（`max_cluster_extent`）双重防御机制，超限类别就地置为 `-1` 噪声，杜绝超大白模铺满地面的异常现象。
4. **多核并行与标准化信号输出**：借助 Open3D 内部并行加速 DBSCAN，维持 50~80 ms 级别的高性能，输出标准化 `[STAGE-DONE]` 阶段日志。

---

## 2. 核心机制与算法原理

### 2.1 整体架构图

```
                /gs/non_ground_pointcloud (O3DPointCloud)
                                │
                                ▼
                 ┌─────────────────────────────┐
                 │    parallel_dbscan_node     │
                 │                             │
                 │ 1. 球坐标转换 (r, θ, φ)     │
                 │ 2. 距离密度平方根平衡       │
                 │ 3. 各向异性物理尺度加权     │
                 │ 4. Open3D 并行 DBSCAN 聚类  │
                 │ 5. 超标巨团与超限尺寸过滤   │
                 └──────────────┬──────────────┘
                                │
                                ▼
            /clustering/clustered_pointcloud (ClusteredPointCloud)
                                │
                                ▼
                        [reconstruction]
                     (OBB 外接包围盒白模生成)
```

### 2.2 特征工程：量纲对齐与距离密度平衡

对非地面点云中的任意笛卡尔坐标 $(x, y, z)$：
1. **球坐标分解**：
   $$r = \sqrt{x^2 + y^2 + z^2}$$
   $$\theta = \operatorname{atan2}(y, x)$$
   $$\phi = \arcsin\left(\frac{z}{\max(r, 10^{-6})}\right)$$

2. **自适应距离密度平衡**：
   若开启 `enable_range_balancing: true`，为消除远近距离点间距的二次方发散效应，引入参考距离 $r_{\text{ref}}$（默认 2000 m），计算有效平衡距离：
   $$r_{\text{eff}} = \sqrt{r \cdot r_{\text{ref}}}$$
   若关闭则 $r_{\text{eff}} = r$。

3. **米制投影与各向异性空间映射**：
   将弧度角转换为对应有效半径下的弧长米制量，并乘以各轴权重：
   $$\tilde{x} = r \cdot r_{\text{weight}}$$
   $$\tilde{y} = (r_{\text{eff}} \cdot \theta) \cdot \theta_{\text{weight}}$$
   $$\tilde{z} = (r_{\text{eff}} \cdot \phi) \cdot \phi_{\text{weight}}$$
   将 $(\tilde{x}, \tilde{y}, \tilde{z})$ 作为新虚拟点云送入 DBSCAN。

### 2.3 异常巨型簇双重过滤防御机制
DBSCAN 输出标签后，节点立即统计每个簇 $l$ 的点数 $N_l$ 及三维极值包围盒尺寸：
$$\Delta x_l = x_{l,\max} - x_{l,\min},\quad \Delta y_l = y_{l,\max} - y_{l,\min},\quad \Delta z_l = z_{l,\max} - z_{l,\min}$$
$$\text{extent}_l = \max(\Delta x_l, \Delta y_l, \Delta z_l)$$

判定条件：
* 若 $N_l > \text{cluster\_max\_samples}$（如超过 3000 点）；
* 或 $\text{extent}_l > \text{max\_cluster\_extent}$（如物理跨度超过 500 米）；
则将该簇内所有点的标签强行覆写为 `-1`（噪声）。彻底杜绝误将成片山坡树冠生成整块巨型白模。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在处理完成并发布消息后，输出标准化日志：

```text
[STAGE-DONE] clustering: Frame %zu processed | Input: %zu, Clusters: %d | Time: %.2f ms
```

* **示例输出**：
  ```text
  [parallel_dbscan_node-3] [STAGE-DONE] clustering: Frame 1 processed | Input: 101860, Clusters: 28 | Time: 65.12 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `subscribe_topic` (默认 `/gs/non_ground_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 上游地面分割模块输出的悬浮非地面障碍物点云 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `publish_topic` (默认 `/clustering/clustered_pointcloud`) | [pc_msgs/msg/ClusteredPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/ClusteredPointCloud.msg) | 携带聚类类别标签（`labels`）的点云数据包 |

---

## 5. 参数配置说明

配置文件位于 [`src/clustering/config/c_config.yaml`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/clustering/config/c_config.yaml)。

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/gs/non_ground_pointcloud"` | 订阅非地面点云话题 |
| `publish_topic` | `string` | `"/clustering/clustered_pointcloud"` | 发布聚类后点云话题 |
| `r_ref` | `double` | `2000.0` | 密度平衡参考基准距离（米） |
| `r_weight` | `double` | `2.5` | 径向距离特征权重（增大此值促使沿视线方向分割更敏锐） |
| `theta_weight` | `double` | `0.5` | 水平方位角切向距离特征权重 |
| `phi_weight` | `double` | `0.6` | 俯仰角天顶向距离特征权重 |
| `enable_range_balancing` | `bool` | `true` | 是否启用非线性距离密度平衡缩放策略 |
| `cluster_eps` | `double` | `20.0` | 映射特征空间下的 DBSCAN 邻域搜索半径 $\epsilon$ |
| `cluster_min_samples` | `int` | `15` | DBSCAN 判定核心点的最小样本数邻域阈值 |
| `cluster_max_samples` | `int` | `3000` | 单个类别的最大点数上限；超出者判为噪声团（$\le 0$ 表示不限制） |
| `max_cluster_extent` | `double` | `500.0` | 单个类别的最大物理跨度上限（米）；超出者判为噪声（$\le 0$ 表示不限制） |

---

## 6. 常用配置与启动脚本

### 6.1 启动脚本：[c_pdbscan.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/clustering/launch/c_pdbscan.launch.py)
用于拉起 `parallel_dbscan_node`，自动加载 `c_config.yaml`。

---

## 7. 启动与使用命令参考

```bash
# 1. 单独启动聚类节点
ros2 launch clustering c_pdbscan.launch.py

# 2. 检查聚类后话题消息频率
ros2 topic hz /clustering/clustered_pointcloud

# 3. 查看输出标签的最大类别数与消息结构
ros2 topic echo /clustering/clustered_pointcloud --field max_label
```

---

## 8. 文件结构与索引

```
src/clustering/
├── CMakeLists.txt                 # 构建工程配置 (链接 Open3D, Eigen3)
├── package.xml                    # 包元信息与依赖
├── config/
│   └── c_config.yaml              # 各向异性权重与聚类超参配置
├── launch/
│   └── c_pdbscan.launch.py        # 聚类节点启动 Launch 文件
└── src/
    └── parallel_dbscan_node.cpp   # 特征变换、并行聚类与巨团过滤实现
```
