# ground_segmentation 节点说明文档

`ground_segmentation` 是点云处理流水线中的地面滤波与点云分割功能包。它位于数据输入节点下游，负责接收大范围原始雷达点云，高保真、低耗时地将点云剥离为**地面点**、**非地面点**（树木、电力线、建筑物等障碍物），并直接滤除深层地下异常噪点，分别作为后续地面曲面重建和障碍物聚类的输入源。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `ground_segmentation` |
| **可执行文件** | 1. `gmz_node` (推荐默认)<br>2. `ransac_node` (备用基线) |
| **主 C++ 类** | 1. `GMZNode` ([gmz_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/ground_segmentation/src/gmz_node.cpp))<br>2. `RansacNode` ([ransac_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/ground_segmentation/src/ransac_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **$O(N)$ 极速线性复杂度地面提取与坡面建模（增强型 GMZ 算法）**：采用二维联合哈希网格极值法提取各单元最低基准，并结合 $3\times 3$ 邻域异常极低点抑制与局部坡面连续高程拟合，在起伏山地和陡坡地形下实现高保真分割，几十万点规模可在几十毫秒内完成。
2. **地下离群噪点抑制与稀疏地面点预采样**：GMZ 节点通过 $3\times 3$ 邻域中位数约束修正孤立极低噪点，输出网格化干净地面代表点作为稀疏地面点（每个网格保留 1 个特征点），同时在分割阶段丢弃低于局部地表阈值的深层地下噪点，降低下游泊松曲面重建误差与显存占用。
3. **纯地形帧安全发布与端到端时序继承**：当纯地形场景经过坡面高程补偿后无非地面点时，节点仍正常发布稀疏地面点云确保地形建模不中断；输出消息完全继承输入点云的 `header.stamp` 与 `header.frame_id`。

---

## 2. 核心机制与算法原理

### 2.1 整体架构图

```
                       /io/raw_pointcloud (O3DPointCloud)
                                     │
                                     ▼
                      ┌─────────────────────────────┐
                      │    gmz_node / ransac_node   │
                      │                             │
                      │  1. 2D 联合哈希网格最低点   │
                      │  2. 3×3 邻域极低噪点抑制    │
                      │  3. 3×3 局部坡面与 IDW 建模 │
                      │  4. 连续地表高程切分过滤    │
                      └──────────────┬──────────────┘
                                     │
             ┌───────────────────────┴───────────────────────┐
             ▼                                               ▼
  /gs/ground_pointcloud (稀疏地面)            /gs/non_ground_pointcloud (非地面)
             │                                               │
             ▼                                               ▼
     [reconstruction]                                  [clustering]
 (泊松地面曲面重建 + 二次防穿模)                    (各向异性 DBSCAN 聚类)
```

### 2.2 GMZ (Grid Minimum Z + 坡面插值补偿) 算法深度解析

GMZ 算法针对大规模起伏山地机载/车载 LiDAR 场景设计，核心由四阶段线性流程构成：

1. **原点基准计算**：
   遍历输入点云，找出全局极小值 $x_{\min}$ 和 $y_{\min}$，作为二维笛卡尔网格的基准原点。

2. **第一遍遍历：网格极小值提取与 64 位联合哈希**：
   对任意三维点 $(x_i, y_i, z_i)$，计算其二维网格单元索引：
   $$\text{grid}_x = \lfloor (x_i - x_{\min}) / \text{grid\_size} \rfloor$$
   $$\text{grid}_y = \lfloor (y_i - y_{\min}) / \text{grid\_size} \rfloor$$
   使用 64 位无符号整型将行列索引无缝拼接为唯一哈希键：
   $$\text{key} = (\text{uint64\_t}(\text{grid}_x) \ll 32) \mid \text{grid}_y$$
   在 `std::unordered_map<uint64_t, size_t>` 中记录并更新落入该网格中具有最小 $z$ 值的点索引。

3. **3×3 邻域极低异常点抑制与局部坡面模型预计算**：
   * **极低噪点抑制**（`enable_outlier_suppression: true`）：对每个有效网格，收集其周围 $3\times 3$ 邻域（不包含自身）有效网格的最低点高程。当有效邻域数 $\ge 3$ 时计算邻域中位数 $z_{\text{med}}$；若当前网格最低点高程 $z_{\text{raw}} < z_{\text{med}} - \text{outlier\_z\_threshold}$，则将其代表点高程上钳至 $z_{\text{clean}} = z_{\text{med}} - \text{outlier\_z\_threshold}$。修正后的各网格代表点 $(x_g, y_g, z_{\text{clean}})$ 直接构成 `sparse_ground_pcd`（稀疏地面点集）。
   * **局部坡面模型预计算**（`CellSlopeModel`，`enable_slope_interpolation: true`）：对每个网格收集包含自身在内的 $3\times 3$ 邻域代表点（最多 9 个）及最高邻域高程 $z_{\max\_\text{nb}}$；当邻域点数 $\ge 3$ 时，以当前网格代表点为原点构建带微弱 Tikhonov 正则化（$10^{-3} I$）的 $2\times 2$ 最小二乘正规方程，解析求解二维地形坡度梯度 $\mathbf{g} = (g_x, g_y)^T$。

4. **第二遍遍历：连续坡面高程切分与深层地下点滤除**：
   再次遍历全部点云，对点 $(x_i, y_i, z_i)$ 落入的网格提取其局部模型（当前基准高程 $\text{own\_z} = z_{\text{clean}}$）：
   * 若开启 `enable_slope_interpolation`，同时计算：
     * **受约束局部平面外推高程**：
       $$z_{\text{plane}} = \operatorname{clamp}\big(\text{own\_z} + g_x (x_i - x_g) + g_y (y_i - y_g),\ \text{own\_z},\ z_{\max\_\text{nb}}\big)$$
     * **邻域反距离加权（IDW）插值高程**（邻域点数 $\ge 2$ 时，权重 $w_k = \frac{1}{(x_i - x_k)^2 + (y_i - y_k)^2 + 1.0}$）：
       $$z_{\text{idw}} = \frac{\sum_k w_k z_k}{\sum_k w_k}$$
     * 综合地表估计高程：
       $$\text{ref\_ground\_z} = \max(\text{own\_z},\ z_{\text{idw}},\ z_{\text{plane}})$$
   * **三分类过滤**：
     * 若 $z_i > \text{ref\_ground\_z} + \text{height\_threshold}$，判定为**非地面点**（加入 `non_ground_pcd`）；
     * 若 $\text{own\_z} - \text{height\_threshold} \le z_i \le \text{ref\_ground\_z} + \text{height\_threshold}$，判定为**全量地面点**（加入 `ground_pcd`）；
     * 若 $z_i < \text{own\_z} - \text{height\_threshold}$，判定为**深层地下异常噪点**直接丢弃。

### 2.3 RANSAC 平面拟合原理（备选节点）
通过 Open3D 的 `SegmentPlane` 接口，在 $N$ 次随机采样迭代中寻找点云内点数最多的最优几何平面方程 $ax + by + cz + d = 0$。适用于平坦地带或简单建筑台基，但在起伏山地与连续坡面地形中，建议优先使用 GMZ。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在处理完一帧并发布后，输出标准化日志：

```text
[STAGE-DONE] ground_segmentation: Frame %zu processed | Ground: %zu, Non-Ground: %zu | Time: %.2f ms
```

* **字段含义**：
  * `Frame %zu`：节点自启动以来的处理帧序编号。
  * `Ground: %zu`：识别并保留的全量地面点数量。
  * `Non-Ground: %zu`：分离出的悬浮非地面点数量。
  * `Time: %.2f ms`：该帧算法从反序列化、网格切分、坡面建模到发布完成的全耗时。

* **输出示例**：
  ```text
  [gmz_node-2] [STAGE-DONE] ground_segmentation: Frame 1 processed | Ground: 33133, Non-Ground: 77309 | Time: 63.82 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `subscribe_topic` (默认 `/io/raw_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 原始输入点云数据 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `publish_ground_topic` (默认 `/gs/ground_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | GMZ 提取并经异常点抑制后的稀疏地面点云（供重建使用） |
| `publish_non_ground_topic` (默认 `/gs/non_ground_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 剥离地表及坡面后的障碍物点云（供聚类使用；纯地形帧为空时不发布） |

---

## 5. 参数配置说明

配置文件位于 [`src/ground_segmentation/config/gs_config.yaml`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/ground_segmentation/config/gs_config.yaml)。

### 5.1 `gmz_node` 参数说明

| 参数名称 | 类型 | 默认配置值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/io/raw_pointcloud"` | 订阅的原始输入点云话题 |
| `publish_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 发布的稀疏地面点云话题 |
| `publish_non_ground_topic` | `string` | `"/gs/non_ground_pointcloud"` | 发布的非地面点云话题 |
| `grid_size` | `double` | `20.0` | 水平网格单元格边长尺寸（单位：米）。山地/林地大场景推荐 10.0~20.0 米 |
| `height_threshold` | `double` | `0.8` | 相对估计地表的地面厚度安全裕度（单位：米）。高于估计地表此厚度的点判为非地面点 |
| `enable_outlier_suppression` | `bool` | `true` | 是否启用 3×3 邻域极低异常点抑制，防止孤立地下噪点拉低网格基准 |
| `outlier_z_threshold` | `double` | `3.0` | 单个网格最低点允许低于 3×3 邻域中位数的最大高差（米）；超出则上钳至 `med_z - outlier_z_threshold` |
| `enable_slope_interpolation` | `bool` | `true` | 是否启用 3×3 邻域局部平面拟合 + IDW 连续坡面插值补偿，消除陡坡阶梯误判 |
| `data_saved_dir` | `string` | `""` | 调试用点云保存目录 |
| `enable_sparse_ground_points_saving` | `bool` | `false` | 是否落盘保存稀疏地面点云（`.ply`） |
| `enable_ground_points_saving` | `bool` | `false` | 是否落盘保存全量地面点云 |
| `enable_non_ground_points_saving` | `bool` | `false` | 是否落盘保存非地面点云 |

### 5.2 `ransac_node` 参数说明

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/io/raw_pointcloud"` | 订阅话题 |
| `publish_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 地面点发布话题 |
| `publish_non_ground_topic` | `string` | `"/gs/non_ground_pointcloud"` | 非地面点发布话题 |
| `distance_threshold` | `double` | `13.0` | 点到平面拟合模型的最大内点判定距离（米） |
| `ransac_n` | `int` | `3` | 拟合平面的采样点数 |
| `num_iterations` | `int` | `100` | RANSAC 最大采样迭代次数 |

---

## 6. 常用配置与启动脚本

功能包提供两个独立 Launch 文件：

### 6.1 GMZ 网格最低点极速滤波（推荐）：[gs_gmz.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/ground_segmentation/launch/gs_gmz.launch.py)
流水线默认使用的启动脚本，具备坡面适应性、抗地下噪点能力与实时性。

### 6.2 RANSAC 平面拟合滤波：[gs_ransac.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/ground_segmentation/launch/gs_ransac.launch.py)
适用于地势平缓、结构规则的平面场景对比测试。

---

## 7. 启动与使用命令参考

```bash
# 1. 启动 GMZ 地面分割节点（主流水线默认）
ros2 launch ground_segmentation gs_gmz.launch.py

# 2. 启动 RANSAC 地面分割节点
ros2 launch ground_segmentation gs_ransac.launch.py

# 3. 实时查看地面与非地面点云话题
ros2 topic hz /gs/ground_pointcloud
ros2 topic hz /gs/non_ground_pointcloud
```

---

## 8. 文件结构与索引

```
src/ground_segmentation/
├── CMakeLists.txt              # 构建工程配置
├── package.xml                 # ROS 2 包定义
├── config/
│   └── gs_config.yaml          # GMZ 与 RANSAC 算法参数配置
├── launch/
│   ├── gs_gmz.launch.py        # GMZ 节点拉起脚本
│   └── gs_ransac.launch.py     # RANSAC 节点拉起脚本
└── src/
    ├── gmz_node.cpp            # 增强型 GMZ（网格最低点 + 异常抑制 + 坡面插值）实现
    └── ransac_node.cpp         # RANSAC 随机抽样一致平面分割实现
```
