# ground_segmentation 节点说明文档

`ground_segmentation` 是点云处理流水线中的地面滤波与点云分割功能包。它位于数据输入节点下游，负责接收大范围原始雷达点云，高保真、低耗时地将点云剥离为**地面点**与**非地面点**（树木、电力线、建筑物等悬浮障碍物），同时抑制地下异常点并对起伏地形进行连续局部坡面估计，分别作为后续地面曲面重建和障碍物聚类的输入源。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `ground_segmentation` |
| **可执行文件** | 1. `gmz_node` (推荐默认)<br>2. `ransac_node` (备用基线) |
| **主 C++ 类** | 1. `GMZNode` ([`gmz_node.cpp`](../src/ground_segmentation/src/gmz_node.cpp))<br>2. `RansacNode` ([`ransac_node.cpp`](../src/ground_segmentation/src/ransac_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **$O(N)$ 极速线性复杂度地面提取（GMZ 算法）**：采用二维联合哈希网格极值法（Grid Minimum Z），实现几十万点规模在 20–40 ms 内完成分割。
2. **邻域极低异常点抑制**：针对激光雷达多路径反射或地下噪声产生的极低离群点，通过 $3 \times 3$ 邻域中位数对比与阈值钳位，防止单个地下噪点拉低整块网格的地面基准高程。
3. **局部连续坡面估计（山区高落差地形优化）**：在每个网格上基于 $3 \times 3$ 邻域最低点拟合局部平面梯度与反距离加权（IDW）高程，使陡坡地形下的地面参考高度连续过渡，避免坡面半腰的真实地表被误判为悬浮障碍物。
4. **地面点稀疏化预采样与地下噪点剔除**：输出每个网格修正后的代表点构成稀疏地面点云，极大降低下游泊松曲面重建的计算量；同时丢弃低于最低地面高程容差以下的深层地下噪点。
5. **保持端到端消息时序继承**：完全继承输入点云的 `header.stamp` 与 `header.frame_id`，为后续时间同步（`message_filters`）与端到端延迟监控提供精确依据。

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
                      │ 1. 2D 联合哈希网格极小值提取│
                      │ 2. 3x3 邻域极低异常点抑制   │
                      │ 3. 3x3 局部坡面梯度+IDW拟合 │
                      │ 4. 连续高程切分与地下点丢弃 │
                      └──────────────┬──────────────┘
                                     │
             ┌───────────────────────┴───────────────────────┐
             ▼                                               ▼
  /gs/ground_pointcloud (稀疏地面)            /gs/non_ground_pointcloud (非地面)
             │                                               │
             ▼                                               ▼
     [reconstruction]                                  [clustering]
 (泊松地面曲面网格重建)                               (各向异性 DBSCAN 聚类)
```

### 2.2 GMZ (Grid Minimum Z) 算法深度解析

`GMZNode` 针对大规模机载/车载 LiDAR 起伏地形场景设计，处理流程分为四个阶段：

1. **原点基准计算与网格极小值提取（第一遍遍历）**：
   遍历输入点云，找出全局极小值 $x_{\min}$ 和 $y_{\min}$ 作为二维笛卡尔网格原点。对每个点 $(x_i, y_i, z_i)$ 计算网格索引：
   $$\text{grid}_x = \lfloor (x_i - x_{\min}) / \text{grid\_size} \rfloor, \quad \text{grid}_y = \lfloor (y_i - y_{\min}) / \text{grid\_size} \rfloor$$
   拼接 64 位联合哈希键 $\text{key} = (\text{uint64\_t}(\text{grid}_x) \ll 32) \mid \text{grid}_y$，在哈希表中记录各网格内 $z$ 值最小的点索引。

2. **邻域极低异常点抑制 (`enable_outlier_suppression`)**：
   若开启异常点抑制，遍历每个有效网格的 8 邻域。当有效邻域数 $\ge 3$ 时，计算邻域最低高程的中位数 $z_{\text{med}}$。若当前网格原始最低高程 $z_{\text{raw}} < z_{\text{med}} - \text{outlier\_z\_threshold}$，则将其高程钳位抬升至：
   $$z_{\text{clean}} = z_{\text{med}} - \text{outlier\_z\_threshold}$$
   修正后的网格代表点集合直接作为 `sparse_ground_pcd`（稀疏地面点云）输出。

3. **预计算 $3 \times 3$ 局部坡面模型 (`enable_slope_interpolation`)**：
   对每个网格收集其 $3 \times 3$ 邻域内的修正地面代表点，若邻域点不少于 3 个，利用正则化最小二乘法（对角线正则项 $10^{-3}$）求解局部地形平面梯度 $\nabla z = (g_x, g_y)$，同时记录局部邻域最大高程 $z_{\max\_\text{nb}}$。

4. **连续高程切分与深层地下噪点剔除（第二遍遍历）**：
   对任意点 $(x_i, y_i, z_i)$，若开启 `enable_slope_interpolation`，结合所属网格中心点 $(x_0, y_0, z_0)$ 计算两种连续估计高度：
   * **局部平面投影高程**：$z_{\text{plane}} = \text{clamp}\big(z_0 + g_x(x_i - x_0) + g_y(y_i - y_0),\; z_0,\; z_{\max\_\text{nb}}\big)$
   * **反距离加权高程 (IDW)**：在 $3 \times 3$ 邻域点上以权重 $w_k = \frac{1}{\Delta x_k^2 + \Delta y_k^2 + 1.0}$ 求加权平均 $z_{\text{idw}}$
   * **参考地面高程**：$z_{\text{ref}} = \max(z_0, z_{\text{idw}}, z_{\text{plane}})$（未开启插值时 $z_{\text{ref}} = z_0$）

   按以下区间完成精确切分：
   * 若 $z_i > z_{\text{ref}} + \text{height\_threshold}$：判定为**非地面点**（树木、电力线、建筑等）；
   * 若 $z_0 - \text{height\_threshold} \le z_i \le z_{\text{ref}} + \text{height\_threshold}$：判定为**全量地面点**；
   * 若 $z_i < z_0 - \text{height\_threshold}$：判定为深层**地下噪点**，直接丢弃。

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
  * `Ground: %zu`：识别并分类出的地面点数量。
  * `Non-Ground: %zu`：分离出的悬浮非地面点数量。
  * `Time: %.2f ms`：该帧算法从反序列化、坡面拟合、切分到发布完成的全耗时。

* **输出示例**：
  ```text
  [gmz_node-2] [STAGE-DONE] ground_segmentation: Frame 1 processed | Ground: 182300, Non-Ground: 101860 | Time: 32.40 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `subscribe_topic` (默认 `/io/raw_pointcloud`) | [`pc_msgs/msg/O3DPointCloud`](../src/pc_msgs/msg/O3DPointCloud.msg) | 原始输入点云数据 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `publish_ground_topic` (默认 `/gs/ground_pointcloud`) | [`pc_msgs/msg/O3DPointCloud`](../src/pc_msgs/msg/O3DPointCloud.msg) | GMZ 提取并经异常抑制后的稀疏地面点（供重建使用） |
| `publish_non_ground_topic` (默认 `/gs/non_ground_pointcloud`) | [`pc_msgs/msg/O3DPointCloud`](../src/pc_msgs/msg/O3DPointCloud.msg) | 剥离地表与地下噪点后的障碍物点云（供聚类使用） |

---

## 5. 参数配置说明

配置文件位于 [`src/ground_segmentation/config/gs_config.yaml`](../src/ground_segmentation/config/gs_config.yaml)。

### 5.1 `gmz_node` 参数说明

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/io/raw_pointcloud"` | 订阅的原始输入点云话题 |
| `publish_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 发布的稀疏地面点云话题 |
| `publish_non_ground_topic` | `string` | `"/gs/non_ground_pointcloud"` | 发布的非地面点云话题 |
| `grid_size` | `double` | `20.0` | 水平网格单元格边长尺寸（单位：米）。山地/林地大场景推荐 10.0~20.0 米 |
| `height_threshold` | `double` | `0.8` | 高于参考地面的厚度安全裕度（单位：米）。大于此厚度的点判为非地面点 |
| `enable_outlier_suppression` | `bool` | `true` | 是否开启 $3 \times 3$ 邻域极低异常点抑制 |
| `outlier_z_threshold` | `double` | `3.0` | 允许网格最低点低于邻域中位数高程的最大容差（单位：米） |
| `enable_slope_interpolation` | `bool` | `true` | 是否启用 $3 \times 3$ 局部坡面梯度与 IDW 连续地形高度估计 |
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

### 6.1 GMZ 网格最低点极速滤波（推荐）：[`gs_gmz.launch.py`](../src/ground_segmentation/launch/gs_gmz.launch.py)
流水线默认使用的启动脚本，支持异常点抑制与起伏坡面连续插值。

### 6.2 RANSAC 平面拟合滤波：[`gs_ransac.launch.py`](../src/ground_segmentation/launch/gs_ransac.launch.py)
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
    ├── gmz_node.cpp            # GMZ 网格滤波、异常抑制与连续坡面拟合实现
    └── ransac_node.cpp         # RANSAC 随机抽样一致平面分割实现
```
