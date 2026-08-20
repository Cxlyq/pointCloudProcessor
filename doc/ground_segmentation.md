# ground_segmentation 节点说明文档

`ground_segmentation` 是点云处理流水线中的地面分割功能包。该功能包负责接收原始激光雷达点云，将其准确剥离分割为**地面点云**（Ground Point Cloud）和**非地面点云**（Non-Ground Point Cloud），为后续的障碍物聚类与三维重建提供洁净的数据输入。

包内提供了两种不同算法原理的地面分割节点：
1. `ransac_node`：基于 RANSAC 随机抽样一致性平面的地面分割节点。
2. `gmz_node`：基于 GMZ (Grid Minimum Z，网格最低点) 算法的地面分割节点。

---

## 1. 节点与可执行文件概览

| 节点名称 | 可执行文件 | 主 C++ 类 | 核心算法特性 |
| :--- | :--- | :--- | :--- |
| **RANSAC 节点** | `ransac_node` | `RansacNode` ([ransac_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/ground_segmentation/src/ransac_node.cpp#L18)) | 基于 RANSAC 平面拟合提取平坦地面 |
| **GMZ 节点** | `gmz_node` | `GMZNode` ([gmz_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/ground_segmentation/src/gmz_node.cpp#L21)) | 基于 2D 网格最低点筛选与高差阈值滤波，复杂度 $O(N)$，支持提取稀疏地面点骨架 |

---

## 2. 算法逻辑与工作流程

### 2.1 RANSAC 平面拟合节点 (`ransac_node`)

RANSAC 节点利用 3D 平面拟合算法分割地面，工作流程如下：

```
[ROS 消息 /io/raw_pointcloud] ──► [反序列化为 Open3D PointCloud]
                                              │
                                              ▼
                               [Open3D SegmentPlane()]
                                拟合平面 ax + by + cz + d = 0
                                              │
                     ┌────────────────────────┴────────────────────────┐
                     ▼                                                 ▼
             [提取内点 Inliers]                                 [提取外点 Outliers]
                     │                                                 │
                     ▼                                                 ▼
          [地面点 ground_pcd]                               [非地面点 non_ground_pcd]
                     │                                                 │
                     └────────────────────────┬────────────────────────┘
                                              ▼
                             [序列化并发布至对应 ROS 话题]
```

#### 关键实现细节：
- 调用 Open3D 的 `pcd->SegmentPlane(distance_threshold, ransac_n, num_iterations)` 函数。
- 算法返回拟合出的平面方程参数 $\mathbf{p} = [a, b, c, d]^T$ 和地面内点索引列表 `inliers`。
- 调用 `pcd->SelectByIndex(inliers, false)` 提取地面点，调用 `pcd->SelectByIndex(inliers, true)` 提取非地面点。
- **异常退化保护**：若平面拟合失败（如点云数量不足或异常），系统会自动捕捉异常，并将全量点归为非地面点发布，避免流水线崩溃。

---

### 2.2 GMZ 网格最低点节点 (`gmz_node`)

GMZ 节点采用了高效率的网格空间统计滤波算法（时间复杂度 $O(N)$），特别适用于大场景 LiDAR 数据的地面快速提取与降采样。工作流程如下：

```
[ROS 消息 /io/raw_pointcloud] ──► [反序列化为 Open3D PointCloud]
                                              │
                                              ▼
                            [计算 X, Y 全局最小值 min_x, min_y]
                                              │
                                              ▼
                            [网格计算与二元哈希 (grid_x, grid_y)]
                                              │
                       ┌──────────────────────┴──────────────────────┐
                       ▼                                             ▼
       [第一遍遍历：网格最低点哈希映射]               [第二遍遍历：高差阈值判定]
      `grid_min_pt_idx_map[key] = min_z_idx`       `pt.z > min_z + height_threshold`
                       │                                             │
                       ▼                                    ┌────────┴────────┐
             [生成稀疏地面点云]                              ▼                 ▼
            (用于后续快速泊松重建)                       [地面点云]       [非地面点云]
                       │                                    │                 │
                       └──────────────────────┬─────────────┴─────────────────┘
                                              ▼
                              [发布 ROS 消息 & 可选 PLY 本地保存]
```

#### 关键实现细节：
1. **坐标归一化与哈希映射**：
   通过找寻点云在 $X$ 和 $Y$ 轴上的最小值 $x_{\min}, y_{\min}$，将点划分到大小为 `grid_size` 的 2D 网格中：
   $$\text{grid\_x} = \lfloor (x - x_{\min}) / \text{grid\_size} \rfloor, \quad \text{grid\_y} = \lfloor (y - y_{\min}) / \text{grid\_size} \rfloor$$
   使用 64 位整数高效组合网格坐标为联合 Hash Key：
   $$\text{key} = (\text{uint64\_t}(\text{grid\_x}) \ll 32) \mid \text{uint64\_t}(\text{grid\_y})$$
2. **稀疏地面点骨架提取 (Sparse Ground Points)**：
   第一遍遍历中，记录每个网格单元内部 $Z$ 坐标最小的点。提取出的点集构成了稀疏地面点云 `sparse_ground_pcd`。该点集保留了完整的地面地形起伏特征，同时将点云数量缩减了几个数量级，极大提升了后续地形三维重建（如泊松重建）的计算效率。
3. **高差阈值分割**：
   第二遍遍历中，若某个点的 $Z$ 坐标满足 $z > z_{\text{grid\_min}} + \text{height\_threshold}$，则判定该点为悬浮在地面之上的**非地面点**；否则判定为**地面点**。
4. **数据保存功能**：
   节点支持将分割好的稀疏地面点 (`sparse_gpcd_*.ply`)、全量地面点 (`full_gpcd_*.ply`) 及非地面点 (`nongpcd_*.ply`) 本地保存，文件名自带 ROS 消息时间戳。

---

## 3. ROS 2 通信接口

### 3.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/io/raw_pointcloud` | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 输入的原始点云数据 |

### 3.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/gs/ground_pointcloud` | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 分割出的地面点云（GMZ节点发布稀疏地面点云） |
| `/gs/non_ground_pointcloud` | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 分割出的非地面点云（送入后续聚类节点） |

---

## 4. 参数配置说明

配置文件位于 [ground_segmentation/config/gs_config.yaml](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/ground_segmentation/config/gs_config.yaml)。

### 4.1 RANSAC 节点参数 (`ransac_node`)

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/io/raw_pointcloud"` | 订阅的原始点云话题 |
| `publish_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 发布地面点云的话题 |
| `publish_non_ground_topic` | `string` | `"/gs/non_ground_pointcloud"` | 发布非地面点云的话题 |
| `distance_threshold` | `double` | `13.0` | RANSAC 平面点到平面的最大距离门限（单位：米） |
| `ransac_n` | `int` | `3` | 拟合平面所需的随机采样点个数 |
| `num_iterations` | `int` | `100` | RANSAC 随机抽样最大迭代次数 |

### 4.2 GMZ 节点参数 (`gmz_node`)

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/io/raw_pointcloud"` | 订阅的原始点云话题 |
| `publish_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 发布地面点云的话题 |
| `publish_non_ground_topic` | `string` | `"/gs/non_ground_pointcloud"` | 发布非地面点云的话题 |
| `grid_size` | `double` | `20.0` | 2D 网格边长大小（单位：米） |
| `height_threshold` | `double` | `0.8` | 判定为非地面点的高差阈值（单位：米） |
| `data_saved_dir` | `string` | `"/home/..."` | 点云文件保存的本地目标目录 |
| `enable_sparse_ground_points_saving` | `bool` | `false` | 是否开启保存稀疏地面点 PLY 文件 |
| `enable_ground_points_saving` | `bool` | `false` | 是否开启保存全量地面点 PLY 文件 |
| `enable_non_ground_points_saving` | `bool` | `false` | 是否开启保存非地面点 PLY 文件 |

---

## 5. 启动与使用方式

### 5.1 启动 RANSAC 地面分割节点
```bash
ros2 launch ground_segmentation gs_ransac.launch.py
```

### 5.2 启动 GMZ 网格最低点地面分割节点（推荐）
```bash
ros2 launch ground_segmentation gs_gmz.launch.py
```

---

## 6. 文件结构与索引

```
src/ground_segmentation/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── gs_config.yaml               # 地面分割节点参数配置文件
├── launch/
│   ├── gs_ransac.launch.py          # RANSAC 节点 Launch 脚本
│   └── gs_gmz.launch.py             # GMZ 节点 Launch 脚本
└── src/
    ├── ransac_node.cpp              # RANSAC 算法节点实现
    └── gmz_node.cpp                 # GMZ 算法节点实现
```
