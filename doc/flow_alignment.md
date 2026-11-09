# flow_alignment 节点说明文档

`flow_alignment` 是点云处理流水线中的白模流式帧间刚体位姿配准功能包。它位于白模重建节点（`reconstruction`）与末端可视化节点（`visualization`）之间，采用基于滑动窗口（Sliding Window Map）的鲁棒点到面截断迭代最近点算法（Trimmed Point-to-Plane ICP），将每帧局部坐标系下的离散重建白模实时对齐到全局统一世界坐标系中，为下游呈现无缝流动的连续三维白模提供高精度的刚体变换。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `flow_alignment` |
| **可执行文件** | `flow_alignment_node` |
| **主 C++ 类** | `FlowAlignmentNode` ([flow_alignment_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/flow_alignment/src/flow_alignment_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `std_msgs`, `geometry_msgs`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **零侵入式模块化嵌入**：严格复用已有 ROS 2 消息体系（`pc_msgs/msg/O3DMesh` 与 `std_msgs/msg/Float64MultiArray`），不修改上游重建节点和下游可视化节点的核心结构，通过修改订阅话题实现即插即用。
2. **多尺度金字塔 Trimmed Point-to-Plane ICP**：采用基于中位数邻近距离的多尺度粗到细迭代（$8r \to 4r \to 2r$），结合法向量方向一致性约束（$< 60^\circ$）和固定比例残差截断（保留最优 70% 对应对），强力抵抗大面积动态遮挡、局部重建空洞与点云边缘噪声。
3. **基于滑动窗口的局部子图融合 (Sliding Window Map)**：维护固定容量（默认 4 帧）的局部子图，自动合并并进行体素网格下采样（3.0 m），既能维持足够的几何重叠区域以防配准漂移，又能将单帧配准耗时稳定控制在 10~50 ms 以内。
4. **配准诊断指标实时回传**：每帧以标准数组形式发布包含 `fitness`、`overlap`、`inlier_rmse`、平移增量 $(t_x, t_y, t_z)$ 及旋转角度变化量在内的 10 维诊断指标，便于监控系统漂移。

---

## 2. 核心机制与算法原理

### 2.1 整体架构图

```
                  /reconstruction/white_mesh (O3DMesh)
                                   │
                                   ▼
                   ┌───────────────────────────────┐
                   │      flow_alignment_node      │
                   │                               │
                   │ 1. 消息转 Open3D TriangleMesh │
                   │ 2. 网格顶点提取与法向量估计   │
                   │ 3. 滑动窗口地图体素下采样     │
                   │ 4. 多尺度 Trimmed ICP 配准    │
                   │ 5. 网格顶点与法线世界系变换   │
                   │ 6. 滑动窗口更新 (FIFO)        │
                   └───────────────┬───────────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         ▼                         ▼                         ▼
/flow/aligned_mesh      /flow/registration_stats   /flow/aligned_pointcloud
(pc_msgs/msg/O3DMesh)   (Float64MultiArray 统计)   (O3DPointCloud 调试地图)
         │                         │
         ▼                         ▼
  [visualization]           [系统监控 / 记录]
 (连续流动渲染展示)
```

### 2.2 配准算法核心流程
1. **输入网格预处理**：
   将 `O3DMesh` 反序列化为 Open3D 网格；若上游未包含法向量，使用 KD 树进行局部法向估计并强制定向朝向 $+z$ 轴。
2. **首帧世界系锚定**：
   收到第 0 帧网格时，设定其世界变换矩阵为单位阵 $T_0 = \mathbf{I}_{4 \times 4}$，直接存入滑动窗口并输出。
3. **滑动窗口子图构建**：
   窗口保持最近的 $W$ 帧（`window_size: 4`）。将窗口内已对齐至世界系的各帧网格顶点合并为参考点云地图，并施加体素滤波（`icp_voxel_downsample: 3.0m`），保证参考点分布均匀且点数可控。
4. **多尺度金字塔粗到细配准**：
   * 在当前源帧与参考子图之间随机采样 2000 个点，计算最近邻欧氏距离中位数 $r$ 作为特征尺度基准。
   * 依次在搜索半径 $d_{\max} \in \{8r, 4r, 2r\}$ 三个尺度下迭代求解：
     * **法向一致性硬过滤**：仅当源点法线与目标点法线夹角 $\le \theta_{\text{normal}}$（默认 $60^\circ$）时保留为候选对应点对；
     * **Trimmed 残差截断**：将所有候选点对按欧氏距离升序排序，只保留残差最小的前 $\alpha$ 比例（`alpha: 0.7`，70%），滤除重叠区之外的离群点；
     * **SVD 点到面求解**：使用 `TransformationEstimationPointToPlane` 求解最小化垂直于目标面法向量残差的刚体变换增量 $\Delta T$；
     * 累加全局变换：$T_{w} \leftarrow \Delta T \cdot T_w$；
     * 提前收敛检测：当平移增量 $< 10^{-4}\text{ m}$ 且旋转增量 $< 10^{-3\circ}$ 时提前跳出本层迭代。
5. **网格变换与发布**：
   调用 `aligned_mesh->Transform(T_w)` 对顶点与法向量完成整体刚体旋转平移，继承时间戳发布给终端。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在完成配准与发布后，输出标准化日志：

```text
[STAGE-DONE] flow_alignment: Frame %zu processed | Aligned vertices: %zu | Time: %.2f ms
```

* **示例输出**：
  ```text
  [flow_alignment_node-5] [STAGE-DONE] flow_alignment: Frame 1 processed | Aligned vertices: 15420 | Time: 18.35 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `sub_mesh_topic` (默认 `/reconstruction/white_mesh`) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 上游重建节点发布的单帧白模网格 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `pub_aligned_mesh_topic` (默认 `/flow/aligned_mesh`) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 经刚体对齐到世界坐标系下的连续白模网格（供可视化） |
| `pub_stats_topic` (默认 `/flow/registration_stats`) | `std_msgs/msg/Float64MultiArray` | 配准质量监控统计数据（10 维数组） |
| `pub_map_pc_topic` (默认 `/flow/aligned_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 滑动窗口合并地图采样点云（留空 `""` 则关闭以节省带宽） |

#### `pub_stats_topic` 统计字段定义：
1. `fitness`：满足距离阈值且参与求解的有效重叠点比例 ($[0.0, 1.0]$)
2. `overlap`：几何重叠度估值
3. `inlier_rmse`：配准内点均方根误差 (米)
4. `tx`, `ty`, `tz`：世界坐标系下的平移向量 (米)
5. `rot_deg`：自首帧以来的累计三维旋转角度标量 (度)
6. `src_pts`：源帧有效顶点采样数
7. `map_pts`：滑动窗口地图体素点数
8. `seq`：处理的帧序号

---

## 5. 参数配置说明

配置文件位于 [`src/flow_alignment/config/flow_alignment_params.yaml`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/flow_alignment/config/flow_alignment_params.yaml)。

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_mesh_topic` | `string` | `"/reconstruction/white_mesh"` | 输入白模网格主题 |
| `pub_aligned_mesh_topic` | `string` | `"/flow/aligned_mesh"` | 对齐后白模发布主题 |
| `pub_map_pc_topic` | `string` | `"/flow/aligned_pointcloud"` | 调试地图点云发布主题（设为空串 `""` 禁用） |
| `pub_stats_topic` | `string` | `"/flow/registration_stats"` | 统计诊断发布主题 |
| `window_size` | `int` | `4` | 累积配准的滑动窗口最大帧数 |
| `icp_voxel_downsample` | `double` | `3.0` | 窗口地图体素下采样边长（米） |
| `alpha` | `double` | `0.7` | Trimmed ICP 排序后保留的最优残差比例 |
| `icp_max_iter_per_level` | `int` | `40` | 金字塔每一尺度的最大迭代次数 |
| `icp_normal_angle_deg` | `double` | `60.0` | 对应点法向量允许的最大夹角阈值（度） |
| `use_gps_init` | `bool` | `false` | 是否使用 GPS 粗估计初值 |
| `normal_radius` | `double` | `20.0` | 兜底法线估计邻域半径（米） |
| `normal_max_nn` | `int` | `15` | 兜底法线估计最大邻近点数 |
| `data_saved_dir` | `string` | `""` | 落盘保存目录（例如直接供 CloudCompare 查看） |
| `enable_aligned_saving` | `bool` | `false` | 是否保存对齐后的每帧网格为 `.ply` 文件 |
| `enable_map_saving` | `bool` | `false` | 是否保存滑动窗口合并后的地图点云为 `.ply` 文件 |

---

## 6. 常用配置与启动脚本

### 6.1 启动脚本：[flow_alignment.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/flow_alignment/launch/flow_alignment.launch.py)
拉起配准节点并加载参数。

---

## 7. 启动与使用命令参考

```bash
# 1. 独立启动流式配准节点
ros2 launch flow_alignment flow_alignment.launch.py

# 2. 命令行实时查看当前配准质量与位姿增量
ros2 topic echo /flow/registration_stats

# 3. 检查对齐后网格输出频率
ros2 topic hz /flow/aligned_mesh
```

---

## 8. 文件结构与索引

```
src/flow_alignment/
├── CMakeLists.txt                         # 构建配置 (链接 Open3D, Eigen3)
├── package.xml                            # 包依赖元信息
├── README.md                              # 原始配准验证记录
├── config/
│   └── flow_alignment_params.yaml         # 滑动窗口与 Trimmed ICP 超参
├── launch/
│   └── flow_alignment.launch.py           # ROS 2 Launch 启动脚本
└── src/
    └── flow_alignment_node.cpp            # 点到面多尺度 Trimmed ICP 配准节点实现
```
