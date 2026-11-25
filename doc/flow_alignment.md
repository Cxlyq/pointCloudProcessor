# flow_alignment 节点说明文档

`flow_alignment` 是点云处理流水线中的白模流式帧间刚体位姿配准功能包。它位于白模重建节点（`reconstruction`）与末端可视化节点（`visualization`）之间，采用基于滑动窗口（Sliding Window Map）的鲁棒点到面截断迭代最近点算法（Trimmed Point-to-Plane ICP），并支持结合载机导航状态（`PlaneState`）进行精确时间戳配对与指数平滑门控，将每帧局部坐标系下的离散重建白模实时对齐到全局统一世界坐标系中，为下游呈现无缝流动的连续三维白模提供高精度的刚体变换。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `flow_alignment` |
| **可执行文件** | `flow_alignment_node` |
| **主 C++ 类** | `FlowAlignmentNode` ([`flow_alignment_node.cpp`](../src/flow_alignment/src/flow_alignment_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `std_msgs`, `geometry_msgs`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **零侵入式模块化嵌入**：严格复用已有 ROS 2 消息体系（`pc_msgs/msg/O3DMesh`、`pc_msgs/msg/PlaneState` 与 `std_msgs/msg/Float64MultiArray`），不修改上游重建节点的核心结构，通过订阅话题配置实现即插即用。
2. **载机导航状态与白模时间戳精确配对 (`use_gps_init`)**：支持订阅 `PlaneState`，以消息采集时间戳（`header.stamp` 纳秒键）在双缓冲队列中严格配对白模与对应时刻的载机经纬高和航向角，自动将载机航迹投影到首帧局部参考系，为 ICP 提供高精度的刚体初值 `T0`。
3. **基于导航先验的门控与平滑修正机制**：当启用 `use_gps_init` 时，对 ICP 结果实施严格的质量与漂移门控（评估 `fitness`、`inlier_rmse` 以及相对于导航先验的平移与旋转偏差），通过 20% 指数移动平均（EMA / Slerp）更新全局修正量，彻底消除长航程累计漂移与单帧异常跳变。
4. **多尺度金字塔 Trimmed Point-to-Plane ICP**：采用基于中位数邻近距离的多尺度粗到细迭代（$8r \to 4r \to 2r$），结合法向量方向一致性约束（$< 60^\circ$）和固定比例残差截断（保留最优 70% 对应对），强力抵抗大面积动态遮挡、局部重建空洞与点云边缘噪声。
5. **基于滑动窗口的局部子图融合 (Sliding Window Map)**：维护固定容量（默认 4 帧）的局部子图，自动合并并进行体素网格下采样（3.0 m），在维持充足重叠区域的同时将单帧配准耗时稳定控制在 10~50 ms 以内。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
  /reconstruction/white_mesh (O3DMesh)        /io/plane/state (PlaneState, 可选)
                   │                                         │
                   └────────────────────┬────────────────────┘
                                        ▼
                   ┌─────────────────────────────────────────┐
                   │           flow_alignment_node           │
                   │                                         │
                   │ 1. 纳秒时间戳精确配对 (use_gps_init)    │
                   │ 2. 导航位姿转首帧参考系初值 T0          │
                   │ 3. 滑动窗口地图体素下采样 (Window Map)  │
                   │ 4. 多尺度 Trimmed Point-to-Plane ICP    │
                   │ 5. 候选位姿门控与 EMA 平滑修正          │
                   │ 6. 网格刚体变换与滑动窗口更新 (FIFO)    │
                   └────────────────────┬────────────────────┘
                                        │
         ┌──────────────────────────────┼──────────────────────────────┐
         ▼                              ▼                              ▼
/flow/aligned_mesh           /flow/registration_stats       /flow/aligned_pointcloud
(pc_msgs/msg/O3DMesh)        (Float64MultiArray 统计)       (O3DPointCloud 调试地图)
         │                              │
         ▼                              ▼
  [visualization]                [系统监控 / 记录]
 (连续流动渲染展示)
```

### 2.2 导航先验与时间戳配对机制
当 `use_gps_init: true` 时：
1. **严格时间戳配对 (`pair_messages`)**：节点分别缓存最多 32 帧 `PlaneState`（`nav_pending_`）与 8 帧 `O3DMesh`（`mesh_pending_`），以 `stamp_key = sec * 1e9 + nanosec` 精确匹配同一采集时刻的数据，不受网络回调到达顺序乱序的影响。
2. **首帧局部参考系转换**：以首帧导航位姿 `nav_origin_` 为世界参考系原点，利用纬度和航向角将后续载机的经纬高与航向变化转换为平面刚体变换 `current_nav_`；首帧直接以此导航位姿（或单位阵）锚定世界系。
3. **先验初值注入与门控平滑 (`accept_candidate`)**：
   * 求解 ICP 前，将累积修正量叠加到当前导航位姿上构成高精初值 `T0`。
   * ICP 求解后，仅当候选变换满足全部门控阈值——`fitness >= 0.35`、`inlier_rmse <= 50.0`、相对当前导航平移偏差 $\le 35.0\text{ m}$ 且旋转偏差 $\le 5.0^\circ$——才予以采纳，并以 $0.2$ 权重平滑更新平移修正量 `correction_`（`0.8 * correction + 0.2 * shift`）与旋转四元数 `rotation_`（`slerp(0.2, turn)`）。若某帧点云稀疏导致 ICP 退化，则自动回退到导航位姿叠加历史稳态修正量，确保轨迹稳定不发散。

### 2.3 多尺度 Trimmed Point-to-Plane ICP 核心流程
1. **网格预处理**：将 `O3DMesh` 转为带法向的源点云 `src`；若消息缺失法向量，则使用 KD 树估计并强制定向朝向 $+z$ 轴。
2. **滑动窗口子图构建**：合并窗口内最近 $W$ 帧（`window_size: 4`）已对齐网格顶点，并施加体素下采样（`icp_voxel_downsample: 3.0m`）。
3. **金字塔粗到细配准**：
   * 在源点云与窗口子图之间采样估算最近邻中位数距离 $r$，依次在搜索半径 $d_{\max} \in \{8r, 4r, 2r\}$ 三个尺度下迭代求解；
   * 过滤法向量夹角 $> \theta_{\text{normal}}$（默认 $60^\circ$）的错误对应点对；
   * 对残差升序排序并截断保留前 $\alpha$ 比例（`alpha: 0.7`，最少 10 对），使用 `TransformationEstimationPointToPlane` 求解增量刚体变换 $\Delta T$；
   * 当平移增量 $< 10^{-4}\text{ m}$ 且旋转增量 $< 10^{-3\circ}$ 时提前收敛。
4. **网格变换与发布**：调用 `aligned->Transform(res.T)` 完成顶点与法线刚体变换（仅在未由 Open3D 自动旋转的分支中手动旋转法线），随后发布结果。

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
| `sub_mesh_topic` (默认 `/reconstruction/white_mesh`) | [`pc_msgs/msg/O3DMesh`](../src/pc_msgs/msg/O3DMesh.msg) | 上游重建节点发布的单帧白模网格 |
| `plane_state_topic` (默认 `/io/plane/state`) | [`pc_msgs/msg/PlaneState`](../src/pc_msgs/msg/PlaneState.msg) | 载机实时 GPS 位置与航向角状态（当 `use_gps_init: true` 时订阅） |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `pub_aligned_mesh_topic` (默认 `/flow/aligned_mesh`) | [`pc_msgs/msg/O3DMesh`](../src/pc_msgs/msg/O3DMesh.msg) | 经刚体对齐到世界坐标系下的连续白模网格（供可视化） |
| `pub_stats_topic` (默认 `/flow/registration_stats`) | `std_msgs/msg/Float64MultiArray` | 配准质量监控统计数据（10 维数组） |
| `pub_map_pc_topic` (默认 `/flow/aligned_pointcloud`) | [`pc_msgs/msg/O3DPointCloud`](../src/pc_msgs/msg/O3DPointCloud.msg) | 滑动窗口合并地图采样点云（留空 `""` 则关闭以节省带宽） |

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

配置文件位于 [`src/flow_alignment/config/flow_alignment_params.yaml`](../src/flow_alignment/config/flow_alignment_params.yaml)。

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_mesh_topic` | `string` | `"/reconstruction/white_mesh"` | 输入白模网格主题 |
| `plane_state_topic` | `string` | `"/io/plane/state"` | 载机导航状态订阅主题（`use_gps_init: true` 时生效） |
| `pub_aligned_mesh_topic` | `string` | `"/flow/aligned_mesh"` | 对齐后白模发布主题 |
| `pub_map_pc_topic` | `string` | `"/flow/aligned_pointcloud"` | 调试地图点云发布主题（设为空串 `""` 禁用） |
| `pub_stats_topic` | `string` | `"/flow/registration_stats"` | 统计诊断发布主题 |
| `window_size` | `int` | `4` | 累积配准的滑动窗口最大帧数 |
| `icp_voxel_downsample` | `double` | `3.0` | 窗口地图体素下采样边长（米） |
| `alpha` | `double` | `0.7` | Trimmed ICP 排序后保留的最优残差比例 |
| `icp_max_iter_per_level` | `int` | `40` | 金字塔每一尺度的最大迭代次数 |
| `icp_normal_angle_deg` | `double` | `60.0` | 对应点法向量允许的最大夹角阈值（度） |
| `use_gps_init` | `bool` | `true` | 是否订阅载机状态，按采集时间戳配对提供导航初值并启用 EMA 门控修正 |
| `align_scale` | `double` | `1.0` | 缩放因子 |
| `normal_radius` | `double` | `20.0` | 兜底法线估计邻域半径（米） |
| `normal_max_nn` | `int` | `15` | 兜底法线估计最大邻近点数 |
| `data_saved_dir` | `string` | `""` | 落盘保存目录（例如直接供 CloudCompare 查看） |
| `enable_aligned_saving` | `bool` | `false` | 是否保存对齐后的每帧网格为 `.ply` 文件 |
| `enable_map_saving` | `bool` | `false` | 是否保存滑动窗口合并后的地图点云为 `.ply` 文件 |

---

## 6. 常用配置与启动脚本

### 6.1 启动脚本：[`flow_alignment.launch.py`](../src/flow_alignment/launch/flow_alignment.launch.py)
拉起配准节点并自动加载 [`flow_alignment_params.yaml`](../src/flow_alignment/config/flow_alignment_params.yaml)。

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
├── README.md                              # 节点设计说明与验证记录
├── config/
│   └── flow_alignment_params.yaml         # 滑动窗口、导航配对与 Trimmed ICP 超参
├── launch/
│   └── flow_alignment.launch.py           # ROS 2 Launch 启动脚本
└── src/
    └── flow_alignment_node.cpp            # 导航配对、点到面多尺度 Trimmed ICP 与门控实现
```
