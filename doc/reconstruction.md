# reconstruction 节点说明文档

`reconstruction` 是点云处理流水线中的三维白模表面重建与模型融合核心功能包。它接收来自上游地面分割的稀疏地面点与聚类模块输出的聚类点云，通过时间同步器（`ApproximateTime`）实现帧同步；采用**泊松曲面重建**（Poisson Surface Reconstruction）算法拟合连续高保真地形网格，结合**定向包围盒**（OBB, Oriented Bounding Box）生成悬浮物体三维白模，并支持通过 GPS 空间检索动态注入**真实高压铁塔 CAD 精细模型**，最终融合成完整的单帧三维白模网格（`O3DMesh`）。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `reconstruction` |
| **可执行文件** | 1. `reconstruction_g_poisson_ng_obb_node` (常规地形泊松+OBB白模，默认推荐)<br>2. `reconstruction_g_poisson_ng_obb_pt_node` (融合高压塔 CAD 模型的高阶全要素节点)<br>3. `reconstruction_g_poisson_ng_qh_node` (泊松地面+凸包 QuickHull)<br>4. `reconstruction_g_dln_ng_qh_node` (2.5D Delaunay 地面+凸包)<br>5. `reconstruction_node` (基础通用节点) |
| **主 C++ 类** | `ReconstructionPoissonOBBNode` ([reconstruction_g_poisson_ng_obb_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/src/reconstruction_g_poisson_ng_obb_node.cpp))<br>`ReconstructionPoissonOBBPTNode` ([reconstruction_g_poisson_ng_obb_pt_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/src/reconstruction_g_poisson_ng_obb_pt_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `message_filters`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **多路异构点云时间戳毫秒级近似同步**：使用 `message_filters::sync_policies::ApproximateTime` 严格对齐地面点流与非地面聚类点流（包含载机状态流），避免由于多节点处理耗时差异引发的数据乱序或帧撕裂。
2. **泊松表面重建与气泡伪影分位数裁剪**：对稀疏地面点估计向天法向量并构建八叉树隐式函数场，配合基于顶点密度的低分位数（`density_quantile`）自适应剔除，消除无点区域的外扩伪影。
3. **高压铁塔精细化 CAD 空间注入（PT 节点）**：集成载机 GPS 状态，通过 WGS84 到 ENU 局部切平面坐标转换，以毫米级精度将真实高压铁塔模型加载并旋转对齐植入地面，构建数字孪生场景。
4. **统一三角网格拓扑合并**：自动聚合地表连续曲面、建筑物 OBB 盒子及铁塔几何体，统一展平为 ROS 2 紧凑网格（`pc_msgs/msg/O3DMesh`），直接供给流式配准与渲染端。

---

## 2. 核心机制与算法原理

### 2.1 整体架构图

```
     /gs/ground_pointcloud            /clustering/clustered_pointcloud     /io/plane/state (可选)
       (O3DPointCloud)                    (ClusteredPointCloud)                 (PlaneState)
              │                                      │                              │
              └──────────────────┬───────────────────┘                              │
                                 ▼                                                  │
                ┌──────────────────────────────────┐                                │
                │  message_filters 近似时间同步器  │                                │
                └────────────────┬─────────────────┘                                │
                                 │                                                  ▼
                                 ▼                                     ┌─────────────────────────┐
             ┌──────────────────────────────────────────────┐          │  QueryNearbyTowers.srv  │
             │   泊松重建 + OBB 白模生成 (reconstruction)   │ ◄─────── │  (查询附近铁塔 GPS 坐标)│
             │                                              │          └─────────────────────────┘
             │  1. 地面法向量估计与八叉树泊松曲面重建       │
             │  2. 分位数密度裁剪 (过滤假边界)              │
             │  3. 各簇点集 OBB 最小外接盒网格化 (12个面)   │
             │  4. (可选) GPS转ENU高压铁塔 CAD 模型平移合并 │
             │  5. 全场景拓扑缝合与颜色/法向统一生成        │
             └──────────────────────┬───────────────────────┘
                                    │
                                    ▼
                 /reconstruction/white_mesh (O3DMesh)
                                    │
                                    ▼
                            [flow_alignment]
                          (ICP 流式全局帧间配准)
```

### 2.2 地面泊松曲面重建流程
1. **法线估计与方向一致性**：对来自 `gmz_node` 的稀疏地面点建立 KD 树，在半径 $R$（`normal_radius`）内搜寻最多 $K$ 个邻近点（`normal_max_nn`）计算协方差矩阵与特征向量作为法向；强制约束法向朝向 $+z$ 轴（天顶方向）：若 $n_z < 0$ 则反向。
2. **泊松方程求解**：构建深度为 $D$（`poisson_depth`，默认 6~8）的自适应八叉树，计算指示函数的梯度并求解泊松方程提取等值面，生成连续平滑的三角形地表。
3. **密度分位数修剪（Density Trimming）**：泊松算法在边界空白处会向外凸出闭合形成无数据依托的“大泡泡”。节点获取每个顶点的密度分布，计算全局分位数阈值 $Q_{\alpha}$（如 $\alpha = 0.06$），将密度小于 $Q_{\alpha}$ 的游离顶点及相连三角形彻底裁剪删除。

### 2.3 障碍物 OBB 定向外接盒生成
针对非地面聚类点云中每个有效类别 $l \ge 0$：
* 若包含的点数 $\ge \text{min\_cluster\_size}$（默认 15 点），通过主成分分析（PCA）构建最小体积的定向包围盒 `OrientedBoundingBox`；
* 提取 OBB 的 8 个三维顶点，按右手螺旋法则生成 6 个矩形面共 12 个三角形的三角网格，赋予高亮颜色。

### 2.4 高压铁塔 GPS 到 ENU 局部空间投影与地面依附估高
在 `reconstruction_g_poisson_ng_obb_pt_node` 中：
1. **坐标变换**：以载机经纬度高 $(lat_0, lon_0, alt_0)$ 为原点，根据 WGS84 椭球几何常数将铁塔经纬度转换为局部东-北-天（ENU, East-North-Up）三维坐标：
   $$(x_{\text{east}}, y_{\text{north}}, z_{\text{up}}) = \text{gps\_to\_enu}(lat_0, lon_0, alt_0, lat_t, lon_t, alt_t)$$
2. **地面高度依托估计**：若启用 `use_estimated_powertower_z: true`，在重建地表面网格中检索距离高压塔水平坐标最近的地表三角面，取其高程作为铁塔底座绝对 $Z$，避免铁塔悬空或深埋地下。
3. **CAD 网格平移合并**：将预先加载并预旋的高压塔网格整体平移至目标位置，拼装入最终白模。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在生成并发布白模网格后，输出标准化日志：

```text
[STAGE-DONE] reconstruction: Frame %zu processed | Vertices: %zu, Triangles: %zu | Time: %.2f ms
```

* **示例输出**：
  ```text
  [reconstruction_g_poisson_ng_obb_node-4] [STAGE-DONE] reconstruction: Frame 1 processed | Vertices: 15420, Triangles: 29810 | Time: 212.05 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `sub_ground_topic` (默认 `/gs/ground_pointcloud`) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 上游提取出的稀疏地面点云 |
| `sub_cluster_topic` (默认 `/clustering/clustered_pointcloud`) | [pc_msgs/msg/ClusteredPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/ClusteredPointCloud.msg) | 上游聚类完成的障碍物点云及标签 |
| `sub_plane_state_topic` (仅 PT 节点，默认 `/io/plane/state`) | [pc_msgs/msg/PlaneState](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/PlaneState.msg) | 载机 GPS 与航向状态 |

### 4.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `pub_mesh_topic` (默认 `/reconstruction/white_mesh`) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 输出的当前帧合并白模三角网格（顶点、三角面、法线、颜色） |

### 4.3 客户端服务调用 (Clients, 仅 PT 节点)

| 服务名称 | 服务类型 | 说明 |
| :--- | :--- | :--- |
| `tower_query_service` (默认 `/io/query_nearby_towers`) | [pc_msgs/srv/QueryNearbyTowers](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/srv/QueryNearbyTowers.srv) | 请求载机周围高压铁塔地理信息 |

---

## 5. 参数配置说明

配置文件位于 [`src/reconstruction/config/r_config.yaml`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/config/r_config.yaml)。

### 5.1 泊松+OBB 核心参数 (`reconstruction_g_poisson_ng_obb_node`)

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 地面点云订阅主题 |
| `sub_cluster_topic` | `string` | `"/clustering/clustered_pointcloud"` | 聚类点云订阅主题 |
| `pub_mesh_topic` | `string` | `"/reconstruction/white_mesh"` | 白模网格发布主题 |
| `min_cluster_size` | `int` | `15` | 生成 OBB 盒子的最小点数阈值（小于此点数当作噪点跳过） |
| `poisson_depth` | `int` | `6` | 泊松八叉树深度。推荐 6~8，深度越大越精细，计算耗时呈指数增加 |
| `density_quantile` | `double` | `0.06` | 泊松边界低密度三角形裁剪分位数（0.0~1.0） |
| `normal_radius` | `double` | `20.0` | 估计地面法向量的搜索球半径（米） |
| `normal_max_nn` | `int` | `15` | 估计法向量的最大邻域点数上限 |
| `data_saved_dir` | `string` | `""` | 调试用白模网格保存路径 |
| `enable_combined_mesh_saving`| `bool` | `false` | 是否将每帧合并后的完整白模保存为 `.ply` 文件 |

### 5.2 高压塔注入专属参数 (`reconstruction_g_poisson_ng_obb_pt_node`)

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_plane_state_topic` | `string` | `"/io/plane/state"` | 载机状态订阅主题 |
| `tower_query_service` | `string` | `"/io/query_nearby_towers"` | 铁塔查询服务名称 |
| `tower_query_radius_km` | `double` | `5.0` | 查询周边铁塔的地理半径（公里） |
| `sync_slop_sec` | `double` | `0.05` | 三路消息近似时间同步的最大容差窗口（秒） |
| `use_estimated_powertower_z` | `bool` | `true` | 是否根据重建地面曲面高程估计铁塔立足高度 |
| `tower_ground_search_radius` | `double` | `25.0` | 搜索地表支撑面的最大水平距离；无依托的塔不予显示 |
| `tower_model_path` | `string` | `"assets/.../powerTransmissionTower.ply"` | 铁塔三维 CAD 模型的本地绝对路径 |
| `tower_model_scale` | `double` | `1.0` | 模型几何缩放比例 |
| `tower_model_color` | `double[3]`| `[0.8, 0.5, 0.1]` | 铁塔模型渲染 RGB 颜色 |
| `tower_model_rotation_deg` | `double[9]`| `[1,0,0, 0,0,-1, 0,1,0]` | 铁塔插入前的 3x3 空间位姿旋转校正矩阵 |

---

## 6. 常用配置与启动脚本

### 6.1 常规泊松+OBB白模重建（主流水线默认）：[r_g_poisson_ng_obb.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_obb.launch.py)
用于单机或分布式环境下快速生成高质量地貌曲面与障碍物包围盒。

### 6.2 融合高压铁塔数字孪生重建：[r_g_poisson_ng_obb_pt.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_obb_pt.launch.py)
搭配 `io_simPlaneDataFlow.launch.py` 使用，提供具备电力铁塔数字孪生呈现的全要素白模。

### 6.3 其他算法组合脚本：
* [r_g_poisson_ng_qh.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_qh.launch.py)：泊松地面 + 障碍物凸包（QuickHull）
* [r_g_dln_ng_qh.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_dln_ng_qh.launch.py)：2.5D Delaunay 三角剖分地面 + 障碍物凸包

---

## 7. 启动与使用命令参考

```bash
# 1. 启动默认 Poisson + OBB 白模重建节点
ros2 launch reconstruction r_g_poisson_ng_obb.launch.py

# 2. 启动带高压铁塔数字孪生注入的高级重建节点
ros2 launch reconstruction r_g_poisson_ng_obb_pt.launch.py

# 3. 查看输出白模的话题频率与顶点面数
ros2 topic hz /reconstruction/white_mesh
```

---

## 8. 文件结构与索引

```
src/reconstruction/
├── CMakeLists.txt                              # 编译配置 (链接 Open3D, message_filters)
├── package.xml                                 # 节点依赖描述
├── config/
│   └── r_config.yaml                           # 重建与铁塔注入全套参数
├── launch/
│   ├── r_g_poisson_ng_obb.launch.py            # 默认泊松+OBB启动脚本
│   ├── r_g_poisson_ng_obb_pt.launch.py         # 融合铁塔 CAD 注入启动脚本
│   ├── r_g_poisson_ng_qh.launch.py             # 泊松+凸包启动脚本
│   ├── r_g_dln_ng_qh.launch.py                 # Delaunay+凸包启动脚本
│   └── r_reconstruction.launch.py             # 基础启动脚本
└── src/
    ├── reconstruction_g_poisson_ng_obb_node.cpp    # 泊松地面+OBB白模实现
    ├── reconstruction_g_poisson_ng_obb_pt_node.cpp # 泊松+OBB+高压塔注入实现
    ├── reconstruction_g_poisson_ng_qh_node.cpp     # 泊松+凸包实现
    ├── reconstruction_g_dln_ng_qh_node.cpp         # Delaunay+凸包实现
    └── reconstruction_node.cpp                     # 基础通用实现
```
