# reconstruction 节点说明文档

`reconstruction` 是点云处理流水线中的三维白模表面重建与模型融合核心功能包。它接收来自上游地面分割的稀疏地面点与聚类模块输出的聚类点云，通过时间同步器（`ApproximateTime`）实现帧同步；采用**泊松曲面重建**（Poisson Surface Reconstruction）算法拟合连续高保真地形网格并构建 2D 空间高程索引，在此基准上完成**聚类点云地面以下点剔除**与**定向包围盒（OBB）防穿模约束**，并支持通过 GPS 空间检索动态注入**真实高压铁塔 CAD 精细模型**，最终融合成完整的单帧三维白模网格（`O3DMesh`）。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `reconstruction` |
| **可执行文件** | 1. `reconstruction_g_poisson_ng_obb_pt_node` (泊松地面 + 防穿模 OBB + 高压塔 CAD 融合，默认推荐)<br>2. `reconstruction_g_poisson_ng_obb_node` (常规泊松地面 + 防穿模 OBB 白模)<br>3. `reconstruction_g_poisson_ng_qh_node` (泊松地面 + 凸包 QuickHull)<br>4. `reconstruction_g_dln_ng_qh_node` (2.5D Delaunay 地面 + 凸包)<br>5. `reconstruction_node` (基础通用节点) |
| **主 C++ 类** | `ReconstructionPoissonOBBPTNode` ([reconstruction_g_poisson_ng_obb_pt_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/src/reconstruction_g_poisson_ng_obb_pt_node.cpp))<br>`ReconstructionPoissonOBBNode` ([reconstruction_g_poisson_ng_obb_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/src/reconstruction_g_poisson_ng_obb_node.cpp)) |
| **共享几何工具** | `TerrainHeightGrid` 等防穿模高程工具 ([terrain_height_grid.hpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/include/reconstruction/terrain_height_grid.hpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `message_filters`), `pc_msgs`, `Open3D`, `Eigen3` |

### 核心设计目标：
1. **多路异构点云时间戳毫秒级近似同步**：使用 `message_filters::sync_policies::ApproximateTime` 严格对齐地面点流、非地面聚类点流与载机状态流，避免由于多节点处理耗时差异引发的数据乱序或帧撕裂。
2. **泊松表面重建、气泡裁剪与快速 2D 地形网格索引**：对稀疏地面点估计向天法向量并求解泊松曲面，配合低密度分位数裁剪消除边界伪影；随后构建 $128\times 128$ 均匀 2D 网格空间索引 `TerrainHeightGrid`，将地面任意 $(x, y)$ 处高程查询加速至近似 $O(1)$。
3. **非地面白模地下点过滤与 OBB 防穿模抬升**：在生成各聚类簇白模前，利用泊松地面白模滤除落入地表以下或贴近地面的残余点与伪簇；构建 OBB 网格后，对其顶点与三角面重心实施高程防穿模钳位并剔除退化扁平盒，彻底消除障碍物白模与地面白模穿模现象。
4. **高压铁塔精细化 CAD 空间注入（PT 节点）**：集成载机 GPS 状态，通过 WGS84 到 ENU 局部切平面坐标转换，并复用 `TerrainHeightGrid` 获取铁塔底座立足地面高度，将真实高压铁塔模型旋转对齐植入地面。

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
             │   泊松重建 + 防穿模 OBB + 铁塔融合           │ ◄─────── │  (查询附近铁塔 GPS 坐标)│
             │                                              │          └─────────────────────────┘
             │  1. 地面法向量估计、泊松曲面重建与密度裁剪   │
             │  2. 构建 128×128 TerrainHeightGrid 高程索引  │
             │  3. 聚类簇地下点过滤 (filter_cluster_above)  │
             │  4. OBB 网格生成与顶点/面心防穿模抬升钳位    │
             │  5. (可选) GPS→ENU 铁塔落地高度估算与合并    │
             │  6. 统一三角网格拓扑合并与消息发布           │
             └──────────────────────┬───────────────────────┘
                                    │
                                    ▼
                 /reconstruction/white_mesh (O3DMesh)
                                    │
                                    ▼
                            [flow_alignment]
                          (ICP 流式全局帧间配准)
```

### 2.2 地面泊松曲面重建与 `TerrainHeightGrid` 2D 网格索引
1. **法线估计与方向一致性**：对来自 `gmz_node` 的稀疏地面点建立 KD 树，在半径 $R$（`normal_radius`）内搜寻最多 $K$ 个邻近点（`normal_max_nn`）计算法向；强制约束法向朝向 $+z$ 轴（若 $n_z < 0$ 则反向）。
2. **泊松方程求解与密度裁剪**：构建深度为 $D$（`poisson_depth`）的自适应八叉树求解泊松方程提取等值面，依据顶点密度分位数阈值 $Q_{\alpha}$（`density_quantile`）切除边界低密度伪影，并以地面点云 AABB 裁剪地形范围。
3. **`TerrainHeightGrid` 空间高程索引构建**：
   * 在泊松地面白模生成后，每帧将其 2D 范围划分为 $128 \times 128$ 均匀网格。
   * 预计算每个地形三角面片的 2D 包围盒、三顶点坐标与重心坐标分母，将三角面片注册至覆盖的网格单元；同时将地形顶点注册到所属单元。
   * 对任意 $(x, y)$ 查询 `get_height(x, y, max_fallback_dist)`：优先检查所在单元格三角面片做 2D 重心坐标包含测试与高程插值（多面重叠时取最大高程）；若未落入三角面片且在 `max_fallback_dist` 容差内，在邻近覆盖单元格中检索最近 4 个地形顶点做反距离加权（IDW）高程插值；超出范围则返回 `std::nullopt`。

### 2.3 基于地面白模的地下点过滤与 OBB 防穿模约束
在遍历非地面聚类点云中各有效类别 $l \ge 0$ 构建 OBB 白模时，执行两级几何约束：
1. **聚类簇点级过滤（`filter_cluster_above_terrain`）**：
   * 当 `enable_sub_ground_filter` 开启且 `TerrainHeightGrid` 有效时，逐点查询泊松地面高程 $Z_{\text{terrain}}(x, y)$（回退容差 `10.0 m`）；
   * 剔除所有满足 $z < Z_{\text{terrain}}(x, y) + \text{sub\_ground\_clearance}$ 的地面以下或紧贴地表的点（点在地面投影外无对应地面高程时予以保留）；
   * 过滤后若该簇剩余点数 `< min_cluster_size`，直接跳过该簇，消除贴地/地下伪包围盒。
2. **OBB 网格级防穿模钳位（`clamp_obb_mesh_above_terrain`）**：
   * 通过 PCA 对过滤后的簇构建最小体积定向包围盒 `OrientedBoundingBox` 并生成 8 顶点、12 三角面的白模网格；
   * 当 `enable_obb_ground_clamp` 开启且 `TerrainHeightGrid` 有效时：
     * **逐顶点抬升**：对 OBB 的 8 个顶点查询地面高程，若 $v_z < Z_{\text{terrain}}(v_x, v_y) + \text{obb\_ground\_clearance}$，将 $v_z$ 抬升至 $Z_{\text{terrain}} + \text{obb\_ground\_clearance}$；
     * **逐三角面重心抬升**：计算 12 个三角面的重心 $m$，若 $m_z < Z_{\text{terrain}}(m_x, m_y) + \text{obb\_ground\_clearance}$，将该三角面三个顶点的 $z$ 同步抬升对应差值 $\delta$，防止大跨度 OBB 底面跨过凸起山脊时中部穿模；
     * **退化扁平盒剔除**：检查修正后网格整体高度跨度 $z_{\max} - z_{\min}$，若 $< 0.5\text{ m}$ 则丢弃该压扁的退化包围盒；随后重新计算顶点法线并合入建筑网格。

### 2.4 高压铁塔 GPS 到 ENU 局部空间投影与地面依附估高
在 `reconstruction_g_poisson_ng_obb_pt_node` 中：
1. **坐标变换**：以载机经纬度高 $(lat_0, lon_0, alt_0)$ 为原点，根据 WGS84 椭球几何常数将铁塔经纬度转换为局部东-北-天（ENU, East-North-Up）三维坐标：
   $$(x_{\text{east}}, y_{\text{north}}, z_{\text{up}}) = \text{gps\_to\_enu}(lat_0, lon_0, alt_0, lat_t, lon_t, alt_t)$$
2. **地面高度依托估计**：若启用 `use_estimated_powertower_z: true`，直接通过 `TerrainHeightGrid` 在 `tower_ground_search_radius` 容差内查询铁塔水平坐标处的地面白模高程（若地面网格重建失败则回退检索 `ground_pcd` 最近点），作为铁塔底座绝对 $Z$，避免铁塔悬空或深埋地下。
3. **CAD 网格平移合并**：将启动时预加载、预旋并将底座对齐至 $Z=0$ 的高压塔模型整体平移至目标位置，拼装入最终白模。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
`reconstruction_g_poisson_ng_obb_pt_node` 在生成并发布白模网格后，输出标准化日志：

```text
[STAGE-DONE] reconstruction: Frame %zu processed | Vertices: %zu, Triangles: %zu | Time: %.2f ms
```

* **输出示例**：
  ```text
  [reconstruction_g_poisson_ng_obb_pt_node-4] [STAGE-DONE] reconstruction: Frame 1 processed | Vertices: 14423, Triangles: 20209 | Time: 780.86 ms
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

### 5.1 泊松 + 防穿模 OBB 核心参数 (`reconstruction_g_poisson_ng_obb_node` 与 PT 节点共有)

| 参数名称 | 类型 | 默认配置值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_ground_topic` | `string` | `"/gs/ground_pointcloud"` | 地面点云订阅主题 |
| `sub_cluster_topic` | `string` | `"/clustering/clustered_pointcloud"` | 聚类点云订阅主题 |
| `pub_mesh_topic` | `string` | `"/reconstruction/white_mesh"` | 白模网格发布主题 |
| `min_cluster_size` | `int` | `15` | 生成 OBB 盒子的最小点数阈值（小于此点数跳过） |
| `enable_sub_ground_filter` | `bool` | `true` | 是否使用泊松地面白模滤除聚类簇内地面以下及贴地碎点 |
| `sub_ground_clearance` | `double` | `0.5` | 聚类点相对泊松地面白模的最小净空阈值（米），低于 `Z_terrain + sub_ground_clearance` 的点被滤除 |
| `enable_obb_ground_clamp` | `bool` | `true` | 是否对生成的 OBB 网格顶点与三角面中心执行防穿模抬升钳位 |
| `obb_ground_clearance` | `double` | `0.1` | OBB 网格相对泊松地面白模的最小防穿模离地余量（米） |
| `poisson_depth` | `int` | `6` | 泊松八叉树深度。推荐 6~8，深度越大越精细，计算耗时呈指数增加 |
| `density_quantile` | `double` | `0.06` | 泊松边界低密度三角形裁剪分位数（0.0~1.0） |
| `normal_radius` | `double` | `20.0` | 估计地面法向量的搜索球半径（米） |
| `normal_max_nn` | `int` | `15` | 估计法向量的最大邻域点数上限 |
| `data_saved_dir` | `string` | 见配置 | 调试用白模网格保存路径 |
| `enable_terrain_mesh_saving` | `bool` | `false` | 是否将每帧地面白模保存为 `.ply` 文件 |
| `enable_architecture_mesh_saving` | `bool` | `false` | 是否将每帧非地面障碍物白模保存为 `.ply` 文件 |
| `enable_combined_mesh_saving` | `bool` | `false` | 是否将每帧合并后的完整白模保存为 `.ply` 文件 |

### 5.2 高压塔注入专属参数 (`reconstruction_g_poisson_ng_obb_pt_node`)

| 参数名称 | 类型 | 默认配置值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `sub_plane_state_topic` | `string` | `"/io/plane/state"` | 载机状态订阅主题 |
| `tower_query_service` | `string` | `"/io/query_nearby_towers"` | 铁塔查询服务名称 |
| `tower_query_radius_km` | `double` | `5.0` | 查询周边铁塔的地理半径（公里） |
| `sync_slop_sec` | `double` | `0.05` | 三路消息近似时间同步的最大容差窗口（秒） |
| `use_estimated_powertower_z` | `bool` | `true` | 是否根据重建地面曲面高程估计铁塔立足高度 |
| `tower_ground_search_radius` | `double` | `25.0` | 搜索地表支撑面的最大水平距离（米）；无依托的塔不予显示 |
| `tower_model_path` | `string` | `"assets/.../powerTransmissionTower_optimized9.ply"` | 铁塔三维 CAD 模型的本地绝对路径 |
| `tower_model_scale` | `double` | `1.0` | 模型几何缩放比例 |
| `tower_model_color` | `double[3]` | `[0.8, 0.5, 0.1]` | 铁塔模型渲染 RGB 颜色 |
| `tower_model_rotation_deg` | `double[9]` | `[1,0,0, 0,0,-1, 0,1,0]` | 铁塔插入前的 3×3 空间位姿旋转校正矩阵 |

---

## 6. 常用配置与启动脚本

### 6.1 融合高压铁塔数字孪生重建（主流水线默认）：[r_g_poisson_ng_obb_pt.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_obb_pt.launch.py)
搭配 `io_simPlaneDataFlow.launch.py` 使用，提供具备防穿模地形与电力铁塔数字孪生呈现的全要素白模。

### 6.2 常规泊松 + OBB 白模重建：[r_g_poisson_ng_obb.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_obb.launch.py)
用于无铁塔查询服务场景下快速生成高质量防穿模地貌曲面与障碍物包围盒。

### 6.3 其他算法组合脚本：
* [r_g_poisson_ng_qh.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_poisson_ng_qh.launch.py)：泊松地面 + 障碍物凸包（QuickHull）
* [r_g_dln_ng_qh.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/reconstruction/launch/r_g_dln_ng_qh.launch.py)：2.5D Delaunay 三角剖分地面 + 障碍物凸包

---

## 7. 启动与使用命令参考

```bash
# 1. 启动带高压铁塔数字孪生注入的默认重建节点
ros2 launch reconstruction r_g_poisson_ng_obb_pt.launch.py

# 2. 启动常规 Poisson + OBB 白模重建节点
ros2 launch reconstruction r_g_poisson_ng_obb.launch.py

# 3. 查看输出白模的话题频率与顶点面数
ros2 topic hz /reconstruction/white_mesh
```

---

## 8. 文件结构与索引

```
src/reconstruction/
├── CMakeLists.txt                                      # 编译配置 (链接 Open3D, message_filters)
├── package.xml                                         # 节点依赖描述
├── config/
│   └── r_config.yaml                                   # 重建、防穿模与铁塔注入全套参数
├── include/
│   └── reconstruction/
│       ├── delaunator.hpp                              # 2.5D Delaunay 三角剖分头文件
│       └── terrain_height_grid.hpp                     # 2D 网格高程索引、地下点过滤与 OBB 防穿模工具
├── launch/
│   ├── r_g_poisson_ng_obb_pt.launch.py                 # 默认泊松+防穿模OBB+铁塔 CAD 启动脚本
│   ├── r_g_poisson_ng_obb.launch.py                    # 常规泊松+防穿模OBB启动脚本
│   ├── r_g_poisson_ng_qh.launch.py                     # 泊松+凸包启动脚本
│   ├── r_g_dln_ng_qh.launch.py                         # Delaunay+凸包启动脚本
│   └── r_reconstruction.launch.py                      # 基础启动脚本
└── src/
    ├── reconstruction_g_poisson_ng_obb_pt_node.cpp     # 泊松地面+防穿模OBB+高压塔注入实现
    ├── reconstruction_g_poisson_ng_obb_node.cpp        # 泊松地面+防穿模OBB实现
    ├── reconstruction_g_poisson_ng_qh_node.cpp         # 泊松+凸包实现
    ├── reconstruction_g_dln_ng_qh_node.cpp             # Delaunay+凸包实现
    └── reconstruction_node.cpp                         # 基础通用实现
```
