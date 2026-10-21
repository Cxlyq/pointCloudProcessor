# flow_alignment —— 白膜帧间配准 ROS 2 节点（正式嵌入版）

> 版本：v1.1（已嵌入并验证） · 2026-10-15
> 位置：远程 `/home/trf/Documents/codes/pointCloudProcessor/src/flow_alignment/`（已构建 `install/flow_alignment`）
>      本地镜像 `E:\aerial_cloud2mask\flow_alignment\`
> 约束：**只新增本包**，不改 `src/` 任何已有文件、不改任何已有节点/launch（visualization 接入走参数）。

---

## 0. 嵌入状态（2026-10-15 已验证）

- **构建**：`colcon build --packages-select pc_msgs flow_alignment` 成功（仅 Open3D CMP0072 弃用警告，与 reconstruction 包同源，无害）。
- **运行**：`ros2 run flow_alignment flow_alignment_node --ros-args --params-file .../flow_alignment_params.yaml` 正常，参数全部就位（window=4 voxel=3.0 alpha=0.70 iter=40 angle=60° gps=off）。
- **话题**：`ros2 topic list` 确认 `/flow/aligned_mesh`（O3DMesh）/ `/flow/aligned_pointcloud`（O3DPointCloud）/ `/flow/registration_stats`（Float64MultiArray）均已注册，`ros2 topic info -v` Publisher count=1。
- **链路验证**：
  - 合成 mesh 帧流式发布 → 节点逐帧实时处理（连续帧 2 s 节奏，处理 1–13 ms/帧，fitness 随帧变化合理，aligned_mesh/stats 实时回发）。
  - 端到端两帧：frame0 锚定 Identity ✓；frame1 配准执行、stats/mesh 输出 ✓。
  - rclpy 环回 1.6 ms、跨进程 1.0 ms —— 消息编解码/传输无问题。
- **环境注意点（非节点缺陷）**：该机 fastrtps 对新加入 participant 的 discovery 首次握手约需 60–150 s（节点启动后对新发布进程首帧延迟；同机 rclpy 环回毫秒级、节点运行稳定后流式帧实时）。真机场景各节点一起启动即可，若观察到"首帧迟迟不到"不是节点卡住，是 discovery 收敛，等待即可。
- **与现有框架的接入方式（零代码改动）**：visualization 节点订阅话题是参数化的（`declare_parameter subscribe_topic` 默认 `/reconstruction/white_mesh`），只需在启动 visualization 时把参数指向 `/flow/aligned_mesh` 即可显示配准后的流动白膜。reconstruction 节点无需任何改动（仍发布原始白膜）。
- 配准算法本身已在 8 套数据集 75 帧验证（fitness 全部 ≥0.84，轨迹不发散），见技术报告 PPT 第 9–12 页。

---

## 1. 干什么的（一句话）

把 `flow_test/demo_flow.cpp` 里已经定稿的**帧间配准**（Trimmed Point-to-Plane ICP + 滑动窗口）
从离线 demo 封装成一个 ROS 2 节点，插在「白膜重建 → 可视化」之间：
每收到一帧重建白膜 mesh，就把它配准到前几帧累积的世界系 map，
输出**已经对齐到同一坐标系**的白膜 mesh —— 下游可视化/查看器直接显示流动效果。

## 2. 消息对接（保证接得上现有流水线）

```
 /reconstruction/white_mesh ──► flow_alignment_node ──► /flow/aligned_mesh        (pc_msgs/msg/O3DMesh)
   (pc_msgs/msg/O3DMesh)      ▲ 订阅重建节点输出              ├► /flow/aligned_pointcloud (pc_msgs/msg/O3DPointCloud, 可选)
                               │                              └► /flow/registration_stats (std_msgs/Float64MultiArray)
```

- **输入**：`/reconstruction/white_mesh`（重建节点已发布，消息类型 `pc_msgs/msg/O3DMesh`，字段与重建节点写入一致：vertices/triangles/vertex_normals/vertex_colors/header）。
- **输出 1**：`/flow/aligned_mesh`（`O3DMesh`）——当前帧白膜顶点已乘配准变换 `T_w`，与世界系对齐。可视化节点若要显示流动白膜，把订阅话题指向它即可（若它的话题是参数化的，零代码改动；若硬编码，只改一行订阅话题）。
- **输出 2**：`/flow/aligned_pointcloud`（`O3DPointCloud`）——滑动窗口合并 map 的体素下采样点云，方便调试/查看（参数留空串即关闭）。
- **输出 3**：`/flow/registration_stats`（`std_msgs/Float64MultiArray`）——每帧 `[fitness, overlap, inlier_rmse, tx, ty, tz, rot_deg, src_pts, map_pts, seq]`，接记录/监控用，**不新增自定义消息**。

> 不依赖任何新消息定义，只复用 `pc_msgs`（已有）与 `std_msgs`（标准库）。

## 3. 算法（与 demo_flow.cpp 完全一致，仅包装）

每帧回调流程：

1. `O3DMesh` → Open3D mesh（顶点/三角形/法线/颜色；若消息无法线则用 KDTree 估计 + 朝向 +z 兜底）。
2. mesh 顶点转点云（带法线、灰白）作为**源** `src`。
3. **首帧**：`T = Identity`，锚定世界系。
4. **后续帧**：窗口 map = 窗口内已配准帧合并 + 体素下采样 3.0 m；
   用 `TrimmedPointToPlaneICP` 配准：
   - 多尺度 `8r → 4r → 2r`（r 为两集最近邻中位数均值，采样 2000 点估算）
   - 每级最多 40 次迭代；每轮 KDTree 最近邻 → **距离阈值**过滤 → **法向夹角 < 60°** 过滤
   - 残差从小到大排序，**截断保留前 70%（alpha=0.7）**，最少 10 对
   - 保留集用 Open3D `TransformationEstimationPointToPlane` 解增量变换，`T = ΔT · T`
   - 收敛阈值：平移 < 1e-4 m 且 旋转 < 1e-3°
5. 窗口更新：新帧入、最旧帧出（窗口 4 帧）。
6. 输出：对齐 mesh（`aligned->Transform(T_w)`，法线做刚性旋转）、map 点云、统计、可选 PLY。

## 4. 参数（config/flow_alignment_params.yaml）

| 参数 | 默认 | 含义 |
|---|---|---|
| sub_mesh_topic | /reconstruction/white_mesh | 输入 mesh 话题 |
| pub_aligned_mesh_topic | /flow/aligned_mesh | 对齐后 mesh 话题 |
| pub_map_pc_topic | /flow/aligned_pointcloud | map 点云话题（空=关） |
| pub_stats_topic | /flow/registration_stats | 统计话题 |
| window_size | 4 | 滑动窗口帧数 |
| icp_voxel_downsample | 3.0 | 窗口合并体素(m) |
| alpha | 0.7 | Trimmed 截断比例 |
| icp_max_iter_per_level | 40 | 每级迭代上限 |
| icp_normal_angle_deg | 60.0 | 法向夹角阈值(°) |
| use_gps_init | false | GPS 初值（mesh 消息无 GPS，暂不可用） |
| normal_radius / normal_max_nn | 20.0 / 15 | 兜底法线估计 |
| data_saved_dir | "" | 保存目录（空=不保存） |
| enable_aligned_saving | false | 保存 aligned_*.ply（CloudCompare 看） |
| enable_map_saving | false | 保存 map_*.ply |

## 5. 构建与运行（远程）

```bash
# 在 trf 工作区根目录（colcon 工作区）
cd /home/trf/Documents/codes/pointCloudProcessor
colcon build --packages-select flow_alignment
source install/setup.bash

# 方式 A：launch 启动（推荐）
ros2 launch flow_alignment flow_alignment.launch.py

# 方式 B：ros2 run
ros2 run flow_alignment flow_alignment_node --ros-args \
  --params-file src/flow_alignment/config/flow_alignment_params.yaml

# 查看输出
ros2 topic echo /flow/aligned_mesh --field header
ros2 topic echo /flow/registration_stats

# 保存对齐白膜供 CloudCompare：在 yaml 里设
#   data_saved_dir: /home/trf/Documents/codes/pointCloudProcessor/flow_alignment_out
#   enable_aligned_saving: true
```

## 6. 部署清单（从本目录推送）

```text
flow_alignment/
├── package.xml
├── CMakeLists.txt
├── src/flow_alignment_node.cpp      # 节点本体（算法移植自 demo_flow.cpp）
├── config/flow_alignment_params.yaml
├── launch/flow_alignment.launch.py
└── README.md
```

推送：`scp -r flow_alignment aerial:/home/trf/Documents/codes/pointCloudProcessor/src/`

> 前置：远程 `src/` 下已有 `pc_msgs` 包且 Open3D 以非 ament 方式安装（提供 Open3DConfig.cmake）。
> 若远程 Open3D 头文件不在默认搜索路径，需在 CMakeLists 中补 include 路径（见本地 CMakeLists 注释）。
