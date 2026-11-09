# pc_msgs 接口定义说明文档

`pc_msgs` 是点云处理流水线（`pointCloudProcessor`）的核心基础接口功能包。它集中定义了整套点云与三维重建处理管线在各节点之间流转所依赖的自定义 ROS 2 消息（Message）与服务（Service）规范，贯穿从原始雷达/载机传感器输入、地面滤波、聚类分割、白模重建、流式帧间配准直到末端三维渲染交互的全链路通信。

---

## 1. 功能包概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `pc_msgs` |
| **包类型** | ROS 2 接口定义包 (`rosidl_default_generators`) |
| **核心依赖** | `std_msgs` |
| **工程依赖范围** | 流水线全部 C++ 节点 (`io_handlers`, `ground_segmentation`, `clustering`, `reconstruction`, `flow_alignment`, `visualization`, `bringup`, `connection_test`) |

### 核心设计目标：
1. **Open3D 原生兼容与零开销转换**：将点云与网格数据设计为一维紧凑扁平数组（如 `float32[] points`, `int32[] triangles`），完美匹配 [Open3D](http://www.open3d.org/) 的底层 `std::vector<Eigen::Vector3d>` 与显存顶点缓冲布局，规避传统 ROS 2 `sensor_msgs/msg/PointCloud2` 繁琐复杂的字段反序列化与内存拷贝开销。
2. **高精度地理与飞行位姿支持**：在载机与高压输电线路空间定位中使用 `float64`（双精度浮点），满足 WGS84 经纬度至少 15 位有效数字米级及厘米级精度要求。
3. **松耦合模块交互契约**：标准化各处理阶段的输入输出数据格式，确保各算法节点（如 RANSAC/GMZ 地面分割、不同网格重建策略、流式配准）可即插即用任意替换。

---

## 2. 消息定义规范 (Messages)

所有消息接口均位于 [`src/pc_msgs/msg/`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/) 目录。

### 2.1 [O3DPointCloud.msg](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) —— 核心点云数据包

用于传输原始雷达点云、地面滤波后分离出的地面点云与非地面点云。

```protobuf
std_msgs/Header header
float32[] points
float32[] colors
```

* **字段详解**：
  * `header`：标准 ROS 2 消息头，携带采样时间戳（`header.stamp`）与参考坐标系（`header.frame_id`，默认 `"map"`）。
  * `points`：一维浮点数组，按 `[x0, y0, z0, x1, y1, z1, ...]` 格式展平存储 $N$ 个三维坐标，数组长度为 $3N$。
  * `colors`：一维浮点数组，可选字段；按 `[r0, g0, b0, r1, g1, b1, ...]` 展平存储各点 RGB 归一化颜色（范围 $[0.0, 1.0]$），长度为 $3N$。若数据源无色彩信息则可为空。

### 2.2 [ClusteredPointCloud.msg](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/ClusteredPointCloud.msg) —— 聚类带标签点云包

由 `clustering` 模块处理后输出，为每个非地面点绑定聚类簇编号。

```protobuf
std_msgs/Header header
float32[] points
int32[] labels
int32 max_label
```

* **字段详解**：
  * `header`：继承自上游地面滤波节点的原始时间戳与坐标系。
  * `points`：展平的点云三维坐标数组，长度为 $3N$。
  * `labels`：长度为 $N$ 的一维整型数组。`labels[i]` 对应第 $i$ 个点所属的类别编号；其中数值 $\ge 0$ 表示有效簇类别（$0, 1, 2, \dots$），数值为 `-1` 表示离群噪声点或被尺寸阈值过滤的背景杂点。
  * `max_label`：当前帧中最大的有效聚类标签值。若未检测到有效聚类簇或全部为噪声，则为 `-1`；若为非负值，则有效聚类总数为 `max_label + 1`。

### 2.3 [O3DMesh.msg](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) —— 三角网格与白模数据包

用于传递白模重建结果、帧间流式配准对齐后网格以及末端渲染显示。

```protobuf
std_msgs/Header header
float32[] vertices
int32[] triangles
float32[] vertex_normals
float32[] vertex_colors
```

* **字段详解**：
  * `header`：继承自数据源的原始时间戳。
  * `vertices`：顶点坐标数组，按 `[x0, y0, z0, ...]` 展开，长度为 $3V$（$V$ 为顶点总数）。
  * `triangles`：三角面索引数组，按 `[v0_0, v0_1, v0_2, v1_0, v1_1, v1_2, ...]` 展开，长度为 $3F$（$F$ 为三角形面数）。每个元素指向 `vertices` 的顶点索引（0-based）。
  * `vertex_normals`：顶点法向量数组，按 `[nx0, ny0, nz0, ...]` 展开，长度为 $3V$。用于光照渲染与点到面 ICP 配准；若缺失可在算法内部兜底估计。
  * `vertex_colors`：顶点颜色数组，按 `[r0, g0, b0, ...]` 展开，范围 $[0.0, 1.0]$，长度为 $3V$。

### 2.4 [PlaneState.msg](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/PlaneState.msg) —— 载机高精度位姿状态包

提供巡检飞机/无人机实时 GPS 导航坐标与航向。

```protobuf
std_msgs/Header header

float64 latitude   # 纬度 (度, WGS84)
float64 longitude  # 经度 (度, WGS84)
float64 altitude   # 海拔高度 (米)
float64 course     # 航向角 (度, 以正北为0，顺时针为正)
```

* **字段详解**：
  * 使用 `float64`（C++ `double`）确保精度无损。
  * `latitude` / `longitude`：WGS84 椭球体地理坐标系度数。
  * `altitude`：椭球高度/海拔高度（米）。
  * `course`：机头真航向方位角（$[0^\circ, 360^\circ)$）。

### 2.5 [PowerTowerGPS.msg](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/PowerTowerGPS.msg) —— 高压铁塔全量/批量 GPS 消息

```protobuf
std_msgs/Header header

float64[] latitudes   # 纬度数组 (度, WGS84)，索引 i 对应第 i 座高压塔的纬度
float64[] longitudes  # 经度数组 (度, WGS84)，索引 i 对应第 i 座高压塔的经度
float64[] altitudes   # 高度数组 (米)，索引 i 对应第 i 座高压塔的高度
```

* **字段详解**：采用三个等长数组并行存储，相同索引 $i$ 对应同一座输电铁塔的三维地理空间位置。

---

## 3. 服务定义规范 (Services)

位于 [`src/pc_msgs/srv/`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/srv/) 目录。

### 3.1 [QueryNearbyTowers.srv](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/srv/QueryNearbyTowers.srv) —— 周边高压塔范围检索服务

用于下游白模重建或场景融合节点主动根据当前载机位置拉取周围高压铁塔，动态插入真实 CAD 几何模型。

```protobuf
# 请求 (Request)
float64 latitude    # 载机当前纬度 (度, WGS84)
float64 longitude   # 载机当前经度 (度, WGS84)
float64 radius_km   # 查询半径 (千米)，若 <= 0 则使用服务端预设默认值
---
# 响应 (Response)
float64[] latitudes   # 命中的高压塔纬度列表
float64[] longitudes  # 命中的高压塔经度列表
float64[] altitudes   # 命中的高压塔高度列表
bool success          # 查询执行状态 (true 为成功)
string message        # 附加状态或错误描述
```

---

## 4. 全流水线消息流转关系

```
[传感器数据源 / io_handlers]
   │
   ├─── O3DPointCloud (/io/raw_pointcloud)
   │         │
   │         ▼
   │   [ground_segmentation]
   │         ├─── O3DPointCloud (/gs/ground_pointcloud) ────────┐
   │         │                                                  │
   │         └─── O3DPointCloud (/gs/non_ground_pointcloud)     │
   │                   │                                        │
   │                   ▼                                        │
   │             [clustering]                                   │
   │                   │                                        │
   │                   ▼                                        │
   │             ClusteredPointCloud (/clustering/clustered...) │
   │                   │                                        │
   │                   └───────────────────────┬────────────────┘
   │                                           │ (ApproximateTime 同步)
   │                                           ▼
   │                                   [reconstruction]
   │                                           │
   │                                           ▼
   │                                   O3DMesh (/reconstruction/white_mesh)
   │                                           │
   │                                           ▼
   │                                    [flow_alignment]
   │                                           │
   │                                           ▼
   │                                   O3DMesh (/flow/aligned_mesh)
   │                                           │
   │                                           ▼
   │                                    [visualization]
   │
   ├─── PlaneState (/io/plane/state) ─────────► [reconstruction_g_poisson_ng_obb_pt]
   │                                                  │
   └─── QueryNearbyTowers.srv (/io/query_...) ◄───────┘
```

---

## 5. C++ 接口代码调用范例

### 5.1 引入头文件与依赖
在 `CMakeLists.txt` 中添加：
```cmake
find_package(pc_msgs REQUIRED)
ament_target_dependencies(your_node "rclcpp" "pc_msgs")
```

在 C++ 源码中引用：
```cpp
#include "pc_msgs/msg/o3_d_point_cloud.hpp"
#include "pc_msgs/msg/clustered_point_cloud.hpp"
#include "pc_msgs/msg/o3_d_mesh.hpp"
#include "pc_msgs/msg/plane_state.hpp"
#include "pc_msgs/srv/query_nearby_towers.hpp"
```

### 5.2 转换为 Open3D 几何对象示例
```cpp
// 1. 将 pc_msgs::msg::O3DPointCloud 快速解析为 open3d::geometry::PointCloud
auto pcd = std::make_shared<open3d::geometry::PointCloud>();
size_t num_points = msg->points.size() / 3;
pcd->points_.reserve(num_points);
for (size_t i = 0; i < num_points; ++i) {
    pcd->points_.emplace_back(
        static_cast<double>(msg->points[i * 3 + 0]),
        static_cast<double>(msg->points[i * 3 + 1]),
        static_cast<double>(msg->points[i * 3 + 2])
    );
}
```

---

## 6. 文件结构与索引

```
src/pc_msgs/
├── CMakeLists.txt                # ROS 2 接口生成构建脚本
├── package.xml                   # 包声明与依赖定义
├── msg/                          # 自定义消息定义
│   ├── ClusteredPointCloud.msg   # 聚类点云消息 (坐标 + 标签)
│   ├── O3DMesh.msg               # Open3D 风格三角网格消息 (顶点/三角形/法线/颜色)
│   ├── O3DPointCloud.msg         # 扁平点云消息 (坐标 + 颜色)
│   ├── PlaneState.msg            # 载机高精度位姿与航向消息
│   └── PowerTowerGPS.msg         # 高压铁塔 GPS 坐标列表
└── srv/                          # 自定义服务定义
    └── QueryNearbyTowers.srv     # 邻近高压铁塔检索服务
```
