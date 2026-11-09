# io_handlers 节点说明文档

`io_handlers` 是点云处理流水线的输入适配与仿真数据源功能包。它主要负责从本地数据集目录中读取按时间序列排列的雷达点云（`.ply` 文件）、载机空间运动轨迹（`extend_data.json`）以及输电高压铁塔地理信息（`powerLineGPS.json`），提供内存全量预加载、定时恒频发布、载机状态与时间戳对齐，以及基于地理大圆距离（Haversine）的高压铁塔空间检索服务，为整条处理流水线提供真实高效的数据注入。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `io_handlers` |
| **可执行文件** | 1. `sim_lidar_data_flow_node`<br>2. `sim_plane_data_flow_node` |
| **主 C++ 类** | 1. `IOHandlerNode` ([sim_lidar_data_flow_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/io_handlers/src/sim_lidar_data_flow_node.cpp))<br>2. `SimPlaneDataFlowNode` ([sim_plane_data_flow_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/io_handlers/src/sim_plane_data_flow_node.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `nlohmann_json` (header-only) |

### 核心设计目标：
1. **内存零磁盘 I/O 实时回放**：节点在初始化阶段一次性将目录下的所有 `.ply` 文件解析展平并驻留在内存中（Preload），运行时彻底消除磁盘读写瓶颈，确保按严格时钟定时触发发布。
2. **多模态传感器时钟精确对齐**：在 `sim_plane_data_flow_node` 中，每帧点云与其对应的经纬高及航向角（`PlaneState`）被打上完全一致的纳秒级系统时间戳（`header.stamp = now_stamp`），为下游 `message_filters` 的多路时间同步提供精准基准。
3. **基于 Haversine 大圆算法的空间铁塔检索**：提供服务服务端 `QueryNearbyTowers`，支持根据无人机当前经纬度，动态计算地表球面距离，毫秒级筛选出指定半径内的所有高压塔。
4. **标准化阶段完成日志输出**：严格遵守全局日志规范，每帧发布后输出带有 `[STAGE-DONE]` 标记的日志，方便总控节点 `bringup` 进行吞吐率与丢帧统计。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
                       [本地数据集目录 (dataset_dir)]
                       ├── *.ply 序列点云文件
                       ├── extend_data.json (载机 GPS / Course)
                       └── powerLineGPS.json (全量铁塔坐标)
                                     │
                     ┌───────────────┴───────────────┐
                     ▼                               ▼
       ┌───────────────────────────┐   ┌───────────────────────────┐
       │ sim_lidar_data_flow_node  │   │ sim_plane_data_flow_node  │
       │ (纯点云模拟回放)          │   │ (多模态全要素仿真)        │
       └─────────────┬─────────────┘   └─────────────┬─────────────┘
                     │                               │
        /io/raw_pointcloud                           ├── /io/raw_pointcloud (O3DPointCloud)
       (O3DPointCloud)                               ├── /io/plane/state (PlaneState)
                     │                               └── /io/query_nearby_towers (Service)
                     ▼                                               ▲
          [下游流水线处理各节点]                                      │
                                                    [reconstruction 等下游节点]
```

### 2.2 内存预加载机制 (Preload Mechanism)
为了防止运行时频繁读取几十兆字节的大尺寸 PLY 点云导致帧率波动与定时器抖动，节点在启动时按文件名升序排列遍历目录：
* 提前解析点坐标与色彩，转为连续的一维紧凑向量 `std::vector<float> points` 与 `colors`；
* 预先调用 `vector::reserve` 申请内存空间；
* 预加载完成后，定时器回调仅需直接引用内存数据进行消息发布，计算耗时几乎为 0.00 ms。

### 2.3 Haversine 地理大圆距离检索
针对球面经纬度定位，节点采用地球平均半径 $R = 6371.0\text{ km}$ 的 Haversine 公式计算两点水平直线距离：
$$\Delta\sigma = 2 \arcsin\left(\sqrt{\sin^2\left(\frac{\Delta\phi}{2}\right) + \cos(\phi_1)\cos(\phi_2)\sin^2\left(\frac{\Delta\lambda}{2}\right)}\right)$$
$$d = R \cdot \Delta\sigma$$
当客户端调用 `QueryNearbyTowers.srv` 时，服务端快速遍历内存中所有高压铁塔，筛选出 $d \le \text{radius\_km}$ 的目标塔并返回。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在成功发送一帧数据后，立即打印标准化完成日志，供 `bringup` 节点监听吞吐率：

```text
[STAGE-DONE] io_handlers: Frame %zu published | File: '%s' | Points: %zu | Time: 0.00 ms
```

* **示例输出**：
  ```text
  [sim_lidar_data_flow_node-1] [STAGE-DONE] io_handlers: Frame 0 published | File: '000000.ply' | Points: 284160 | Time: 0.00 ms
  ```

---

## 4. 通信接口

### 4.1 发布话题 (Published Topics)

| 节点 | 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- | :--- |
| `sim_lidar_data_flow_node` | `/io/raw_pointcloud` (可配) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 原始雷达点云序列 |
| `sim_plane_data_flow_node` | `/io/raw_pointcloud` (可配) | [pc_msgs/msg/O3DPointCloud](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DPointCloud.msg) | 原始雷达点云序列 |
| `sim_plane_data_flow_node` | `/io/plane/state` (可配) | [pc_msgs/msg/PlaneState](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/PlaneState.msg) | 载机经纬度、海拔与航向状态 |

### 4.2 提供服务 (Services)

| 节点 | 服务名称 | 服务类型 | 说明 |
| :--- | :--- | :--- | :--- |
| `sim_plane_data_flow_node` | `/io/query_nearby_towers` (可配) | [pc_msgs/srv/QueryNearbyTowers](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/srv/QueryNearbyTowers.srv) | 接收载机经纬度，返回范围内的所有高压铁塔列表 |

---

## 5. 参数配置说明

配置文件位于 [`src/io_handlers/config/io_config.yaml`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/io_handlers/config/io_config.yaml)。

### 5.1 `sim_lidar_data_flow_node` 参数

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `dataset_dir` | `string` | `""` | 包含 `.ply` 文件的输入数据集目录绝对路径 |
| `publish_rate` | `double` | `0.5` | 点云发布频率（单位：Hz） |
| `publish_topic` | `string` | `"/io/raw_pointcloud"` | 发布点云的话题名称 |
| `frame_id` | `string` | `"map"` | ROS 消息坐标系名称 |
| `is_loop` | `bool` | `false` | 数据播放完毕后是否循环重播（`false` 时发完即停止定时器） |

### 5.2 `sim_plane_data_flow_node` 参数

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `dataset_dir` | `string` | `""` | 数据集根目录（须包含 `extend_data.json` 与 `powerLineGPS.json`） |
| `publish_rate` | `double` | `0.5` | 点云与载机状态同步发布频率（Hz） |
| `pointcloud_topic` | `string` | `"/io/raw_pointcloud"` | 点云发布话题 |
| `plane_state_topic` | `string` | `"/io/plane/state"` | 载机状态（`PlaneState`）发布话题 |
| `query_service_name` | `string` | `"/io/query_nearby_towers"` | 高压塔空间检索 Service 服务名称 |
| `frame_id` | `string` | `"map"` | 消息坐标系名称 |
| `is_loop` | `bool` | `false` | 是否循环播放 |
| `default_query_radius_km`| `double` | `5.0` | 当 Service 请求中未指定或 `radius_km <= 0` 时的默认检索半径（公里） |

---

## 6. 常用配置与启动脚本

功能包提供了两个开箱即用的 Launch 文件：

### 6.1 纯点云数据回放：[io_simLidarDataFlow.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/io_handlers/launch/io_simLidarDataFlow.launch.py)
用于不需要载机轨迹或铁塔插入的常规单机测试。

### 6.2 载机全模态数据回放：[io_simPlaneDataFlow.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/io_handlers/launch/io_simPlaneDataFlow.launch.py)
用于配合带高压塔白模注入的重建节点（`reconstruction_g_poisson_ng_obb_pt_node`），同时提供点云、GPS 轨迹和铁塔查询服务。

---

## 7. 启动与使用命令参考

```bash
# 1. 启动纯雷达点云模拟节点
ros2 launch io_handlers io_simLidarDataFlow.launch.py

# 2. 启动载机多模态综合数据流节点（推荐）
ros2 launch io_handlers io_simPlaneDataFlow.launch.py

# 3. 通过命令行查看发布的消息
ros2 topic echo /io/raw_pointcloud --field header
ros2 topic echo /io/plane/state

# 4. 手动调用高压铁塔检索服务测试
ros2 service call /io/query_nearby_towers pc_msgs/srv/QueryNearbyTowers "{latitude: 31.2304, longitude: 121.4737, radius_km: 10.0}"
```

---

## 8. 文件结构与索引

```
src/io_handlers/
├── CMakeLists.txt                    # 构建工程描述文件
├── package.xml                       # 包依赖与元信息
├── config/
│   └── io_config.yaml                # 节点核心配置文件
├── include/
│   └── nlohmann/
│       └── json.hpp                  # header-only JSON 解析库
├── launch/
│   ├── io_simLidarDataFlow.launch.py # 纯雷达点云播放启动脚本
│   └── io_simPlaneDataFlow.launch.py # 综合载机多模态数据播放启动脚本
└── src/
    ├── sim_lidar_data_flow_node.cpp  # 纯点云序列回放节点实现
    └── sim_plane_data_flow_node.cpp  # 载机多模态点云/GPS/铁塔查询服务实现
```
