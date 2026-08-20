# io_handlers 节点说明文档

`io_handlers` 是点云处理流水线的数据输入输出（IO）控制功能包。该功能包负责从本地存储中载入激光雷达（LiDAR）点云数据集，并将其序列化转换为 ROS 2 自定义消息格式，按指定的频率周期性模拟发布实时激光雷达点云数据流。

---

## 1. 节点与可执行文件概览

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `io_handlers` |
| **可执行文件** | `sim_lidar_data_flow_node` |
| **主 C++ 类** | `IOHandlerNode` ([sim_lidar_data_flow_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/io_handlers/src/sim_lidar_data_flow_node.cpp#L28)) |
| **依赖库** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `std::filesystem` |

---

## 2. 核心算法逻辑与工作流程

`sim_lidar_data_flow_node` 节点的生命周期与数据处理流程如下：

```
[本地数据集路径 (dataset_dir)] 
            │
            ▼ (std::filesystem 扫描 .ply 文件并按名称排序)
[PLY 文件列表 (ply_files_)]
            │
            ▼ (open3d::io::CreatePointCloudFromFile 预加载)
[内存帧数据缓存 (frames_data_)]
            │  └─ 展平 Double(X,Y,Z) -> Float 数组 (points/colors)
            ▼
[ROS 2 墙上定时器 (timer_callback)]
            │  └─ 依据 publish_rate 频率触发
            ▼
[发布 pc_msgs::msg::O3DPointCloud 至 /io/raw_pointcloud]
```

### 关键步骤细节：
1. **数据集检索与排序**：节点启动时，通过 `std::filesystem::directory_iterator` 遍历 `dataset_dir` 目录，检索所有扩展名为 `.ply` 的点云文件，并按照文件字典序进行 `std::sort` 排序，确保点云按照时序正确播放。
2. **高效预加载机制 (`preload_pointclouds`)**：
   - 采用一次性预加载策略（Preloading），避免在定时器回调中频繁进行磁盘 I/O 导致帧率抖动。
   - 使用 Open3D 的 `open3d::io::CreatePointCloudFromFile` 读取点云。
   - 提取 3D 点坐标 $(x, y, z)$ 和颜色数据 $(r, g, b)$，将其从 Open3D 的双精度 `double` 转换为单精度 `float` 并展平放入一维数组 `std::vector<float>` 中。
   - 内部定义结构体 `FrameData` 存储预展平的数据：
     ```cpp
     struct FrameData {
         std::string file_name;
         std::vector<float> points;
         std::vector<float> colors;
         size_t num_points;
     };
     ```
3. **周期性定时发布与循环逻辑**：
   - 根据参数 `publish_rate` 创建 `rclcpp::TimerBase` 定时器。
   - 定时器回调 `timer_callback()` 每次打包当前帧的 `O3DPointCloud` 消息并发布。
   - 消息的 `header.stamp` 设置为当前 ROS 时钟时间，`header.frame_id` 设为配置的 TF 坐标系。
   - 当播放至最后一帧时，若 `is_loop == true` 则重置索引 `current_idx_ = 0` 继续循环播放；若为 `false` 则取消定时器停止发布。

---

## 3. ROS 2 通信接口

### 3.1 订阅话题 (Subscribed Topics)
无（本节点为数据源节点）。

### 3.2 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/io/raw_pointcloud` (默认) | [pc_msgs/msg/O3DPointCloud](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DPointCloud.msg) | 原始雷达点云数据流，包含坐标与可选颜色信息 |

### 3.3 关联消息数据结构 (`pc_msgs::msg::O3DPointCloud`)
```protobuf
std_msgs/Header header   # 包含 stamp 时间戳与 frame_id 坐标系
float32[] points        # 展平的点坐标数组 [x0, y0, z0, x1, y1, z1, ...]
float32[] colors        # 展平的 RGB 颜色数组 [r0, g0, b0, r1, g1, b1, ...] (可选)
```

---

## 4. 参数配置说明

配置文件位于 [io_handlers/config/io_config.yaml](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/io_handlers/config/io_config.yaml)。

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `dataset_dir` | `string` | `""` | 包含 `.ply` 点云文件的本地目录路径 |
| `publish_rate` | `double` | `2.0` | 点云数据流发布频率（单位：Hz） |
| `publish_topic` | `string` | `"/io/raw_pointcloud"` | 发布的 ROS 2 点云话题名称 |
| `frame_id` | `string` | `"map"` | ROS TF 坐标系 ID |
| `is_loop` | `bool` | `true` | 数据集中所有文件发布完毕后是否循环播放 |

---

## 5. 启动与使用方式

### 5.1 Launch 启动
可以使用 ROS 2 Launch 脚本一键加载配置参数并启动节点：
```bash
ros2 launch io_handlers io_simLidarDataFlow.launch.py
```

### 5.2 Launch 脚本解析
[io_simLidarDataFlow.launch.py](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/io_handlers/launch/io_simLidarDataFlow.launch.py) 会自动定位 `io_handlers` 的 `share` 目录，读取并注入 `io_config.yaml` 配置文件中的全部参数。

---

## 6. 文件结构与索引

```
src/io_handlers/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── io_config.yaml               # 节点配置文件
├── launch/
│   └── io_simLidarDataFlow.launch.py # 节点 Launch 启动文件
└── src/
    └── sim_lidar_data_flow_node.cpp  # 节点核心源码实现
```
