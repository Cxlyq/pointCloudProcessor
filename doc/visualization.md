# visualization 节点说明文档

`visualization` 是点云处理流水线末端的高性能 3D 渲染与动态可视化功能包。该功能包订阅上游发来的三维白模网格消息（`/reconstruction/white_mesh`），利用 Open3D 渲染引擎进行多视角 3D 图形渲染。

为了解决低帧率点云重建（如 2Hz）在实时渲染时出现的画面卡顿顿挫感，包内集成了基于 **OpenCV DIS（Dense Inverse Search）稠密光流算法** 的视频帧率内插（Frame Interpolation）引擎，并设计了避开 OpenCV HighGUI 阻塞的原生窗口组件 **`NativeFrameWindow`**，能够在后台多线程中实现高帧率（如 60+ FPS）的超平滑动态白模渲染展示。

---

## 1. 节点与可执行文件概览

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `visualization` |
| **可执行文件** | `visualization_node` |
| **主 C++ 类** | `VisualizationNode` ([visualization_node.cpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/visualization/src/visualization_node.cpp#L215)) |
| **辅助核心模块** | `DisFrameInterpolator` ([dis_frame_interpolator.hpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/visualization/include/visualization/dis_frame_interpolator.hpp#L51)), `NativeFrameWindow` ([native_frame_window.hpp](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/visualization/include/visualization/native_frame_window.hpp#L13)) |
| **依赖库** | ROS 2 (`rclcpp`), `pc_msgs`, `Open3D`, `OpenCV` (core/highgui/imgproc/video), `X11` (Linux) / `GDI` (Windows), `Eigen3` |

---

## 2. 核心架构与技术特色

```
 [三维白模 ROS 消息 /reconstruction/white_mesh]
                          │
                          ▼
 [Open3D 三角网格反序列化 & 多视角 Visualizer 渲染]
 (cam_front, cam_top 视角 / 离屏抓图抓取 RGB 帧)
                          │
         ┌────────────────┴────────────────┐
         ▼ (若开启插帧 interpolation)       ▼ (若未开启插帧)
 [DIS 光流多线程插帧 Worker 队列]           [直接输出/原生窗口展示]
 (计算稠密运动矢量场 / 生成 N 帧中间过渡帧)           │
         │                                 │
         ▼                                 │
 [NativeFrameWindow 原生 X11/GDI 低延迟呈现] ◄┘
```

### 2.1 Open3D 多视角渲染引擎 (Multi-Camera Offscreen Rendering)
- 支持同时开启多个虚拟相机视角视窗（如正面视角 `cam_front`、上帝俯视视角 `cam_top` 等）。
- 每个视角可独立配置相机的视点位置 (`front`, `lookat`, `up`, `zoom`)、窗口分辨率 (`width`, `height`) 和背景颜色 (`background_color`)。
- 当新的三维 Mesh 到达时，自动更新 Open3D 的几何体数据并重新刷新画面。

---

### 2.2 DIS 稠密光流插帧引擎 (`DisFrameInterpolator`)
在点云实时处理中，上游地面分割与泊松重建通常耗时较长（例如发布频率仅 2~5 Hz），直接播放会导致视窗画面极度卡顿。本节点集成了基于 OpenCV DIS 光流的画面插帧技术：

1. **光流计算 (DIS Optical Flow)**：
   - 提取相邻两个真实渲染帧 $I_t$ 与 $I_{t+1}$。
   - 使用 OpenCV 的 `cv::DISOpticalFlow` 计算像素级稠密光流场（DIS 算法速度极快且细节保持优异）。
   - 支持 `ultrafast`（极速）、`fast`（快速）和 `medium`（中等）三种运算预设。
   - 支持**双向光流推算** (`use_bidirectional_flow`)，有效解决物体遮挡与边缘变形问题。
2. **多线程异步生成**：
   - 抓图与光流计算在专门的 Worker 线程中异步进行，完全解耦 ROS 2 消息接收线程与 UI 呈现线程。
3. **中间帧平滑播放**：
   - 根据配置的 `intermediate_frame_count`（例如 4 帧或 19 帧），以均匀的时间间隔内插中间画面，使 2Hz 的低帧率输入在显示器上以 **60 FPS** 甚至更高的平滑帧率播放。

---

### 2.3 Native Window 原生同线程渲染窗口 (`NativeFrameWindow`)
- **痛点**：传统 OpenCV `cv::imshow` / HighGUI 界面在 Linux X11 环境变量复杂或多线程环境下极易引发界面冻结、无响应或线程锁死。
- **自研原生窗口**：直接调用 Linux 原生 **X11 绘图 API**（Windows 平台调用 GDI API）封装为 `NativeFrameWindow` 窗口。
- 能够在同一个 UI 线程中进行极低延迟的图像 Buffer 呈现与事件 Poll，彻底消除 HighGUI 阻塞导致的卡顿。

---

### 2.4 全方位 Diagnostics 性能诊断系统
节点内部内置了高强度的诊断日志统计机制，按设定的时间间隔（如每 5 秒）在终端输出细粒度的运行统计数据：
- **接收帧率 (Rx FPS)** 与平均顶点数/三角形数。
- **渲染阶段耗时分解**：`poll_events`, `geometry_update`, `capture`, `conversion`, `submit` 耗时毫秒数。
- **插帧队列状态 (DIS Queue Stats)**：挂起对、已就绪序列数、Worker 线程忙碌状态及调度延迟。

---

## 3. ROS 2 通信接口

### 3.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/reconstruction/white_mesh` | [pc_msgs/msg/O3DMesh](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/pc_msgs/msg/O3DMesh.msg) | 由重建节点发布的三维白模网格消息 |

### 3.2 发布话题 (Published Topics)
无（本节点为终端可视化渲染节点）。

---

## 4. 参数配置说明

配置文件位于 [visualization/config/](file:///home/cx/Documents/codes/pointcloud_607_roscpp/src/visualization/config/) 目录下，提供了多种预设配置：
- `v_config.yaml`：标准渲染配置（关闭插帧）。
- `v_dis_ultrafast_config.yaml`：极速 DIS 光流插帧配置（插入 4 帧）。
- `v_dis_quality_x20_config.yaml`：高清 20 倍插帧配置（插入 19 帧，高流畅度）。
- `v_dis_bidirectional_config.yaml`：双向 DIS 光流插帧配置。

### 核心参数详解

| 参数分类 | 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- | :--- |
| **基础配置** | `subscribe_topic` | `string` | `"/reconstruction/white_mesh"` | 订阅的白模 Mesh 话题 |
| | `fps_logging_enabled` | `bool` | `true` | 是否在终端输出 FPS 与耗时诊断日志 |
| | `fps_logging_interval_sec` | `double` | `5.0` | 诊断日志输出时间间隔（单位：秒） |
| | `camera_ids` | `array[string]` | `["cam_front"]` | 需要启用的视窗相机 ID 列表 |
| **插帧配置** | `interpolation_enabled` | `bool` | `false` | 是否开启 DIS 光流插帧功能 |
| | `interpolation_intermediate_frames` | `int` | `4` | 相邻两帧之间插入的中间帧数量 |
| | `interpolation_flow_scale` | `double` | `0.25` | 计算光流时的分辨率下采样缩放比例 |
| | `interpolation_dis_preset` | `string` | `"ultrafast"` | DIS 光流预设模式 (`"ultrafast"`, `"fast"`, `"medium"`) |
| | `interpolation_bidirectional_flow` | `bool` | `false` | 是否开启双向光流推算 |
| | `interpolation_display_backend` | `string` | `"auto"` | 视窗呈现后端 (`"auto"`, `"native"`, `"opencv"`) |
| **相机视角** | `cam_front.name` | `string` | `"Front View"` | 渲染窗口标题名称 |
| | `cam_front.width` | `int` | `800` | 渲染窗口宽度像素数 |
| | `cam_front.height` | `int` | `600` | 渲染窗口高度像素数 |
| | `cam_front.background_color` | `array[double]` | `[0.1, 0.1, 0.1]` | 背景 RGB 颜色 (0.0~1.0) |
| | `cam_front.front` | `array[double]` | `[-0.0, -1.0, 0.35]` | 相机朝向向量 |
| | `cam_front.lookat` | `array[double]` | `[-17.5, 2213.0, -86.0]` | 相机观察焦点坐标 |
| | `cam_front.up` | `array[double]` | `[-0.0, 0.35, 1.0]` | 相机上方头顶向量 |
| | `cam_front.zoom` | `double` | `0.7` | 相机缩放倍率 |

---

## 5. 启动与使用方式

可以根据需要选择不同模式启动可视化节点：

```bash
# 1. 启动标准 3D 渲染可视化节点（无插帧）
ros2 launch visualization v_vis.launch.py

# 2. 启动极速 DIS 光流插帧模式 (Ultrafast Interpolation)
ros2 launch visualization v_dis_ultrafast.launch.py

# 3. 启动高清 x20 光流插帧模式 (High-Quality x20 Smooth Interpolation)
ros2 launch visualization v_dis_quality_x20.launch.py

# 4. 启动双向 DIS 光流插帧模式 (Bidirectional Optical Flow)
ros2 launch visualization v_dis_bidirectional.launch.py
```

---

## 6. 文件结构与索引

```
src/visualization/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── v_config.yaml                         # 标准可视化配置
│   ├── v_dis_ultrafast_config.yaml           # 极速光流插帧配置
│   ├── v_dis_quality_x20_config.yaml         # 20倍高清插帧配置
│   └── v_dis_bidirectional_config.yaml       # 双向光流插帧配置
├── include/visualization/
│   ├── dis_frame_interpolator.hpp            # DIS 光流插帧引擎头文件
│   └── native_frame_window.hpp               # 原生 X11/GDI 渲染窗口头文件
├── launch/
│   ├── v_vis.launch.py                       # 标准可视化 Launch
│   ├── v_dis_ultrafast.launch.py             # 极速插帧 Launch
│   ├── v_dis_quality_x20.launch.py           # 20倍高清插帧 Launch
│   └── v_dis_bidirectional.launch.py         # 双向插帧 Launch
└── src/
    ├── visualization_node.cpp                # 节点主程序与 Diagnostics 实现
    ├── dis_frame_interpolator.cpp            # DIS 光流插帧计算逻辑
    └── native_frame_window.cpp               # 本地原生绘图窗口实现
```
