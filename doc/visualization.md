# visualization 节点说明文档

`visualization` 是点云处理流水线的末端三维高保真图形渲染与人机交互展示功能包。它接收来自上游的白模三角网格（经配准的 `/flow/aligned_mesh` 或未配准的 `/reconstruction/white_mesh`），支持三种核心渲染模式：**离线 GPS 跟随相机与累计覆盖渐进渲染（`offline_terrain_display`）**、**Open3D 多机位传送带式平滑动画渲染（`animation_enabled`）** 以及 **基于 DIS 稠密光流的实时多倍率视频插帧渲染（`interpolation_enabled`）**，使低频离散白模在操作界面上呈现出丝滑连贯的高清数字孪生视觉效果。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `visualization` |
| **可执行文件** | `visualization_node` |
| **主 C++ 类 / 模块** | `VisualizationNode` ([`visualization_node.cpp`](../src/visualization/src/visualization_node.cpp))<br>`offline_terrain` ([`offline_terrain_display.hpp`](../src/visualization/src/offline_terrain_display.hpp))<br>`DisFrameInterpolator` ([`dis_frame_interpolator.cpp`](../src/visualization/src/dis_frame_interpolator.cpp))<br>`NativeFrameWindow` ([`native_frame_window.cpp`](../src/visualization/src/native_frame_window.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `std_msgs`, `geometry_msgs`), `pc_msgs`, `Open3D`, `OpenCV` (core, highgui, imgproc, video), `X11`, `Threads` |

### 核心设计目标：
1. **离线 GPS 跟随相机、累计覆盖与远景渐变 (`offline_terrain_display`)**：默认开启的高保真显示管线。将对齐后的世界系网格与 `PlaneState` 按采集时间戳精准配对，利用 5 米 2D 网格占用表去重并累计拼接地形白模；在相邻帧导航位姿之间以 60 步（30 FPS）平滑推进虚拟相机，采用固定光照 CPU 软光栅并发渲染历史累计层与前后帧局部更新层，并在视口远景区域执行 Smoothstep 渐变混合，消除新块突兀跳变与远程 SSH 渲染兼容问题。
2. **多机位并行视口与传送带式平滑流动动画 (`animation_enabled`)**：在 Open3D 渲染模式中支持按 YAML 配置并发开启多个独立视口（如 `cam_front`、`cam_top`），将多帧网格组织为环形缓冲区，以 30 FPS 匀速流动装载与视锥动态卸载，并通过 Z 轴微小错位消除重叠面闪烁。
3. **基于 DIS 稠密光流的 2D 实时极速超分辨率插帧 (`interpolation_enabled`)**：集成 OpenCV DIS（Dense Inverse Search）算法，在捕获的渲染帧之间实时生成 4~19 帧运动平滑过渡帧，将低频刷新提升至 30~60 Hz 流畅显示。
4. **原生 X11 / HighGUI 双后端加速显示**：针对 Linux 桌面环境，提供基于 X11 原生共享内存与双缓冲的 `NativeFrameWindow` 显示后端，保证高分辨率图像低延迟刷屏。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
      /flow/aligned_mesh (O3DMesh)              /io/plane/state 或 /flow/registration_stats
                   │                                             │
                   └──────────────────────┬──────────────────────┘
                                          ▼
                      ┌───────────────────────────────────────┐
                      │          visualization_node           │
                      │                                       │
                      │  ┌─────────────────────────────────┐  │
                      │  │ 模式 A: offline_terrain_display │  │
                      │  │ * 纳秒时间戳配对 PlaneState     │  │
                      │  │ * 5m 网格累计覆盖去重 (Coverage) │  │
                      │  │ * 60步/30FPS 导航跟随与远景渐变 │  │
                      │  └─────────────────────────────────┘  │
                      │  ┌─────────────────────────────────┐  │
                      │  │ 模式 B: Open3D 传送带流动动画   │  │
                      │  │ * 环形缓冲维护与视锥动态裁剪    │  │
                      │  │ * Z-Fighting 深度微调           │  │
                      │  └─────────────────────────────────┘  │
                      │  ┌─────────────────────────────────┐  │
                      │  │ 模式 C: DIS 稠密光流极速插帧    │  │
                      │  │ * DIS 多尺度光流场解算          │  │
                      │  │ * 双向加权 Warping (x5~x20)     │  │
                      │  └─────────────────────────────────┘  │
                      └───────────────────┬───────────────────┘
                                          │
                                          ▼
                            [Native X11 视窗 / Open3D 视窗]
```

### 2.2 离线地形渐进渲染模式 (`offline_terrain_display`)
当 `offline_terrain_display: true`（当前 [`v_config.yaml`](../src/visualization/config/v_config.yaml) 默认模式）时：
1. **时间戳精确配对 (`pair_messages`)**：同时订阅 `subscribe_topic`（默认 `/flow/aligned_mesh`）与 `plane_state_topic`（默认 `/io/plane/state`），以纳秒采集时间戳 `stamp_key` 在缓存队列中配对，将载机经纬高和航向角转换为以首帧为原点的局部刚体位姿 `current_nav_`；世界系下的白模无需再次变换，导航仅用于驱动巡航相机。
2. **5 米空间网格累计去重 (`Coverage::append`)**：维护 $5.0\text{ m} \times 5.0\text{ m}$ 的 2D 网格占用集合 `occupied`。对新到达网格的每个三角面提取 7 个采样点（3 个顶点、3 个边中点与重心），仅当均未被历史网格覆盖时才将其保留并光栅化注册进占用表，最终无缝合入全局累计网格 `cumulative`。
3. **三层并发软光栅与远景 Smoothstep 渐变 (`Segment::draw`)**：
   * 在相邻两帧导航位姿之间以 Smoothstep 插值航向偏转角 `yaw` 并线性插值相机位置 `P`，沿运动方向设定前倾俯视相机位姿（`eye = P + 400*g + 680*front`）。
   * 利用 `std::async` 并发执行固定光照 CPU 软光栅器 `raster()`（$800 \times 600$），同时渲染历史累计层（`b.cumulative`）、前一帧更新层（`a.update`）与当前帧更新层（`b.update`）。
   * 在画面顶部 `y < 160` 像素的远景区域内结合垂直边缘羽化权重 `edge` 与时间平滑系数 `fade`，对前后帧更新层进行平滑淡入淡出合成，再以约 30 FPS（每帧间隔 `33.333 ms`，每段 60 步）推流至 `NativeFrameWindow`。

### 2.3 Open3D 传送带式平滑动画渲染原理
当关闭离线模式且启用 `animation_enabled: true` 时：
* 维持容量为 `animation_buffer_size`（默认 20 帧）的滑动网格库，结合 `/flow/registration_stats` 计算前向（`animation_view_ahead`）与后向（`animation_view_behind`）可视范围并动态卸载超出视锥的模型；
* 对相邻帧施加微小 Z 轴偏移（`animation_depth_offset: 0.02m`）消除深度缓冲竞争闪烁。

### 2.4 DIS 稠密光流插帧机制
当启用 `interpolation_enabled: true` 时：
* Open3D 作为后台无缝渲染器输出真实几何渲染帧；
* `DisFrameInterpolator` 使用 `cv::DISOpticalFlow`（支持 `ultrafast` / `fast` / `medium` 预设）在连续渲染帧间解算像素光流场，通过前向或双向 Warping 融合生成虚拟过渡帧，并由 `NativeFrameWindow` 以高倍率输出。

---

## 3. 性能优化与确认信号规范

### 3.1 阶段确认标准日志格式
节点在接收并解析完成每一帧网格后，输出标准化日志：

```text
[STAGE-DONE] visualization: Frame %zu received | Vertices: %zu, Triangles: %zu | Time: %.2f ms
```

* **示例输出**：
  ```text
  [visualization_node-6] [STAGE-DONE] visualization: Frame 1 received | Vertices: 15420, Triangles: 29810 | Time: 8.50 ms
  ```

---

## 4. 通信接口

### 4.1 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `subscribe_topic` (默认 `/flow/aligned_mesh`) | [`pc_msgs/msg/O3DMesh`](../src/pc_msgs/msg/O3DMesh.msg) | 待呈现的三角网格（经配准的 `/flow/aligned_mesh` 或未配准的 `/reconstruction/white_mesh`） |
| `plane_state_topic` (默认 `/io/plane/state`) | [`pc_msgs/msg/PlaneState`](../src/pc_msgs/msg/PlaneState.msg) | `offline_terrain_display` 模式下用于驱动跟随相机的载机 GPS 与航向状态 |
| `animation_stats_topic` (默认 `/flow/registration_stats`) | `std_msgs/msg/Float64MultiArray` | Open3D 传送带动画模式下参考的位姿与统计指标 |

---

## 5. 参数配置说明

配置文件位于 [`src/visualization/config/`](../src/visualization/config/) 目录下。

### 5.1 离线地形展示、基础与相机参数 ([`v_config.yaml`](../src/visualization/config/v_config.yaml))

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `offline_terrain_display` | `bool` | `true` | 是否启用离线 GPS 跟随相机、累计覆盖去重与远景渐变渲染模式 |
| `plane_state_topic` | `string` | `"/io/plane/state"` | `offline_terrain_display` 模式下订阅的载机状态话题 |
| `subscribe_topic` | `string` | `"/flow/aligned_mesh"` | 输入三维网格主题 |
| `fps_logging_enabled` | `bool` | `true` | 是否在终端输出本地渲染帧率日志 |
| `fps_logging_interval_sec`| `double`| `5.0` | 帧率日志输出周期（秒） |
| `camera_ids` | `string[]` | `["cam_front"]` | Open3D 模式下激活的视口列表（支持单视口或多视口） |
| `<cam_id>.width` / `height` | `int` | `800` / `600` | 视口窗口像素宽高 |
| `<cam_id>.front` | `double[3]`| `[0, -1, 0.35]` | 相机视线方向朝向向量 |
| `<cam_id>.lookat` | `double[3]`| `[-17.5, 2213, -86]`| 相机三维注视中心目标点 |
| `<cam_id>.up` | `double[3]`| `[0, 0.35, 1]` | 相机正上方朝向向量 |
| `<cam_id>.zoom` | `double` | `0.12` | 相机视场缩放系数（数值越小视野越开阔） |

### 5.2 Open3D 传送带流动动画参数

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `animation_enabled` | `bool` | `true` | 是否启用传送带平滑滑动动画（`offline_terrain_display: false` 时生效） |
| `animation_fps` | `double` | `30.0` | 动画内部渲染目标帧率（Hz） |
| `animation_look_dist` | `double` | `400.0` | 视点前瞻注视距离（米） |
| `animation_view_ahead` | `double` | `1200.0` | 前向有效挂载与可视距离（米） |
| `animation_view_behind` | `double` | `120.0` | 后向可视保留距离（米） |
| `animation_visible_radius` | `double`| `300.0` | 以注视点为中心的外扩可见半径 |
| `animation_segment_sec` | `double` | `0.5` | 相邻两帧之间完成推进动画的时长 |
| `animation_buffer_size` | `int` | `20` | 最大缓存历史网格帧数 |
| `animation_depth_offset` | `double` | `0.02` | 重叠面 Z 轴抗闪烁错位量（米） |

### 5.3 DIS 实时光流插帧参数 ([`v_dis_ultrafast_config.yaml`](../src/visualization/config/v_dis_ultrafast_config.yaml))

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `interpolation_enabled` | `bool` | `false` | 是否开启 DIS 稠密光流插帧模式 |
| `interpolation_intermediate_frames` | `int` | `4` | 两真实帧之间插入的虚拟过渡帧数（4 表示提升至约 5 倍帧率） |
| `interpolation_flow_scale` | `double` | `0.25` | 计算光流时的分辨率下采样比例（0.25 加快 16 倍光流求解） |
| `interpolation_dis_preset` | `string` | `"ultrafast"` | DIS 质量预设（`"ultrafast"`, `"fast"`, `"medium"`） |
| `interpolation_bidirectional_flow` | `bool` | `false` | 是否启用前向+后向双向光流融合（更精确但计算量翻倍） |
| `interpolation_lock_camera` | `bool` | `true` | 插帧过程中是否锁定视口相机姿态以防画面撕裂 |
| `interpolation_show_source_window` | `bool` | `false` | 是否同时显示原始未插帧的源渲染窗口（用于调试） |

---

## 6. 常用配置与启动脚本

功能包针对不同渲染策略预置了四套开箱即用的启动脚本：

### 6.1 常规三维渲染（主流水线默认）：[`v_vis.launch.py`](../src/visualization/launch/v_vis.launch.py)
加载 [`v_config.yaml`](../src/visualization/config/v_config.yaml)，默认启用离线 GPS 跟随、累计覆盖去重与远景渐变渲染模式。

### 6.2 DIS 极速 5 倍插帧模式：[`v_dis_ultrafast.launch.py`](../src/visualization/launch/v_dis_ultrafast.launch.py)
采用极速单向 DIS 光流与 0.25 尺度缩放下采样，低延迟将帧率提升 5 倍。

### 6.3 DIS 20 倍超高帧率模式：[`v_dis_quality_x20.launch.py`](../src/visualization/launch/v_dis_quality_x20.launch.py)
插入 19 张虚拟帧，在算力充裕环境下呈现电影级细腻视觉。

### 6.4 DIS 双向高质量插帧模式：[`v_dis_bidirectional.launch.py`](../src/visualization/launch/v_dis_bidirectional.launch.py)
采用前后双向光流对称一致性融合，显著抑制大范围剧烈视点变动引发的空洞瑕疵。

---

## 7. 启动与使用命令参考

```bash
# 1. 常规启动默认三维白模可视化节点（单机流水线默认）
ros2 launch visualization v_vis.launch.py

# 2. 启动 DIS 极速 5 倍光流插帧可视化
ros2 launch visualization v_dis_ultrafast.launch.py

# 3. 启动 20 倍电影级超高帧率平滑模式
ros2 launch visualization v_dis_quality_x20.launch.py

# 4. 动态更改订阅话题（例如直接查看未配准的原始白模）
ros2 launch visualization v_vis.launch.py subscribe_topic:=/reconstruction/white_mesh
```

---

## 8. 文件结构与索引

```
src/visualization/
├── CMakeLists.txt                                  # 构建描述 (链接 Open3D, OpenCV, X11, Threads)
├── package.xml                                     # ROS 2 包依赖
├── config/
│   ├── v_config.yaml                               # 离线地形展示、三维交互与传送带动画参数
│   ├── v_dis_ultrafast_config.yaml                 # DIS 5倍极速插帧配置
│   ├── v_dis_quality_x20_config.yaml               # DIS 20倍超高刷新配置
│   └── v_dis_bidirectional_config.yaml             # DIS 双向融合配置
├── include/
│   └── visualization/
│       ├── dis_frame_interpolator.hpp              # DIS 光流插帧计算头文件
│       └── native_frame_window.hpp                 # Linux 原生 X11 极速视窗头文件
├── launch/
│   ├── v_vis.launch.py                             # 常规可视化启动脚本
│   ├── v_dis_ultrafast.launch.py                   # 5倍极速插帧启动脚本
│   ├── v_dis_quality_x20.launch.py                 # 20倍超分辨率启动脚本
│   └── v_dis_bidirectional.launch.py               # 双向光流插帧启动脚本
└── src/
    ├── visualization_node.cpp                      # 节点主逻辑、导航配对、相机视角与渲染调度
    ├── offline_terrain_display.hpp                 # 累计覆盖去重、软光栅器与远景渐变渲染实现
    ├── dis_frame_interpolator.cpp                  # DIS 光流估计与 Warping 图像生成实现
    └── native_frame_window.cpp                     # 原生 X11 窗口极速渲染后端实现
```
