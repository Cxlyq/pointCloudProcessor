# visualization 节点说明文档

`visualization` 是点云处理流水线的末端三维高保真图形渲染与人机交互展示功能包。它接收来自上游的白模三角网格（经配准的 `/flow/aligned_mesh` 或未配准的 `/reconstruction/white_mesh`），提供基于 Open3D 的多机位视口渲染、传送带式（Conveyor Belt）多帧平滑移动拼接动画渲染，以及基于密集逆搜索（DIS, Dense Inverse Search）光流的多倍率超实时视频插帧呈现，使低频离散白模在操作界面上呈现出丝滑连贯的高清数字孪生视觉效果。

---

## 1. 节点概览与设计目标

| 属性 | 内容 |
| :--- | :--- |
| **功能包名称** | `visualization` |
| **可执行文件** | `visualization_node` |
| **主 C++ 类** | `VisualizationNode` ([visualization_node.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/src/visualization_node.cpp))<br>`DisFrameInterpolator` ([dis_frame_interpolator.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/src/dis_frame_interpolator.cpp))<br>`NativeFrameWindow` ([native_frame_window.cpp](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/src/native_frame_window.cpp)) |
| **核心依赖** | ROS 2 (`rclcpp`, `std_msgs`, `geometry_msgs`), `pc_msgs`, `Open3D`, `OpenCV` (core, highgui, imgproc, video), `X11`, `Threads` |

### 核心设计目标：
1. **多机位并行视口与视场预设**：支持按 YAML 配置并发开启一个或多个独立视口（如正面跟随视角 `cam_front`、全局上帝视角 `cam_top`），精确指定机位三维相机参数（`lookat`, `front`, `up`, `zoom`）。
2. **传送带式平滑流动动画 (Conveyor Animation)**：在对齐模式下，结合上游 `/flow/registration_stats` 配准位姿序列，将多帧网格组织为时间滑动窗口，以 30 FPS 高帧率在空间中匀速平滑流动装载与卸载，避免传统逐帧硬切换带来的剧烈闪烁与视觉卡顿。
3. **基于 DIS 稠密光流的 2D 实时极速超分辨率插帧**：针对计算资源受限场景，集成 OpenCV DIS（Dense Inverse Search）算法，在捕获的离线渲染帧序列之间实时生成 4~19 帧合成运动平滑过渡帧，将 0.5~2 Hz 的原始低频刷新瞬间提升至 30~60 Hz 电影级丝滑显示。
4. **原生 X11 / HighGUI 双后端加速显示**：针对 Linux 桌面环境，提供基于 X11 原生共享内存与双缓冲的高性能显示后端，保证高分辨率图像零拷贝刷屏。

---

## 2. 核心机制与架构

### 2.1 整体架构图

```
      /flow/aligned_mesh (O3DMesh)              /flow/registration_stats (统计/位姿)
                   │                                             │
                   └──────────────────────┬──────────────────────┘
                                          │
                                          ▼
                      ┌───────────────────────────────────────┐
                      │          visualization_node           │
                      │                                       │
                      │  ┌─────────────────────────────────┐  │
                      │  │   模式 A: 传送带流动平滑动画     │  │
                      │  │   * 环形缓冲维护 (FIFO)         │  │
                      │  │   * 视锥前向动态剔除与装载      │  │
                      │  │   * Z-Fighting 深度微小微调     │  │
                      │  └─────────────────────────────────┘  │
                      │  ┌─────────────────────────────────┐  │
                      │  │   模式 B: DIS 稠密光流极速插帧  │  │
                      │  │   * 渲染帧后台图像抓取          │  │
                      │  │   * DIS 多尺度光流场解算        │  │
                      │  │   * 双向加权 Warping 帧生成     │  │
                      │  │   * Native X11 极速呈现 (x5~x20)│  │
                      │  └─────────────────────────────────┘  │
                      └───────────────────┬───────────────────┘
                                          │
                                          ▼
                               [Open3D 视窗 / X11 交互窗口]
```

### 2.2 传送带式平滑动画渲染原理
当启用 `animation_enabled: true` 时：
1. **多帧缓冲区维护**：维持一个容量为 `animation_buffer_size`（如 20 帧）的滑动网格库。
2. **视锥空间范围动态裁剪**：以当前虚拟巡航相机为中心，计算 `animation_view_ahead`（前向可视距离）与 `animation_view_behind`（后向可视距离），仅将位于可视范围内的白模挂载到 Open3D 场景中，超出视锥的模型自动卸载，恒定显存与渲染负荷。
3. **Z-Fighting 重叠面防闪烁**：两帧白模拼接过渡处存在重合面，节点对相邻帧施加细微的 Z 轴分层微调偏移（`animation_depth_offset: 0.02m`），彻底根除 OpenGL 深度缓冲竞争引起的闪烁。

### 2.3 DIS 稠密光流插帧机制
当启用 `interpolation_enabled: true` 时：
1. **渲染图像采集**：Open3D 作为后台无缝渲染器渲染真实几何帧，并将像素流无缝转为 OpenCV `cv::Mat`。
2. **DIS 稠密光流预测**：采用高效的密集逆搜索算法（`cv::DISOpticalFlow`，可选 `ultrafast` / `fast` / `medium` 档位），在连续两张图像之间快速求解像素运动矢量场。
3. **运动变形融合 (Forward/Backward Warping)**：根据插入时间点 $t \in (0, 1)$，结合光流场对两帧图像进行双向重采样与线性加权混合，消除运动伪影与残影，通过 `NativeFrameWindow` 以指定倍率输出。

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
| `subscribe_topic` (默认 `/flow/aligned_mesh`) | [pc_msgs/msg/O3DMesh](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/pc_msgs/msg/O3DMesh.msg) | 待呈现的三角网格（经配准的 `/flow/aligned_mesh` 或未配准的 `/reconstruction/white_mesh`） |
| `animation_stats_topic` (默认 `/flow/registration_stats`) | `std_msgs/msg/Float64MultiArray` | 传送带动画模式下参考的位姿与统计指标 |

---

## 5. 参数配置说明

配置文件位于 [`src/visualization/config/`](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/config/) 目录下。

### 5.1 基础与相机参数 ([v_config.yaml](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/config/v_config.yaml))

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `subscribe_topic` | `string` | `"/flow/aligned_mesh"` | 输入三维网格主题 |
| `fps_logging_enabled` | `bool` | `true` | 是否在终端输出本地渲染帧率日志 |
| `fps_logging_interval_sec`| `double`| `5.0` | 帧率日志输出周期（秒） |
| `camera_ids` | `string[]` | `["cam_front"]` | 激活的视口列表（支持单视口或多视口） |
| `<cam_id>.width` / `height` | `int` | `800` / `600` | 视口窗口像素宽高 |
| `<cam_id>.front` | `double[3]`| `[0, -1, 0.35]` | 相机视线方向朝向向量 |
| `<cam_id>.lookat` | `double[3]`| `[-17.5, 2213, -86]`| 相机三维注视中心目标点 |
| `<cam_id>.up` | `double[3]`| `[0, 0.35, 1]` | 相机正上方朝向向量 |
| `<cam_id>.zoom` | `double` | `0.12` | 相机视场缩放系数（数值越小视野越开阔） |

### 5.2 传送带流动动画专属参数

| 参数名称 | 类型 | 默认值 | 参数功能说明 |
| :--- | :--- | :--- | :--- |
| `animation_enabled` | `bool` | `true` | 是否启用传送带平滑滑动动画 |
| `animation_fps` | `double` | `30.0` | 动画内部渲染目标帧率（Hz） |
| `animation_look_dist` | `double` | `400.0` | 视点前瞻注视距离（米） |
| `animation_view_ahead` | `double` | `1200.0` | 前向有效挂载与可视距离（米） |
| `animation_view_behind` | `double` | `120.0` | 后向可视保留距离（米） |
| `animation_visible_radius` | `double`| `300.0` | 以注视点为中心的外扩可见半径 |
| `animation_segment_sec` | `double` | `0.5` | 相邻两帧之间完成推进动画的时长 |
| `animation_buffer_size` | `int` | `20` | 最大缓存历史网格帧数 |
| `animation_depth_offset` | `double` | `0.02` | 重叠面 Z 轴抗闪烁错位量（米） |

### 5.3 DIS 实时光流插帧参数 ([v_dis_ultrafast_config.yaml](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/config/v_dis_ultrafast_config.yaml))

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

### 6.1 常规三维渲染（主流水线默认）：[v_vis.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/launch/v_vis.launch.py)
加载 `v_config.yaml`，提供具备传送带流动平滑动画的三维交互查看器。

### 6.2 DIS 极速 5 倍插帧模式：[v_dis_ultrafast.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/launch/v_dis_ultrafast.launch.py)
采用极速单向 DIS 光流与 0.25 尺度缩放下采样，低延迟将帧率提升 5 倍。

### 6.3 DIS 20 倍超高帧率模式：[v_dis_quality_x20.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/launch/v_dis_quality_x20.launch.py)
插入 19 张虚拟帧，在算力充裕环境下呈现电影级细腻视觉。

### 6.4 DIS 双向高质量插帧模式：[v_dis_bidirectional.launch.py](file:///home/asahichinn/Documents/codes/pointCloudProcessor/src/visualization/launch/v_dis_bidirectional.launch.py)
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
│   ├── v_config.yaml                               # 基础三维交互与传送带动画参数
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
    ├── visualization_node.cpp                      # 节点主逻辑、相机视角与传送带动画管理
    ├── dis_frame_interpolator.cpp                  # DIS 光流估计与 Warping 图像生成实现
    └── native_frame_window.cpp                     # 原生 X11 窗口极速渲染后端实现
```
